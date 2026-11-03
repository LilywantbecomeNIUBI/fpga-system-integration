"""Frozen-source Ethernet build. Default: static sensor diagnostic, never programs.

--system builds the full M2 camera/navigation candidate with Ethernet telemetry.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import os
import re
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from build import ROOT, m2_rows, base_constraints, sources as system_sources


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(system=False,raw=False,place=2,route=0,auto_clock=False,replicate=1,clock_cleanup=False,mecanum=False):
    if mecanum and not system:raise ValueError('Mecanum requires --system')
    top='system_mecanum_top' if mecanum else 'system_task4_top' if system else 'm2_sensor_eth_diag_top'
    out=ROOT/('build/lidar_raw_ethernet_20261102' if raw else 'build/lidar_ethernet_20261102')/('mecanum' if mecanum else 'system' if system else 'diagnostic')/time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
    # Freeze-only checks and an actual build can start in the same second.
    stem=out
    for attempt in range(100):
        try:out.mkdir(parents=True,exist_ok=False);break
        except FileExistsError:out=stem.with_name(stem.name+'_'+str(attempt+1))
    else:raise RuntimeError('Unable to allocate a unique frozen build directory')
    if system:
        files=system_sources()
        if mecanum:files+=list((ROOT/'fpga/mecanum').glob('*.sv'))
    else:
        files=list((ROOT/'fpga/motor').glob('*.sv'))
        files += [ROOT/'fpga/navigation'/p for p in ('task4_chassis.sv','nav_command_mux.sv','nav_status_source.sv','lidar_scan_360_source.sv','lidar_points_source.sv')]
        files += list((ROOT/'fpga/navigation/lidar').glob('*.v'))
        files += [ROOT/'fpga/system'/p for p in ('imu_status_source.sv','telemetry_cdc.sv','sensor_ethernet.v')]
        files += [ROOT/'fpga/diagnostics/m2_sensor_eth_diag_top.sv']
        files += [ROOT/'fpga/imu'/p for p in ('imu_1khz_core.v','mpu6050_reader.v','imu_fast_i2c_master.v')]
        files += [ROOT/'fpga/vision/src'/p for p in ('reset_sync.v','ethernet_pll/ethernet_pll.v','rgmii_input.v','rgmii_output.v','network_runtime_dual.v')]
        files += list((ROOT/'fpga/vision/src/mdio').glob('*.v'))
        files += list((ROOT/'fpga/vision/src/eth_udp_gmii').glob('*.v'))
    files=sorted(set(files));hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    for p in files:
        dest=out/'source'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
    if any(sha(ROOT/p)!=h for p,h in hashes.items()):raise RuntimeError('Sources changed during snapshot')
    variant=out/'source'/('fpga/mecanum/system_mecanum_top.sv' if mecanum else 'fpga/navigation/system_task4_top.sv' if system else 'fpga/diagnostics/m2_sensor_eth_diag_top.sv')
    variant.write_text(re.sub(r'LIDAR_RAW=[01]','LIDAR_RAW='+str(int(raw)),variant.read_text()))
    hashes[variant.relative_to(out/'source').as_posix()]=sha(variant)
    with (ROOT/'hardware/pins.csv').open(encoding='utf-8-sig') as table:
        rows=m2_rows(list(csv.DictReader(table)))
    if mecanum:
        import sys
        sys.path.insert(0,str(ROOT))
        from tools.mecanum.build import constraints
        rows,cst,sdc=constraints()
    elif system:cst,sdc=base_constraints(rows)
    else:
        names={'clk50m','reset_n','uart_tx','imu_i2c_scl','imu_i2c_sda','left_in1','left_in2','right_in1','right_in2',
               'left_enc_a','left_enc_b','right_enc_a','right_enc_b','lidar_rx','legacy_m1_disable','legacy_cal_disabled[0]'}
        rows=[r for r in rows if r['signal'] in names or r['group']=='ethernet' or r['signal'].startswith(('led[','legacy_m1_pwm_disabled['))]
        ref=(ROOT/'vendor/reference/robot.cst').read_text(encoding='utf-8')
        ports={signal:statement for statement,signal in re.findall(r'(IO_PORT "([^"]+)"[^;]+;)',ref)}
        lines=[]
        for r in rows:
            name=r['signal'];pull='UP' if name in ('lidar_rx','left_enc_a','left_enc_b','right_enc_a','right_enc_b') else 'DOWN' if name.startswith('legacy_') or name in ('left_in1','left_in2','right_in1','right_in2') else 'NONE'
            port=ports.get(name) if r['group']=='ethernet' or name=='clk50m' else None
            lines += ['IO_LOC "%s" %s;'%(name,r['ball']),port or 'IO_PORT "%s" IO_TYPE=LVCMOS33 PULL_MODE=%s BANK_VCCIO=3.3;'%(name,pull)]
        cst='\n'.join(lines)+'\n'
        sdc='''create_clock -name sys50 -period 20 [get_ports {clk50m}]
create_generated_clock -name phy50 -source [get_ports {clk50m}] -divide_by 1 [get_pins {network/pll/PLL_inst/CLKOUT0}]
create_generated_clock -name eth125 -source [get_ports {clk50m}] -multiply_by 5 -divide_by 2 [get_pins {network/pll/PLL_inst/CLKOUT1}]
create_clock -name eth_rx125 -period 8 [get_ports {rgmii_rxc}]
set_clock_groups -asynchronous -group [get_clocks {eth_rx125}] -group [get_clocks {sys50 phy50 eth125}]
set_false_path -from [get_ports {reset_n}]
set_input_delay -clock eth_rx125 -max 2.2 [get_ports {rgmii_rxd[*] rgmii_rxctl}]
set_input_delay -clock eth_rx125 -min 1.8 [get_ports {rgmii_rxd[*] rgmii_rxctl}]
set_input_delay -clock eth_rx125 -clock_fall -add_delay -max 2.2 [get_ports {rgmii_rxd[*] rgmii_rxctl}]
set_input_delay -clock eth_rx125 -clock_fall -add_delay -min 1.8 [get_ports {rgmii_rxd[*] rgmii_rxctl}]
'''
        for p in ('imu/reader/bus/scl_sync_0_s0','imu/reader/bus/sda_sync_0_s0','chassis/core/enc_l/ab_meta_0_s0','chassis/core/enc_l/ab_meta_1_s0',
                  'chassis/core/enc_r/ab_meta_0_s0','chassis/core/enc_r/ab_meta_1_s0','lidar_uart/rx_meta_s0'):
            sdc+='set_false_path -to [get_pins {%s/D}]\n'%p
    bridge='camera/telemetry_enabled.telemetry_bridge' if system else 'network/telemetry_bridge'
    # Destination data is captured only after the synchronized request. The
    # source cannot change held until acknowledgement returns through two source
    # flops. A same-edge hold relationship does not describe this handshake.
    # Keep the real setup/max-delay bound; waive only that bus's false hold edge.
    if not system:
        sdc+='\n# Immutable held bus: at least two receiving cycles precede capture.\n'
        sdc+='set_max_delay -from [get_pins {%s/held*/Q}] -to [get_pins {%s/dst_payload*/D}] 10.0\n'%(bridge,bridge)
        sdc+='set_false_path -hold -from [get_pins {%s/held*/Q}] -to [get_pins {%s/dst_payload*/D}]\n'%(bridge,bridge)
        sdc+='set_false_path -to [get_pins {%s/ack_meta*/D %s/request_meta*/D}]\n'%(bridge,bridge)
        for reset in ['network/sys_reset','network/source_reset','network/network_reset','network/receive_reset']:
            sdc+='set_false_path -to [get_pins {%s/release_pipe*/CLEAR}]\n'%reset
    sdc+='report_exceptions -setup -from [get_pins {%s/held*/Q}] -max_paths 600\n'%bridge
    if len(rows)!=len({r['ball'] for r in rows}) or len(rows)!=len({r['signal'] for r in rows}):raise RuntimeError('Pin collision')
    mapping={r['signal']:r['ball'] for r in rows}
    for p in json.loads((ROOT/('hardware/mecanum.json' if mecanum else 'hardware/task4_m2.json')).read_text())['pins']:
        if mapping.get(p['signal'])!=p['ball']:raise RuntimeError('Chassis pin mismatch')
    if auto_clock:
        # Let the router select a clock network instead of carrying forward the
        # legacy PCLK assignment which produced PR1014. No timing waiver.
        cst=re.sub(r'^CLOCK_LOC "camera_pclk_d"[^\n]*\n?','',cst,flags=re.M)
    if system:
        # Keep the unrestricted worst paths as well as each real clock domain.
        # Do not change clock periods or waive any failing setup/pulse paths.
        sdc+='\nreport_timing -setup -max_paths 50\n'
        for clock in ('sys50','cam_pclk','eth_rx125','control50','eth125'):
            paths=1500 if clock=='eth125' else 400 if clock=='sys50' else 50
            sdc+='report_timing -setup -from_clock [get_clocks {%s}] -to_clock [get_clocks {%s}] -max_paths %d\n'%(clock,clock,paths)
        # Report both halves of the bank-local read pipeline even when other
        # modules dominate the global worst-path sample. Report only; no waiver.
        sdc+='report_timing -setup -to [get_pins {camera/stream/storage_bank*.read_pixel_pipe*/D}] -max_paths 100\n'
        sdc+='report_timing -setup -to [get_pins {camera/stream/cached_word*/D}] -max_paths 32\n'
        sdc+='report_timing -setup -to [get_pins {camera/stream/frame_address*/D}] -max_paths 64\n'
        sdc+='report_timing -setup -to [get_pins {camera/stream/storage_bank*.read_address_pipe*/D}] -max_paths 128\n'
        sdc+='report_timing -setup -to [get_pins {camera/runtime_network.net/retry_count*/*}] -max_paths 256\n'
        if raw:
            sdc+='report_timing -setup -from [get_pins {ethernet_raw.lidar_display/packet_ram*/*}] -max_paths 128\n'
            sdc+='report_timing -setup -to [get_pins {ethernet_raw.lidar_display/active_points*/D}] -max_paths 128\n'
    if clock_cleanup:
        if not system:raise ValueError('Clock cleanup is only for the full system trial')
        # SUG1018 LOCAL_CLOCK excludes non-clock control nets from clock trees.
        # Use stable RTL net names, never autogenerated synthesis node numbers.
        controls=['reset_sync','camera/sys_rst_n','camera/eth_rst_stream',
                    'camera/runtime_network.network_rst_n']
        # Mecanum snapshot capture is absorbed/renamed by synthesis; do not add
        # an unverified hierarchical name merely because it exists in RTL.
        if not mecanum:controls+=['auxiliary_valid','chassis/telemetry/diagnostic_enabled.diagnostics/capture']
        # Raw mode has capture_crc, not the sector assembler's capture signal.
        controls += ['ethernet_raw.lidar_display/capture_crc' if raw else 'ethernet_scan.lidar_display/capture']
        for net in controls:
            cst+='CLOCK_LOC "%s" LOCAL_CLOCK;\n'%net
        for net in ('clk50m_d','rgmii_rxc_d','camera_pclk_d','camera/clk50','camera/clk125'):
            cst=re.sub(r'^CLOCK_LOC "'+re.escape(net)+r'"[^\n]*\n?','',cst,flags=re.M)
            cst+='CLOCK_LOC "%s" BUFG = CLK;\n'%net
    (out/'sensor.cst').write_text(cst,encoding='utf-8');(out/'sensor.sdc').write_text(sdc,encoding='utf-8')
    project=ET.Element('Project');ET.SubElement(project,'Template').text='FPGA';ET.SubElement(project,'Version').text='5'
    ET.SubElement(project,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012';fl=ET.SubElement(project,'FileList')
    for p in [out/'source'/p.relative_to(ROOT) for p in files]+[out/'sensor.cst',out/'sensor.sdc']:
        ET.SubElement(fl,'File',path=p.relative_to(out).as_posix(),type='file.'+('verilog' if p.suffix in ('.v','.sv') else p.suffix[1:]),enable='1')
    ET.ElementTree(project).write(out/'sensor.gprj',encoding='utf-8',xml_declaration=True)
    tcl='set here [file dirname [file normalize [info script]]]\ncd $here\nopen_project sensor.gprj\n'
    for k,v in [('top_module',top),('verilog_std','sysv2017'),('output_base_name','sensor'),('max_fanout','32' if system else '16'),('replicate_resources',str(replicate)),('timing_driven','1'),('retiming_resource','bsram'),('use_sspi_as_gpio','1'),('use_cpu_as_gpio','1'),('place_option',str(place)),('route_option',str(route))]:
        tcl+='set_option -%s %s\n'%(k,v)
    (out/'build.tcl').write_text(tcl+'run all\nexit\n')
    record=dict(top=top,telemetry_transport='udp',telemetry_port=6205,lidar_bins=None if raw else 360,
                lidar_mode='raw_points' if raw else 'nearest_return_per_sector',pin_profile='mecanum_p7' if mecanum else 'task4_m2',place_option=place,route_option=route,auto_clock_routing=auto_clock,
                replicate_resources=replicate,clock_cleanup=clock_cleanup,
                source_sha256=hashes,programmed=False,hardware_tested=False,system=system,
                camera_enabled=system,navigation_enabled=system and not mecanum,mecanum_pose_enabled=mecanum,command_receiver_enabled=system,command_transport='udp6206_v3' if mecanum else 'uart')
    (out/'pin_audit.json').write_text(json.dumps(dict(passed=True,pins=rows),indent=2))
    (out/'manifest.json').write_text(json.dumps(record,indent=2))
    return out,record


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--system',action='store_true');parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--mecanum',action='store_true',help='Use teammate four-wheel pins and integrated Mecanum top (requires --system)')
    mode=parser.add_mutually_exclusive_group();mode.add_argument('--sector360',action='store_true');mode.add_argument('--raw-points',action='store_true',help='Explicit opt-in; the default is the restored 360-point mode')
    parser.add_argument('--place',type=int,choices=(2,3,4),default=2);parser.add_argument('--route-option',type=int,choices=(0,1,2),default=0)
    parser.add_argument('--replicate-resources',type=int,choices=(0,1),default=1)
    parser.add_argument('--clock-cleanup',action='store_true',help='Experimental: keep controls off clock trees and reserve real clocks')
    parser.add_argument('--incremental-from',type=Path,help='Complete whole-system build directory or .pr; defaults to config/pnr.json')
    parser.add_argument('--auto-clock-routing',action='store_true');parser.add_argument('--timeout-seconds',type=int,default=5400);args=parser.parse_args()
    if not args.system and not args.prepare_only:parser.error('Use --system: board builds must use whole-system incremental PnR')
    out,record=prepare(args.system,raw=args.raw_points,place=args.place,route=args.route_option,auto_clock=args.auto_clock_routing,replicate=args.replicate_resources,clock_cleanup=args.clock_cleanup,mecanum=args.mecanum);print(out,flush=True)
    if args.system:
        from incremental import attach
        baseline=args.incremental_from
        if baseline is None:
            config=json.loads((ROOT/'config/pnr.json').read_text())
            baseline=config.get('incremental_base')
        if not baseline:raise SystemExit('No whole-system .pr baseline configured; no full rebuild fallback')
        baseline=Path(baseline)
        if not baseline.is_absolute():baseline=ROOT/baseline
        record=attach(out,baseline)
        print('Whole-system incremental PnR reference: '+str(baseline),flush=True)
    if args.prepare_only:return
    active=subprocess.run(['powershell','-NoProfile','-Command','Get-Process gw_sh,gw_syn,gw_pnr -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id'],capture_output=True,text=True) if os.name=='nt' else None
    if active and active.stdout.strip():raise SystemExit('Another Gowin build is running; frozen snapshot prepared only')
    exe=Path(os.environ.get('GOWIN_HOME','C:/Gowin/Gowin_V1.9.12.04_x64'))/'IDE/bin/gw_sh.exe'
    start=time.monotonic();finished=None;terminated=False
    with (out/'build.log').open('w',encoding='utf-8') as log:
        proc=subprocess.Popen([str(exe),str(out/'build.tcl')],cwd=out,stdout=log,stderr=subprocess.STDOUT)
        while proc.poll() is None:
            time.sleep(2)
            text=(out/'build.log').read_text(encoding='utf-8',errors='replace')
            complete='[100%] Power analysis completed' in text and 'sensor.tr" completed' in text and (out/'impl/pnr/sensor.fs').exists()
            if complete and finished is None:finished=time.monotonic()
            if finished is not None and time.monotonic()-finished>30:
                proc.terminate();proc.wait(timeout=15);terminated=True;break
            if time.monotonic()-start>args.timeout_seconds:
                proc.terminate();proc.wait(timeout=15)
                record.update(passed=False,timed_out=True,elapsed_s=time.monotonic()-start)
                (out/'result.json').write_text(json.dumps(record,indent=2))
                raise TimeoutError('Build timeout; frozen sources, failure metadata and log preserved')
    trace_path=out/'impl/pnr/sensor.tr';trace=trace_path.read_text(errors='replace') if trace_path.exists() else ''
    text=(out/'build.log').read_text(encoding='utf-8',errors='replace')
    violations={k.lower():int(m[1]) if (m:=re.search(r'<Numbers of '+k+r' Violated Endpoints>:(\d+)',trace)) else None for k in ('Setup','Hold')}
    pulse=[float(x) for x in re.findall(r'MPW Summary:\s+Slack:\s+([-\d.]+)',trace)]
    warnings=[line for line in text.splitlines() if 'PR1014' in line]
    net_path=out/'impl/gwsynthesis/sensor.vg';net=net_path.read_text(errors='replace') if net_path.exists() else '';pads={}
    for body in re.findall(r'\bOBUF\s+\S+\s*\((.*?)\);',net,re.S):
        o=re.search(r'\.O\(([^)]+)\)',body);i=re.search(r'\.I\(([^)]+)\)',body)
        if o and i and (o[1].startswith('legacy_') or o[1] in ('left_in1','left_in2','right_in1','right_in2')):pads[o[1]]=i[1]
    static=not args.system and len(pads)==12 and set(pads.values())=={'GND'}
    record.update(elapsed_s=time.monotonic()-start,actual_exit_code=proc.returncode,cli_terminated_after_completed_outputs=terminated,
                  violations=violations,min_pulse_width_slack_ns=min(pulse) if pulse else None,
                  clock_route_warnings=warnings,motor_pads=pads,motor_outputs_constant_low=static)
    bit=out/'impl/pnr/sensor.fs'
    if bit.exists():record['bitstream_sha256']=sha(bit)
    record['frozen_sources_unchanged']=all(sha(out/'source'/p)==h for p,h in record['source_sha256'].items())
    if args.system:
        reference=record['incremental_pnr']
        reference['reference_unchanged_after_run']=sha(out/reference['reference_path'])==reference['reference_sha256']
        if not reference['reference_unchanged_after_run']:raise RuntimeError('Incremental reference changed during PnR')
        command=out/'impl/pnr/cmd.do'
        reference['tool_command_evidence']=[line for line in command.read_text(errors='replace').splitlines() if 'inc' in line.lower() or 'sensor.pr' in line] if command.exists() else []
    record['passed']=(proc.returncode==0 or terminated) and violations==dict(setup=0,hold=0) and bool(pulse) and min(pulse)>=0 and not warnings and bit.exists() and record['frozen_sources_unchanged'] and (args.system or static)
    if args.system and net_path.exists():
        audits={}
        for name in ('control50','lk_state'):
            output=out/(name+'_audit.json')
            checked=subprocess.run([os.sys.executable,str(ROOT/'tools'/('audit_'+name+'.py')),
                '--netlist',str(net_path),'--output',str(output)],cwd=ROOT,capture_output=True,text=True)
            (out/(name+'_audit.log')).write_text(checked.stdout+checked.stderr,encoding='utf-8')
            audits[name]=checked.returncode==0
        checked=subprocess.run([os.sys.executable,str(ROOT/'tools/audit_control50_timing.py'),str(out),'--basename','sensor'],cwd=ROOT,capture_output=True,text=True)
        (out/'timing_audit.log').write_text(checked.stdout+checked.stderr,encoding='utf-8')
        audit_path=out/'timing_audit.json'
        audits['cdc_timing']=checked.returncode==0 and audit_path.exists() and json.loads(audit_path.read_text()).get('passed') is True
        checked=subprocess.run([os.sys.executable,str(ROOT/'tools/task4/audit_frame_read_timing.py'),str(out)],cwd=ROOT,capture_output=True,text=True)
        (out/'frame_read_timing_audit.log').write_text(checked.stdout+checked.stderr,encoding='utf-8')
        audit_path=out/'frame_read_timing_audit.json'
        audits['frame_read_setup']=checked.returncode==0 and audit_path.exists() and json.loads(audit_path.read_text()).get('passed') is True
        record['system_audits']=audits;record['passed']=record['passed'] and all(audits.values())
    (out/'result.json').write_text(json.dumps(record,indent=2))
    # Advance only to a complete whole-system route that passes setup/hold and
    # the functional timing audits. Keep the accepted MPW warning explicit.
    if args.system and proc.returncode==0 and violations==dict(setup=0,hold=0) and record['frozen_sources_unchanged'] and record.get('system_audits') and all(record['system_audits'].values()):
        config_path=ROOT/'config/pnr.json';config=json.loads(config_path.read_text())
        config['incremental_pnr']=True;config['incremental_base']=out.relative_to(ROOT).as_posix()
        temporary=config_path.with_suffix('.tmp');temporary.write_text(json.dumps(config,indent=2)+'\n');temporary.replace(config_path)
    (out.parents[1]/'latest_build.txt').write_text(str(out))
    print(json.dumps({k:v for k,v in record.items() if k!='source_sha256'},indent=2),flush=True)
    raise SystemExit(0 if record['passed'] else 1)


if __name__=='__main__':main()
