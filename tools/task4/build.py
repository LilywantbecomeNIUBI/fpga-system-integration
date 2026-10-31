"""Isolated frozen-source build. Never programs hardware or edits build/system.

Full PnR uses the confirmed task4_m2 pin profile. --mode syn
can validate elaboration before wiring is decided. A timing harness separately
measures the new 50MHz kernel without borrowing an unconfirmed physical pin.
"""
from pathlib import Path
import argparse,csv,hashlib,json,os,re,shutil,subprocess,time,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def sources():
    paths=list((ROOT/'fpga/motor').glob('*.sv'))+list((ROOT/'fpga/navigation').rglob('*.sv'))+list((ROOT/'fpga/navigation').rglob('*.v'))
    paths+=list((ROOT/'fpga/system').glob('*.v'))+list((ROOT/'fpga/system').glob('*.sv'))
    paths+=list((ROOT/'fpga/vision/src').rglob('*.v'))+list((ROOT/'fpga/vision/src').rglob('*.sv'))
    paths+=[ROOT/'fpga/imu'/n for n in ('imu_1khz_core.v','icm42688_fast_reader.v','imu_fast_i2c_master.v')]
    return sorted(set(p for p in paths if p.name not in ('system_top.sv','camera_perception_top.v')))
def base_constraints(rows):
    """Generate locally; do not depend on an ignored build/system directory."""
    reference=(ROOT/'vendor/reference/robot.cst').read_text(encoding='utf-8')
    ports={signal:statement for statement,signal in re.findall(r'(IO_PORT "([^"]+)"[^;]+;)',reference)}
    names={'clk50m','reset_n','uart_rx','uart_tx','legacy_m1_disable','legacy_m2_disable','imu_i2c_scl','imu_i2c_sda'}
    selected=[r for r in rows if r['group'] in ('chassis','legacy_disabled','camera','ethernet') or r['signal'] in names or r['signal'].startswith('led[')]
    cst=[]
    for r in selected:
        pull='UP' if r['signal'] in ('uart_rx','lidar_rx','left_enc_a','left_enc_b','right_enc_a','right_enc_b') else 'DOWN' if r['group'] in ('chassis','legacy_disabled') else 'NONE'
        port=ports.get(r['signal']) if r['group'] not in ('chassis','legacy_disabled') else None
        cst.extend([f'IO_LOC "{r["signal"]}" {r["ball"]};',port or f'IO_PORT "{r["signal"]}" IO_TYPE=LVCMOS33 PULL_MODE={pull} BANK_VCCIO=3.3;'])
    cst+=re.findall(r'^CLOCK_LOC[^\n]+',reference,re.M)
    sdc=(ROOT/'vendor/reference/robot.sdc').read_text(encoding='utf-8').replace('imu/core/reader/','imu/reader/')
    sdc=sdc.replace('[get_nets {camera/clk50}]','[get_pins {camera/pll/PLL_inst/CLKOUT0}]').replace('[get_nets {camera/clk125}]','[get_pins {camera/pll/PLL_inst/CLKOUT1}]')
    for pin in ('chassis/rx/rx_meta_s0/D','chassis/core/enc_l/ab_meta_0_s0/D','chassis/core/enc_l/ab_meta_1_s0/D','chassis/core/enc_r/ab_meta_0_s0/D','chassis/core/enc_r/ab_meta_1_s0/D'):
        sdc+=f'set_false_path -to [get_pins {{{pin}}}]\n'
    return '\n'.join(cst)+'\n',sdc
def m2_rows(rows):
    profile=json.loads((ROOT/'hardware/task4_m2.json').read_text())
    mapping={p['signal']:p for p in profile['pins']}
    result=[]
    for row in rows:
        name=row['signal']
        if name=='legacy_m2_disable' or name.startswith('legacy_m2_pwm_disabled') or name=='legacy_cal_disabled[1]':continue
        if name in mapping:
            row=dict(row);row.update(ball=mapping[name]['ball'],p7=str(mapping[name]['p7']),group='chassis');result.append(row)
        elif row['ball'] not in {p['ball'] for p in profile['pins']}:result.append(row)
    p=mapping['lidar_rx'];result.append(dict(group='chassis',signal='lidar_rx',ball=p['ball'],direction='IN',p7=str(p['p7'])))
    return result

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=['prepare','syn','pnr'],default='prepare');ap.add_argument('--lidar-ball');ap.add_argument('--harness',action='store_true');ap.add_argument('--route-option',type=int,choices=[0,1,2],default=0);args=ap.parse_args()
    if args.harness and args.lidar_ball:ap.error('Harness does not use a LiDAR pin')
    if not args.harness and args.lidar_ball and args.lidar_ball.upper()!='B16':ap.error('M2 profile fixes lidar_rx at J_DRV2-14/B16')
    stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
    folder=ROOT/'build/task4'/('harness_' if args.harness else 'system_')/stamp;folder.mkdir(parents=True)
    files=sources();hashes={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    for p in files:
        dest=folder/'source'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dest)
    if any(sha(ROOT/p)!=h for p,h in hashes.items()):raise SystemExit('Sources changed during snapshot; retry after the other build finishes')
    rows=list(csv.DictReader((ROOT/'hardware/pins.csv').open(encoding='utf-8-sig')))
    if not args.harness:rows=m2_rows(rows)
    cst,sdc=base_constraints(rows)
    input_hashes={}
    for name in ('hardware/pins.csv','vendor/reference/robot.cst','vendor/reference/robot.sdc','hardware/task4_m2.json'):
        path=ROOT/name;dest=folder/'inputs'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,dest);input_hashes[name]=sha(dest)
    if args.harness:
        allowed={'clk50m','reset_n','uart_rx','uart_tx'}
        cst='\n'.join(line for line in cst.splitlines() if any(f'"{n}"' in line for n in allowed))+'\n'
        sdc='create_clock -name sys50 -period 20 [get_ports {clk50m}]\nset_false_path -from [get_ports {reset_n}]\n'
        sdc+='set_false_path -to [get_pins {rx/rx_meta_s0/D lidar_uart/rx_meta_s0/D}]\n'
        top='nav_timing_harness'
    else:
        top='system_task4_top'
        sdc+='set_false_path -to [get_pins {lidar_uart/rx_meta_s0/D}]\n'
    (folder/'task4.cst').write_text(cst,encoding='utf-8');(folder/'task4.sdc').write_text(sdc,encoding='utf-8')
    project=ET.Element('Project');ET.SubElement(project,'Template').text='FPGA';ET.SubElement(project,'Version').text='5'
    ET.SubElement(project,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012';fl=ET.SubElement(project,'FileList')
    for p in list((folder/'source').rglob('*.sv'))+list((folder/'source').rglob('*.v'))+[folder/'task4.cst',folder/'task4.sdc']:
        ET.SubElement(fl,'File',path=p.relative_to(folder).as_posix(),type={'.v':'file.verilog','.sv':'file.verilog','.cst':'file.cst','.sdc':'file.sdc'}[p.suffix],enable='1')
    ET.ElementTree(project).write(folder/'task4.gprj',encoding='utf-8',xml_declaration=True)
    tcl='set here [file dirname [file normalize [info script]]]\ncd $here\nopen_project task4.gprj\n'
    for k,v in [('top_module',top),('verilog_std','sysv2017'),('output_base_name','task4'),('max_fanout','32'),('replicate_resources','0'),('timing_driven','1'),('retiming_resource','bsram'),('use_sspi_as_gpio','1'),('use_cpu_as_gpio','1'),('place_option','2'),('route_option',str(args.route_option))]:tcl+=f'set_option -{k} {v}\n'
    (folder/'build.tcl').write_text(tcl+('run syn\n' if args.mode!='pnr' else 'run all\n')+'exit\n')
    record=dict(source_sha256=hashes,constraint_inputs_sha256=input_hashes,top=top,lidar_ball=None if args.harness else 'B16',pin_profile='harness' if args.harness else 'task4_m2',programmed=False,hardware_tested=False,harness=args.harness,mode=args.mode,route_option=args.route_option)
    (folder/'manifest.json').write_text(json.dumps(record,indent=2)+'\n');print(folder,flush=True)
    if args.mode=='prepare':return
    if os.name=='nt':
        active=subprocess.run(['powershell','-NoProfile','-Command','Get-Process gw_sh,gw_syn,gw_pnr -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id'],capture_output=True,text=True)
        if active.stdout.strip():raise SystemExit('Another Gowin build has priority; snapshot prepared, shared processes left untouched')
    exe=Path(os.environ.get('GOWIN_HOME','C:/Gowin/Gowin_V1.9.12.04_x64'))/'IDE/bin/gw_sh.exe'
    start=time.monotonic()
    completed_at=None;cleaned_up=False;timed_out=False
    with (folder/'build.log').open('w',encoding='utf-8') as log:
        proc=subprocess.Popen([str(exe),str(folder/'build.tcl')],cwd=folder,stdout=log,stderr=subprocess.STDOUT)
        while proc.poll() is None:
            time.sleep(1)
            text=(folder/'build.log').read_text(encoding='utf-8',errors='replace')
            complete=('GowinSynthesis finish' in text and (folder/'impl/gwsynthesis/task4_syn.rpt.html').exists()) if args.mode=='syn' else ('[100%] Power analysis completed' in text and (folder/'impl/pnr/task4.tr').exists())
            if complete and completed_at is None:completed_at=time.monotonic()
            # Some Windows CLI releases linger after all outputs. Terminate only
            # this Popen handle, and preserve the real exit code in the record.
            if completed_at is not None and time.monotonic()-completed_at>30:
                proc.terminate();proc.wait(timeout=15);cleaned_up=True;break
            if time.monotonic()-start>1200:
                proc.terminate();proc.wait(timeout=15);timed_out=True;break
    logtext=(folder/'build.log').read_text(encoding='utf-8',errors='replace')
    flow_completed=('GowinSynthesis finish' in logtext and (folder/'impl/gwsynthesis/task4.vg').exists()) if args.mode=='syn' else ('[100%] Power analysis completed' in logtext and (folder/'impl/pnr/task4.fs').exists())
    record.update(exit_code=proc.returncode,elapsed_s=time.monotonic()-start,log_sha256=sha(folder/'build.log'),tool_flow_completed=flow_completed,cli_terminated_after_completed_outputs=cleaned_up,timed_out=timed_out)
    timing=folder/'impl/pnr/task4_tr_content.html'
    if timing.exists():
        content=timing.read_text(encoding='utf-8',errors='replace')
        record['violations']={kind.lower():int(m[1]) if (m:=re.search(r'Numbers of '+kind+r' Violated Endpoints</td>\s*<td[^>]*>(\d+)',content)) else None for kind in ('Setup','Hold')}
        record['timing_sha256']=sha(timing)
        trace=(folder/'impl/pnr/task4.tr').read_text(encoding='utf-8',errors='replace')
        match=re.search(r'Slack\s*:\s*([-0-9.]+)',trace);record['worst_setup_slack_ns']=float(match[1]) if match else None
        pulse=trace.split('3.2 Minimum Pulse Width Table')[-1].split('3.3 Timing Report By Analysis Type')[0]
        widths=[float(m[1]) for m in re.finditer(r'^\s+\d+\s+([-0-9.]+)\s+[-0-9.]+\s+[-0-9.]+\s+(?:High|Low) Pulse Width',pulse,re.M)]
        record['pulse_width_passed']=bool(widths) and min(widths)>=0
        record['min_pulse_width_slack_ns']=min(widths) if widths else None
        logtext=(folder/'build.log').read_text(encoding='utf-8',errors='replace')
        record['clock_route_warnings']=[line for line in logtext.splitlines() if 'PR1014' in line]
        record['clock_routing_passed']=not record['clock_route_warnings']
    record['passed']=(proc.returncode==0 or cleaned_up) and flow_completed and not timed_out and not re.search(r'^ERROR\s*\(',logtext,re.M) and (args.mode=='syn' or (record.get('violations')==dict(setup=0,hold=0) and record.get('pulse_width_passed') and record.get('clock_routing_passed')))
    (folder/'result.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps({k:v for k,v in record.items() if k!='source_sha256'},indent=2))
    raise SystemExit(0 if record['passed'] else 1)
if __name__=='__main__':main()
