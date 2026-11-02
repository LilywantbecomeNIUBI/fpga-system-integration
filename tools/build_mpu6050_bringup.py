"""Build a motor-disabled MPU6050-only check using the current RevB pin map."""
import argparse,csv,hashlib,json,os,re,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
from gowin_runner import run_gowin

ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--prepare-only',action='store_true');a=ap.parse_args()
    folder=ROOT/'build/mpu6050_bringup_20261102';folder.mkdir(parents=True,exist_ok=True)
    names={'clk50m','reset_n','uart_tx','imu_i2c_scl','imu_i2c_sda','left_in1','left_in2','right_in1','right_in2','legacy_m1_disable','legacy_m2_disable'}
    rows=[r for r in csv.DictReader((ROOT/'hardware/pins.csv').open(encoding='utf-8-sig'))
        if r['signal'] in names or r['group']=='legacy_disabled' or r['signal'].startswith('led[')]
    assert len(rows)==len({r['ball'] for r in rows})==33
    cst=[]
    for r in rows:
        pull='DOWN' if r['group'] in ('chassis','legacy_disabled') or r['signal'].startswith('legacy_') else 'NONE'
        cst += ['IO_LOC "'+r['signal']+'" '+r['ball']+';',
            'IO_PORT "'+r['signal']+'" IO_TYPE=LVCMOS33 PULL_MODE='+pull+' BANK_VCCIO=3.3;']
    (folder/'system.cst').write_text('\n'.join(cst)+'\n',encoding='utf-8')
    sdc='create_clock -name sys50 -period 20 [get_ports {clk50m}]\nset_false_path -from [get_ports {reset_n}]\n'
    for pin in ('scl_sync_0_s0','sda_sync_0_s0'):
        sdc+='set_false_path -to [get_pins {imu/reader/bus/'+pin+'/D}]\n'
    (folder/'system.sdc').write_text(sdc,encoding='utf-8')
    sources=[ROOT/'fpga/imu'/n for n in ('mpu6050_bringup_top.sv','imu_1khz_core.v','mpu6050_reader.v','imu_fast_i2c_master.v','imu_fast_uart_stream.v')]
    sources += [folder/'system.cst',folder/'system.sdc']
    project=ET.Element('Project');ET.SubElement(project,'Template').text='FPGA';ET.SubElement(project,'Version').text='5'
    ET.SubElement(project,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012'
    files=ET.SubElement(project,'FileList')
    for p in sources:
        ET.SubElement(files,'File',path=Path(os.path.relpath(p,folder)).as_posix(),type='file.'+('verilog' if p.suffix in ('.v','.sv') else p.suffix[1:]),enable='1')
    ET.ElementTree(project).write(folder/'system.gprj',encoding='utf-8',xml_declaration=True)
    tcl='set here [file dirname [file normalize [info script]]]\ncd $here\nopen_project system.gprj\n'
    for k,v in [('top_module','mpu6050_bringup_top'),('verilog_std','sysv2017'),('output_base_name','system'),('max_fanout','32'),('replicate_resources','0'),('timing_driven','1'),('use_sspi_as_gpio','1'),('use_cpu_as_gpio','1'),('place_option','2'),('route_option','1')]:
        tcl+='set_option -'+k+' '+v+'\n'
    (folder/'build.tcl').write_text(tcl+'run all\nexit\n',encoding='utf-8')
    if a.prepare_only:print(folder/'system.gprj');return
    active=subprocess.run(['powershell','-NoProfile','-Command','Get-Process gw_sh,gw_syn,gw_pnr -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id'],capture_output=True,text=True)
    if active.stdout.strip():raise SystemExit('Another Gowin build is running; left untouched')
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in sources+[folder/'system.gprj',folder/'build.tcl',ROOT/'hardware/pins.csv']}
    exe=Path(os.environ.get('GOWIN_HOME','C:/Gowin/Gowin_V1.9.12.04_x64'))/'IDE/bin/gw_sh.exe'
    result=run_gowin(str(exe),folder/'build.tcl',folder/'build.log',pnr=True)
    trace=(folder/'impl/pnr/system.tr').read_text(errors='replace')
    violations={kind.lower():int(re.search(r'<Numbers of '+kind+r' Violated Endpoints>:(\d+)',trace)[1]) for kind in ('Setup','Hold')}
    pulse=[float(x) for x in re.findall(r'MPW Summary:\s+Slack:\s+([-\d.]+)',trace)]
    warnings=[line for line in (folder/'build.log').read_text(errors='replace').splitlines() if 'PR1014' in line]
    netlist=(folder/'impl/gwsynthesis/system.vg').read_text(errors='replace')
    motor_pads={}
    for body in re.findall(r'\bOBUF\s+\S+\s*\((.*?)\);',netlist,re.S):
        output=re.search(r'\.O\(([^)]+)\)',body);source=re.search(r'\.I\(([^)]+)\)',body)
        if output and source and (output[1] in {'left_in1','left_in2','right_in1','right_in2'} or output[1].startswith('legacy_')):
            motor_pads[output[1]]=source[1]
    motor_audit=len(motor_pads)==20 and set(motor_pads.values())=={'GND'}
    unchanged=all(sha(ROOT/p)==h for p,h in hashes.items())
    record=dict(**result,violations=violations,min_pulse_width_slack_ns=min(pulse) if pulse else None,
        pulse_width_passed=bool(pulse) and min(pulse)>=0,clock_route_warnings=warnings,sources_unchanged=unchanged,
        bitstream_sha256=sha(folder/'impl/pnr/system.fs'),source_sha256=hashes,top='mpu6050_bringup_top',
        pin_profile='current_RevB',imu_address='0x68',imu_config_id=4,motor_outputs_constant_low=True,
        motor_commands_supported=False,motor_pad_netlist_audit_passed=motor_audit,motor_pad_sources=motor_pads,
        netlist_sha256=sha(folder/'impl/gwsynthesis/system.vg'),timing_sha256=sha(folder/'impl/pnr/system.tr'),programmed=False,hardware_tested=False)
    record['passed']=violations==dict(setup=0,hold=0) and record['pulse_width_passed'] and not warnings and unchanged and motor_audit
    (folder/'result.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in record.items() if k!='source_sha256'},indent=2))
    raise SystemExit(0 if record['passed'] else 1)

if __name__=='__main__':main()
