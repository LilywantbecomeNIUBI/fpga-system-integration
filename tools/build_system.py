"""Build the integrated system candidate; never programs a board."""
import argparse
import csv
import html
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import shutil
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def prepare():
    folder=ROOT/'build/system'; folder.mkdir(parents=True,exist_ok=True)
    rows=list(csv.DictReader((ROOT/'hardware/pins.csv').open(encoding='utf-8-sig')))
    names={'clk50m','reset_n','uart_rx','uart_tx','legacy_m1_disable','legacy_m2_disable','imu_i2c_scl','imu_i2c_sda'}
    selected=[r for r in rows if r['group'] in ('chassis','legacy_disabled','camera','ethernet') or r['signal'] in names or r['signal'].startswith('led[')]
    cst=['// Integrated chassis/sensing candidate. Encoder inputs require verified 3.3V conditioning.']
    original_ports=dict(re.findall(r'(IO_PORT "([^"]+)"[^;]+;)',(ROOT/'vendor/reference/robot.cst').read_text(encoding='utf-8')))
    original_ports={signal:statement for statement,signal in original_ports.items()}
    for r in selected:
        pull='UP' if r['signal']=='uart_rx' or r['signal'] in ('left_enc_a','left_enc_b','right_enc_a','right_enc_b') else 'DOWN' if r['group'] in ('chassis','legacy_disabled') else 'NONE'
        port=original_ports.get(r['signal']) if r['group'] not in ('chassis','legacy_disabled') else None
        cst += [f'IO_LOC "{r["signal"]}" {r["ball"]};',port or f'IO_PORT "{r["signal"]}" IO_TYPE=LVCMOS33 PULL_MODE={pull} BANK_VCCIO=3.3;']
    cst+=re.findall(r'^CLOCK_LOC[^\n]+',(ROOT/'vendor/reference/robot.cst').read_text(encoding='utf-8'),re.M)
    (folder/'system.cst').write_text('\n'.join(cst)+'\n',encoding='utf-8')
    # Bind generated clocks to their PLL output pins. Preserve actual PLL ratios
    # and the existing CDC exceptions; no clock is relaxed to hide a data path.
    sdc=(ROOT/'vendor/reference/robot.sdc').read_text(encoding='utf-8').replace('imu/core/reader/','imu/reader/')
    sdc=sdc.replace('[get_nets {camera/clk50}]','[get_pins {camera/pll/PLL_inst/CLKOUT0}]')
    sdc=sdc.replace('[get_nets {camera/clk125}]','[get_pins {camera/pll/PLL_inst/CLKOUT1}]')
    # Clock groups outrank max-delay in Gowin. Leave the new pointer CDC
    # timed with explicit bounds, instead of silently masking its skew budget.
    sdc=sdc.replace('-group [get_clocks {cam_pclk}] -group [get_clocks {sys50 control50 eth125}]',
                    '-group [get_clocks {cam_pclk}] -group [get_clocks {sys50 eth125}]')
    sdc+='\n'+(ROOT/'fpga/vision/constraints/perception_cdc.sdc').read_text(encoding='utf-8')
    for pin in ('chassis/rx/rx_meta_s0/D','chassis/core/enc_l/ab_meta_0_s0/D','chassis/core/enc_l/ab_meta_1_s0/D','chassis/core/enc_r/ab_meta_0_s0/D','chassis/core/enc_r/ab_meta_1_s0/D'):
        sdc+=f'set_false_path -to [get_pins {{{pin}}}]\n'
    (folder/'system.sdc').write_text(sdc,encoding='utf-8')
    sources=sorted((ROOT/'fpga/control').glob('*.sv'))+sorted((ROOT/'fpga/motor').glob('*.sv'))+sorted((ROOT/'fpga/system').glob('*.sv'))+sorted((ROOT/'fpga/system').glob('*.v'))
    sources+=sorted(p for p in (ROOT/'fpga/vision/src').rglob('*.v') if p.name!='camera_perception_top.v')
    sources+=sorted((ROOT/'fpga/vision/src').rglob('*.sv'))
    sources+=[ROOT/'fpga/imu'/name for name in ('imu_1khz_core.v','icm42688_fast_reader.v','imu_fast_i2c_master.v')]
    sources+=[folder/'system.cst',folder/'system.sdc']
    project=ET.Element('Project')
    ET.SubElement(project,'Template').text='FPGA'; ET.SubElement(project,'Version').text='5'
    ET.SubElement(project,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012'
    files=ET.SubElement(project,'FileList')
    for p in sources:
        ET.SubElement(files,'File',path=Path(os.path.relpath(p,folder)).as_posix(),type={'.v':'file.verilog','.sv':'file.verilog','.cst':'file.cst','.sdc':'file.sdc'}[p.suffix],enable='1')
    ET.ElementTree(project).write(folder/'system.gprj',encoding='utf-8',xml_declaration=True)
    tcl='set here [file dirname [file normalize [info script]]]\ncd $here\nopen_project system.gprj\n'
    profile=ROOT/'config/pnr.json'
    place=json.loads(profile.read_text())['place_option'] if profile.exists() else 2
    for k,v in [('top_module','system_top'),('verilog_std','sysv2017'),('output_base_name','system'),('max_fanout','32'),('replicate_resources',str(json.loads(profile.read_text()).get('replicate_resources',1)) if profile.exists() else '1'),('timing_driven','1'),('retiming_resource','bsram'),('use_sspi_as_gpio','1'),('use_cpu_as_gpio','1'),('place_option',str(place)),('route_option','1')]:
        tcl+=f'set_option -{k} {v}\n'
    (folder/'build.tcl').write_text(tcl+'run all\n',encoding='utf-8')
    return folder,sources

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    folder,sources=prepare()
    if args.prepare_only: print(folder/'system.gprj');return
    if os.name=='nt':
        active=subprocess.run(['powershell','-NoProfile','-Command','Get-Process gw_sh,gw_syn,gw_pnr -ErrorAction SilentlyContinue | Where-Object { -not $_.HasExited } | Select-Object -ExpandProperty Id'],capture_output=True,text=True)
        if active.stdout.strip(): raise SystemExit('Another Gowin build is running; left untouched.')
    exe=Path(os.environ.get('GOWIN_HOME','C:/Gowin/Gowin_V1.9.12.04_x64'))/'IDE/bin/gw_sh.exe'
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    attempt=folder/('attempt_'+stamp);attempt.mkdir()
    hashes={p.relative_to(ROOT).as_posix():sha(p) for p in sources}
    for name in ('system.gprj','build.tcl'):
        hashes[(folder/name).relative_to(ROOT).as_posix()]=sha(folder/name)
        shutil.copyfile(folder/name,attempt/name)
    options=dict(re.findall(r'set_option -(\w+) ([^\n]+)',(folder/'build.tcl').read_text(encoding='utf-8')))
    with (attempt/'build.log').open('w',encoding='utf-8') as log:
        result=subprocess.run([str(exe),str(folder/'build.tcl')],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    timing=folder/'impl/pnr/system_tr_content.html'; bit=folder/'impl/pnr/system.fs'
    record=dict(started_utc=stamp,exit_code=result.returncode,source_sha256=hashes,options=options,tool=str(exe),tool_sha256=sha(exe),programmed=False,hardware_tested=False)
    audit=subprocess.run([sys.executable,str(ROOT/'tools/audit_lk_state.py')],cwd=ROOT,capture_output=True,text=True) if result.returncode==0 else None
    record['state_register_audit_passed']=bool(audit is not None and audit.returncode==0)
    if audit is not None:(attempt/'lk_state_audit.txt').write_text(audit.stdout+audit.stderr,encoding='utf-8')
    record.update(pwm_limit_per_mille=350, left_encoder_profile='P7-31/F16, P7-32/E17', avoidance_mode='observe_only')
    record['sources_unchanged']=all(sha(ROOT/p)==h for p,h in hashes.items())
    if timing.exists() and result.returncode==0:
        content=timing.read_text(encoding='utf-8',errors='replace')
        record['violations']={}
        for kind in ('Setup','Hold'):
            match=re.search(r'Numbers of '+kind+r' Violated Endpoints</td>\s*<td[^>]*>(\d+)',content)
            record['violations'][kind.lower()]=int(match[1]) if match else None
        record['timing_sha256']=sha(timing)
        table=re.search(r'name="MIN_PULSE_WIDTH_TABLE".*?<table[^>]*>(.*?)</table>',content,re.S)
        record['reported_min_pulse_width']=[]
        if table:
            for row in re.findall(r'<tr>(.*?)</tr>',table[1],re.S):
                cells=[html.unescape(re.sub(r'<[^>]+>','',c)).strip() for c in re.findall(r'<td[^>]*>(.*?)</td>',row,re.S)]
                if len(cells)==7:
                    record['reported_min_pulse_width'].append(dict(slack_ns=float(cells[1]),actual_ns=float(cells[2]),required_ns=float(cells[3]),kind=cells[4],clock=cells[5],object=cells[6]))
        record['pulse_width_passed']=bool(record['reported_min_pulse_width']) and all(p['slack_ns']>=0 for p in record['reported_min_pulse_width'])
    pnr_log=folder/'impl/pnr/system.log'
    if result.returncode==0 and pnr_log.exists():
        record['clock_route_warnings']=[line for line in pnr_log.read_text(encoding='utf-8',errors='replace').splitlines() if 'PR1014' in line]
        record['clock_routing_passed']=not record['clock_route_warnings']
    if result.returncode==0 and bit.exists(): record['bitstream_sha256']=sha(bit)
    record['setup_hold_passed']=result.returncode==0 and record.get('violations')=={'setup':0,'hold':0}
    record['passed']=record['setup_hold_passed'] and record.get('pulse_width_passed',False) and record.get('clock_routing_passed',False) and record['sources_unchanged'] and record['state_register_audit_passed'] and 'bitstream_sha256' in record
    for src,name in ((timing,'timing.html'),(folder/'impl/pnr/system.tr','timing.txt'),(folder/'impl/pnr/system.rpt.txt','pnr_report.txt'),(pnr_log,'pnr.log'),(folder/'system.sdc','system.sdc'),(folder/'system.cst','system.cst')):
        if result.returncode==0 and src.exists():shutil.copyfile(src,attempt/name)
    for p in (attempt/'result.json',folder/'result.json'):p.write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in record.items() if k!='source_sha256'},indent=2))
    raise SystemExit(0 if record['passed'] else 1)

if __name__=='__main__': main()
