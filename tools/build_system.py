"""Build the integrated system candidate; never programs a board."""
import argparse
import csv
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
        pull='UP' if r['signal']=='uart_rx' else 'DOWN' if r['group'] in ('chassis','legacy_disabled') else 'NONE'
        port=original_ports.get(r['signal']) if r['group'] not in ('chassis','legacy_disabled') else None
        cst += [f'IO_LOC "{r["signal"]}" {r["ball"]};',port or f'IO_PORT "{r["signal"]}" IO_TYPE=LVCMOS33 PULL_MODE={pull} BANK_VCCIO=3.3;']
    cst+=re.findall(r'^CLOCK_LOC[^\n]+',(ROOT/'vendor/reference/robot.cst').read_text(encoding='utf-8'),re.M)
    (folder/'system.cst').write_text('\n'.join(cst)+'\n',encoding='utf-8')
    # Preserve the sensing candidate's clock/CDC constraints; no extra module-wide exceptions.
    sdc=(ROOT/'vendor/reference/robot.sdc').read_text(encoding='utf-8').replace('imu/core/reader/','imu/reader/')
    for pin in ('rx/rx_meta_s0/D','core/enc_l/ab_meta_0_s0/D','core/enc_l/ab_meta_1_s0/D','core/enc_r/ab_meta_0_s0/D','core/enc_r/ab_meta_1_s0/D'):
        sdc+=f'set_false_path -to [get_pins {{{pin}}}]\n'
    (folder/'system.sdc').write_text(sdc,encoding='utf-8')
    sources=sorted((ROOT/'fpga/motor').glob('*.sv'))+sorted((ROOT/'fpga/system').glob('*.sv'))+sorted((ROOT/'fpga/system').glob('*.v'))
    sources+=sorted(p for p in (ROOT/'fpga/vision/src').rglob('*.v') if p.name!='camera_perception_top.v')
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
    for k,v in [('top_module','system_top'),('verilog_std','v2001'),('output_base_name','system'),('max_fanout','16'),('replicate_resources','1'),('retiming_resource','bsram'),('use_sspi_as_gpio','1'),('use_cpu_as_gpio','1'),('place_option','3'),('route_option','2')]:
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
    record['sources_unchanged']=all(sha(ROOT/p)==h for p,h in hashes.items())
    if timing.exists() and result.returncode==0:
        content=timing.read_text(encoding='utf-8',errors='replace')
        record['violations']={}
        for kind in ('Setup','Hold'):
            match=re.search(r'Numbers of '+kind+r' Violated Endpoints</td>\s*<td[^>]*>(\d+)',content)
            record['violations'][kind.lower()]=int(match[1]) if match else None
        record['timing_sha256']=sha(timing)
    if result.returncode==0 and bit.exists(): record['bitstream_sha256']=sha(bit)
    record['passed']=result.returncode==0 and record.get('violations')=={'setup':0,'hold':0} and record['sources_unchanged'] and 'bitstream_sha256' in record
    for p in (attempt/'result.json',folder/'result.json'):p.write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in record.items() if k!='source_sha256'},indent=2))
    raise SystemExit(0 if record['passed'] else 1)

if __name__=='__main__': main()
