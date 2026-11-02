"""Prepare/build only. Never downloads a bitstream or opens a serial port."""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import stat
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
# Reused from existing ACG720 pin inventory. P7-11=5V,29=3V3,12/30=GND.
WHEELS = [
    ('FL', (21, 'D17'), (22, 'C17'), (31, 'F16'), (32, 'E17')),
    ('FR', (23, 'E16'), (24, 'D16'), (27, 'C18'), (36, 'D19')),
    ('RL', (25, 'B17'), (26, 'B18'), (34, 'C20'), (35, 'E19')),
    ('RR', (28, 'C19'), (33, 'D20'), (37, 'F18'), (38, 'E18')),
]


def copy_result(source,destination):
    # Gowin marks bitstreams read-only; repeated builds must replace our own outputs.
    dest=Path(destination)
    if dest.exists():
        if not dest.resolve().is_relative_to(ROOT.resolve()):
            raise ValueError('Refusing to change attributes outside this project')
        dest.chmod(stat.S_IREAD | stat.S_IWRITE)
    return shutil.copy2(source,destination)


def prepare():
    rows = []
    for n, (wheel, *pins) in enumerate(WHEELS):
        for signal, (p7, ball) in zip(('in1', 'in2', 'enc_a', 'enc_b'), pins):
            rows.append((f'{signal}[{n}]', ball, p7, wheel, 'IN' if signal.startswith('enc') else 'OUT'))
    for signal, ball in [('clk50m','Y18'),('reset_n','F15'),('uart_rx','J21'),('uart_tx','M15')]:
        rows.append((signal,ball,'','board','OUT' if signal=='uart_tx' else 'IN'))
    rows.extend([('imu_scl','F19',39,'MPU6050','INOUT'),('imu_sda','F20',40,'MPU6050','INOUT')])
    for n, ball in enumerate(['M22','N22','L21','K21','K22','J22','H22','M21']):
        rows.append((f'led[{n}]',ball,'','board','OUT'))
    for group, pins in [('legacy_m1_pwm_disabled',[(6,'A13'),(5,'A14'),(4,'A15'),(3,'A16'),(2,'A18'),(1,'A19')]),
                        ('legacy_m2_pwm_disabled',[(6,'E14'),(5,'C13'),(4,'B13'),(3,'D14'),(2,'D15'),(1,'C14')])]:
        for bit, ball in pins:
            rows.append((f'{group}[{bit-1}]',ball,'','fixed-low','OUT'))
    for signal,ball,p7 in [('legacy_m1_disable','F13',7),('legacy_m2_disable','C15',18),('legacy_cal_disabled[0]','F14',8),('legacy_cal_disabled[1]','B15',19)]:
        rows.append((signal,ball,p7,'fixed-low','OUT'))
    assert len(set(r[1] for r in rows)) == len(rows)
    folder=ROOT/'project';folder.mkdir(exist_ok=True)
    with (ROOT/'hardware/pins.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.writer(f);writer.writerow(['signal','ball','p7','wheel','direction']);writer.writerows(rows)
    cst=[]
    for signal,ball,p7,wheel,direction in rows:
        pull='UP' if signal.startswith('enc_') or signal=='uart_rx' else 'DOWN' if wheel in ('FL','FR','RL','RR','fixed-low') else 'NONE'
        cst.extend([f'IO_LOC "{signal}" {ball};',f'IO_PORT "{signal}" IO_TYPE=LVCMOS33 PULL_MODE={pull} BANK_VCCIO=3.3;'])
    (folder/'mecanum.cst').write_text('\n'.join(cst)+'\n',encoding='utf-8')
    # No broad internal timing exceptions. Async inputs terminate in 2FF synchronizers.
    (folder/'mecanum.sdc').write_text('create_clock -name clk50m -period 20.000 [get_ports {clk50m}]\n',encoding='utf-8')
    project=ET.Element('Project');ET.SubElement(project,'Template').text='FPGA';ET.SubElement(project,'Version').text='5'
    ET.SubElement(project,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012'
    files=ET.SubElement(project,'FileList')
    for p in sorted((ROOT/'rtl').glob('*.sv'))+[folder/'mecanum.cst',folder/'mecanum.sdc']:
        ET.SubElement(files,'File',path=Path(os.path.relpath(p,folder)).as_posix(),type={'.sv':'file.verilog','.cst':'file.cst','.sdc':'file.sdc'}[p.suffix],enable='1')
    ET.ElementTree(project).write(folder/'mecanum.gprj',encoding='utf-8',xml_declaration=True)
    tcl='set here [file dirname [file normalize [info script]]]\ncd $here\nopen_project mecanum.gprj\n'
    for key,value in [('top_module','mecanum_top'),('verilog_std','sysv2017'),('output_base_name','mecanum'),('max_fanout','32'),('replicate_resources','1'),('use_sspi_as_gpio','1'),('use_cpu_as_gpio','1')]:
        tcl+=f'set_option -{key} {value}\n'
    (folder/'build.tcl').write_text(tcl+'run all\nexit\n',encoding='utf-8')
    return folder


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',action='store_true');args=parser.parse_args()
    folder=prepare()
    print(folder/'mecanum.gprj')
    if args.run:
        tool=Path(os.environ.get('GOWIN_HOME','C:/Gowin/Gowin_V1.9.12.04_x64'))/'IDE/bin/gw_sh.exe'
        # Gowin on this machine cannot build under a Chinese pathname.
        staging=Path('C:/fpga_build');staging.mkdir(exist_ok=True)
        stage=Path(tempfile.mkdtemp(prefix='mecanum_task2_',dir=staging))
        shutil.copytree(ROOT/'rtl',stage/'rtl')
        shutil.copytree(folder,stage/'project',ignore=shutil.ignore_patterns('impl','*.log'))
        staged_project=stage/'project'
        with (folder/'build.log').open('w',encoding='utf-8') as log:
            result=subprocess.run([str(tool),str(staged_project/'build.tcl')],cwd=staged_project,stdout=log,stderr=subprocess.STDOUT)
        if (staged_project/'impl').exists():
            shutil.copytree(staged_project/'impl',folder/'impl',dirs_exist_ok=True,copy_function=copy_result)
        (folder/'build_stage.txt').write_text(str(stage),encoding='utf-8')
        print('Build exit:',result.returncode,'Log:',folder/'build.log')
        if result.returncode==0:
            from verify_build import verify
            if not verify()['passed']:raise SystemExit('Timing/source verification failed; see project/verification.json')
        raise SystemExit(result.returncode)
