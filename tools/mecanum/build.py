"""Prepare a frozen Mecanum full-system build; never programs hardware.

Default only prepares files. Run build.tcl using an available Gowin instance
after coordinating with other timing builds. Signoff is intentionally pending.
"""
from pathlib import Path
import csv, hashlib, json, re, shutil, sys, time
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/task4'))
from build import sources, base_constraints


def prepare():
    out = ROOT/'build/mecanum'/time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    out.mkdir(parents=True, exist_ok=False)
    files = sorted(set(sources()+list((ROOT/'fpga/mecanum').glob('*.sv'))))
    hashes = {}
    for p in files:
        dest = out/'source'/p.relative_to(ROOT)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, dest)
        hashes[p.relative_to(ROOT).as_posix()] = hashlib.sha256(dest.read_bytes()).hexdigest()
    if any(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h for p,h in hashes.items()):
        raise RuntimeError('Source changed while preparing snapshot')
    profile = json.loads((ROOT/'hardware/mecanum.json').read_text())
    pins = profile['pins']
    rows = [dict(group='chassis', **p) for p in pins]
    rows += [r for r in csv.DictReader((ROOT/'hardware/pins.csv').open(encoding='utf-8-sig'))
             if r['group'] in ('camera', 'ethernet')]
    if len(rows)!=len({r['ball'] for r in rows}) or len(rows)!=len({r['signal'] for r in rows}):
        raise RuntimeError('Mecanum/camera/Ethernet physical pin collision')
    cst, sdc = base_constraints(rows)
    # Four vector encoder inputs use external idle pullups, as in the team CST.
    for pin in pins:
        if pin['signal'].startswith(('enc_a[','enc_b[')):
            name=re.escape(pin['signal'])
            cst=re.sub(r'(IO_PORT "'+name+r'"[^\n]*PULL_MODE=)DOWN',r'\g<1>UP',cst)
    # The base has differential-chassis and old IMU synchronizer names. Remove
    # only those absent-instance constraints; add real Mecanum first stages.
    sdc = '\n'.join(line for line in sdc.splitlines()
                    if not any(old in line for old in ('chassis/rx/', 'chassis/core/enc_l/',
                       'chassis/core/enc_r/', 'imu/reader/bus/')))+'\n'
    for pin in ('chassis/receiver/rx_meta_s0',
                'chassis/imu.reader/bus/scl_sync_0_s0',
                'chassis/imu.reader/bus/sda_sync_0_s0', 'lidar_uart/rx_meta_s0'):
        sdc += 'set_false_path -to [get_pins {%s/D}]\n'%pin
    for wheel in range(4):
        for bit in range(2):
            sdc += 'set_false_path -to [get_pins {chassis/core/wheels[%d].encoder/ab_meta_%d_s0/D}]\n'%(wheel,bit)
    for clock in ('sys50','control50','cam_pclk','eth125','eth_rx125'):
        sdc += 'report_timing -setup -from_clock [get_clocks {%s}] -to_clock [get_clocks {%s}] -max_paths 100\n'%(clock,clock)
    (out/'mecanum.cst').write_text(cst)
    (out/'mecanum.sdc').write_text(sdc)
    (out/'pin_audit.json').write_text(json.dumps(dict(passed=True,pins=rows),indent=2))
    project = ET.Element('Project')
    ET.SubElement(project,'Template').text='FPGA'
    ET.SubElement(project,'Version').text='5'
    ET.SubElement(project,'Device',name='GW5AT-138B',pn='GW5AT-LV138PG484AC1/I0').text='gw5at138b-012'
    fl=ET.SubElement(project,'FileList')
    for p in [out/'source'/p.relative_to(ROOT) for p in files]+[out/'mecanum.cst',out/'mecanum.sdc']:
        ET.SubElement(fl,'File',path=p.relative_to(out).as_posix(),type='file.'+('verilog' if p.suffix in ('.v','.sv') else p.suffix[1:]),enable='1')
    ET.ElementTree(project).write(out/'mecanum.gprj',encoding='utf-8',xml_declaration=True)
    tcl='set here [file dirname [file normalize [info script]]]\ncd $here\nopen_project mecanum.gprj\n'
    for k,v in [('top_module','system_mecanum_top'),('verilog_std','sysv2017'),
                ('output_base_name','mecanum'),('max_fanout','32'),('replicate_resources','1'),
                ('timing_driven','1'),('retiming_resource','bsram'),('use_sspi_as_gpio','1'),
                ('use_cpu_as_gpio','1'),('place_option','2'),('route_option','0')]:
        tcl+='set_option -%s %s\n'%(k,v)
    (out/'build.tcl').write_text(tcl+'run all\nexit\n')
    input_hashes={}
    for name in ('hardware/mecanum.json','hardware/pins.csv','vendor/reference/robot.cst',
                 'vendor/reference/robot.sdc','fpga/vision/constraints/perception_cdc.sdc',
                 'tools/mecanum/build.py','tools/task4/build.py'):
        dest=out/'inputs'/name
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,dest)
        input_hashes[name]=hashlib.sha256(dest.read_bytes()).hexdigest()
    constraints={name:hashlib.sha256((out/name).read_bytes()).hexdigest()
                 for name in ('mecanum.cst','mecanum.sdc','build.tcl','mecanum.gprj')}
    metadata=dict(top='system_mecanum_top',source_sha256=hashes,pin_profile='mecanum_p7',
        input_sha256=input_hashes,constraint_sha256=constraints,
        base_commit='acaff0457777a5a4b50c86a7274178452ff09d20',lidar_bins=360,
        camera_enabled=True,optical_flow_enabled=True,differential_navigation_enabled=False,
        mecanum_pose_enabled=True,imu_sample_hz=100,telemetry_transport='udp',telemetry_port=6205,
        commands='uart_v3',programmed=False,hardware_tested=False,timing_signoff=False)
    (out/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return out


if __name__=='__main__':
    print(prepare())
