"""Explicit experimental raw Mecanum SRAM load; no Flash or motor commands."""
from pathlib import Path
import argparse,hashlib,json,re,subprocess,time
ROOT=Path(__file__).resolve().parents[2]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('build',type=Path)
    ap.add_argument('--experimental',action='store_true',required=True)
    ap.add_argument('--expected-sha256',required=True)
    args=ap.parse_args();b=args.build.resolve()
    m=json.loads((b/'result.json').read_text());t=json.loads((b/'system_test_result.json').read_text())
    a=json.loads((b/'raw_timing_audit.json').read_text())
    assert m['top']=='system_mecanum_top' and m['pin_profile']=='mecanum_p7'
    assert m['lidar_mode']=='raw_points' and m['violations']==dict(setup=0,hold=0)
    assert m['actual_exit_code']==0 and m['frozen_sources_unchanged']
    assert t['passed'] and t['raw_points_checked']==12000 and t['source_sha256']==m['source_sha256']
    assert t['motor_outputs_low'] and t['uart_idle'] and a['requested_setup_hold_and_margin_passed']
    assert all(m['system_audits'].values()) and json.loads((b/'pin_audit.json').read_text())['passed']
    assert all(sha(b/'source'/p)==h for p,h in m['source_sha256'].items())
    bit=b/'impl/pnr/sensor.fs';assert sha(bit)==m['bitstream_sha256']
    assert sha(bit)==args.expected_sha256
    # Ball-only pin audits missed incorrect internal pulls during integration.
    # Validate complete teammate IO_PORT statements before touching hardware.
    cst=(b/'sensor.cst').read_text();team=(ROOT/'vendor/mecanum_team/project/mecanum.cst').read_text()
    for statement,name in re.findall(r'(IO_PORT "([^"]+)"[^;]+;)',team):
        new={'imu_scl':'imu_i2c_scl','imu_sda':'imu_i2c_sda'}.get(name,name)
        assert statement.replace('"'+name+'"','"'+new+'"') in cst,(name,'electrical pin mismatch')
    out=b/('board_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
    active=ROOT/'runs/commissioning/active_system_sram.json';active.parent.mkdir(parents=True,exist_ok=True)
    record=dict(build=str(b),top=m['top'],firmware_mode='experimental_mecanum_raw_full_system',
        bitstream_sha256=sha(bit),pin_profile=m['pin_profile'],operation='SRAM Program',flash_written=False,
        timing_signoff=m['passed'],timing_violations=m['violations'],min_pulse_width_slack_ns=m['min_pulse_width_slack_ns'],
        clock_route_warnings=m['clock_route_warnings'],camera_enabled=True,optical_flow_enabled=True,
        command_receiver_enabled=True,telemetry_transport='udp',telemetry_port=6205,
        lidar_mode='raw_points',motor_commands_sent=0,serial_bytes_written=0,started_unix=time.time())
    cli=Path('C:/Gowin/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe')
    def run(arguments):
        p=subprocess.run([str(cli),*arguments],capture_output=True,timeout=90)
        text=(p.stdout+p.stderr).decode('utf-8',errors='replace')
        with (out/'programmer.log').open('a',encoding='utf-8') as f:f.write(repr(arguments)+'\n'+text+'\n')
        if p.returncode or re.search(r'Error:',text,re.I):raise RuntimeError(text)
        return text
    try:
        locations=re.findall(r'USB location:\s*(\d+)',run(['--scan-cables','F']))
        assert len(locations)==1,'Expected exactly one connected FPGA'
        cable=['--cable-index','1','--location',locations[0]]
        assert '0011081B' in run(cable+['--scan']).upper()
        text=run(cable+['--device','GW5AT-138B','--operation_index','2','--fsFile',str(bit)])
        assert 'Finished.' in text
        record['download_confirmed']=True
        active.write_text(json.dumps(dict(record,last_program=str(out/'result.json')),indent=2)+'\n')
    finally:
        record['finished_unix']=time.time();(out/'result.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2));print(out)


if __name__=='__main__':main()
