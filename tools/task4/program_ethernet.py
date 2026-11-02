"""Program only a verified static Ethernet sensor image into SRAM; never Flash.

Requires passing timing, matching frozen RTL simulation and twelve constant-low
motor pads. This tool sends no motor commands and opens no serial port.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import subprocess
import time
ROOT=Path(__file__).resolve().parents[2]


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('build',type=Path);args=parser.parse_args()
    build=args.build.resolve();meta=json.loads((build/'result.json').read_text());test=json.loads((build/'test_result.json').read_text())
    assert meta['passed'] and meta['top']=='m2_sensor_eth_diag_top' and meta['motor_outputs_constant_low']
    assert meta['violations']==dict(setup=0,hold=0) and meta['min_pulse_width_slack_ns']>=0 and not meta['clock_route_warnings']
    assert meta['command_receiver_enabled']==meta['camera_enabled']==meta['navigation_enabled']==False
    assert test['passed'] and test['source_sha256']==meta['source_sha256'] and test['all_motor_pads_low']
    raw=meta.get('lidar_mode')=='raw_points'
    bench='tests/diagnostics/tb_m2_sensor_raw_ethernet.sv' if raw else 'tests/diagnostics/tb_m2_sensor_ethernet.sv'
    assert test.get('testbench_path',bench)==bench and test['testbench_sha256']==sha(ROOT/bench)
    assert all(sha(build/'source'/p)==h for p,h in meta['source_sha256'].items())
    assert json.loads((build/'pin_audit.json').read_text())['passed']
    bit=build/'impl/pnr/sensor.fs';assert sha(bit)==meta['bitstream_sha256']
    out=build/('board_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(exist_ok=False)
    active=ROOT/'runs/commissioning/active_system_sram.json'
    if active.exists():(out/'previous_active_system_sram.json').write_bytes(active.read_bytes())
    record=dict(top=meta['top'],firmware_mode='static_sensors_raw_ethernet' if raw else 'static_sensors_360_ethernet',build=str(build),
                operation='SRAM Program',flash_written=False,serial_bytes_written=0,
                bitstream_sha256=sha(bit),motor_outputs_constant_low=True,command_receiver_enabled=False,
                telemetry_transport='udp',telemetry_port=6205,started_unix=time.time())
    cli=Path('C:/Gowin/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe')
    def run(arguments):
        result=subprocess.run([str(cli)]+arguments,capture_output=True,timeout=90)
        text=(result.stdout+result.stderr).decode('utf-8',errors='replace')
        with (out/'programmer.log').open('a',encoding='utf-8') as log:log.write(repr(arguments)+'\n'+text+'\n')
        if result.returncode or re.search(r'Error:',text,re.I):raise RuntimeError(text)
        return text
    try:
        locations=re.findall(r'USB location:\s*(\d+)',run(['--scan-cables','F']))
        if len(locations)!=1:raise RuntimeError('Expected one FPGA cable')
        cable=['--cable-index','1','--location',locations[0]]
        if '0011081B' not in run(cable+['--scan']).upper():raise RuntimeError('FPGA ID mismatch')
        output=run(cable+['--device','GW5AT-138B','--operation_index','2','--fsFile',str(bit)])
        if 'Finished.' not in output:raise RuntimeError('No programming completion acknowledgment')
        record['download_confirmed']=True
        active.write_text(json.dumps(dict(record,last_program=str(out/'result.json')),indent=2))
        print('Verified static Ethernet sensor image programmed into SRAM; no Flash write.',flush=True)
    finally:
        record['finished_unix']=time.time();(out/'result.json').write_text(json.dumps(record,indent=2))
    print(out,flush=True)


if __name__=='__main__':main()
