"""Explicit experimental full-system SRAM download; never writes Flash.

Keeps the production/static programmer's timing gates unchanged. Requires a
frozen, simulated system, pin/CDC structure audits and an explicit experiment
flag. Failed timing is recorded and never converted to a passing release.
"""
import argparse,hashlib,json,re,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build',type=Path)
    parser.add_argument('--experimental',action='store_true',required=True)
    args=parser.parse_args();build=args.build.resolve()
    meta=json.loads((build/'result.json').read_text());test=json.loads((build/'test_result.json').read_text())
    assert meta['top']=='system_task4_top' and meta['system'] and meta['frozen_sources_unchanged']
    assert meta['actual_exit_code']==0 and not meta.get('timed_out')
    assert test['passed'] and test['source_sha256']==meta['source_sha256']
    assert test['all_motor_pads_low_while_unarmed'] and test['uart_idle']
    assert meta['system_audits']['control50'] and meta['system_audits']['lk_state']
    assert json.loads((build/'timing_audit.json').read_text())['cdc_data_paths_passed']
    assert json.loads((build/'pin_audit.json').read_text())['passed']
    assert all(sha(build/'source'/p)==h for p,h in meta['source_sha256'].items())
    assert sha(build/'tb_system_sector_ethernet.sv')==test['generated_bench_sha256']
    bit=build/'impl/pnr/sensor.fs';assert sha(bit)==meta['bitstream_sha256']
    out=build/('board_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
    active=ROOT/'runs/commissioning/active_system_sram.json'
    if active.exists():(out/'previous_active_system_sram.json').write_bytes(active.read_bytes())
    record=dict(top=meta['top'],build=str(build),firmware_mode='experimental_full_system_360',
        experimental=True,timing_signoff=meta['passed'],timing_violations=meta['violations'],
        min_pulse_width_slack_ns=meta['min_pulse_width_slack_ns'],clock_route_warnings=meta['clock_route_warnings'],
        operation='SRAM Program',flash_written=False,bitstream_sha256=sha(bit),
        motor_outputs_constant_low=False,command_receiver_enabled=True,camera_enabled=True,
        optical_flow_enabled=True,lidar_bins=360,pin_profile=meta['pin_profile'],
        serial_bytes_written=0,telemetry_transport='udp',telemetry_port=6205,
        rollback_tag='lidar360-mpu6500-20261102',started_unix=time.time())
    cli=Path('C:/Gowin/Gowin_V1.9.12.04_x64/Programmer/bin/programmer_cli.exe')
    def run(arguments):
        done=subprocess.run([str(cli)]+arguments,capture_output=True,timeout=90)
        text=(done.stdout+done.stderr).decode('utf-8',errors='replace')
        with (out/'programmer.log').open('a',encoding='utf-8') as f:f.write(repr(arguments)+'\n'+text+'\n')
        if done.returncode or re.search(r'Error:',text,re.I):raise RuntimeError(text)
        return text
    try:
        locations=re.findall(r'USB location:\s*(\d+)',run(['--scan-cables','F']))
        if len(locations)!=1:raise RuntimeError('Expected one FPGA cable')
        cable=['--cable-index','1','--location',locations[0]]
        if '0011081B' not in run(cable+['--scan']).upper():raise RuntimeError('FPGA ID mismatch')
        text=run(cable+['--device','GW5AT-138B','--operation_index','2','--fsFile',str(bit)])
        if 'Finished.' not in text:raise RuntimeError('Programming completion not acknowledged')
        record['download_confirmed']=True
        active.write_text(json.dumps(dict(record,last_program=str(out/'result.json')),indent=2))
    finally:
        record['finished_unix']=time.time();(out/'result.json').write_text(json.dumps(record,indent=2))
    print(json.dumps(record,indent=2));print(out)

if __name__=='__main__':main()
