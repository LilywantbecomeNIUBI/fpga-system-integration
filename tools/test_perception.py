"""Pinned optical-flow RTL oracle regression inside this repository."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--events-only',action='store_true',help='Reuse an existing successful optical-flow oracle result')
    args=parser.parse_args()
    original = (ROOT/'vendor/reference/test_optical_flow_original.py').read_text(encoding='utf-8')
    source = original.replace('ROOT = Path(__file__).resolve().parents[1]', 'ROOT = Path('+repr(str(ROOT))+')')
    source = source.replace("str(ROOT/'reference/optical_flow')", "str(ROOT/'vendor/vision')").replace('import model', 'import lk_model as model')
    source = source.replace("ROOT/'tests/optical_flow'", "ROOT/'build/tests/optical_flow'")
    source = source.replace("'reference/optical_flow/upstream/rtl/frame_memory.sv'", "'fpga/vision/src/optical_flow/frame_memory.sv'")
    source = source.replace("'src/", "'fpga/vision/src/").replace("f'sim/optical_flow/", "f'build/tests/flow_benches/").replace("'sim/optical_flow/", "'build/tests/flow_benches/")
    folder = ROOT/'build/tests/flow_benches'
    folder.mkdir(parents=True, exist_ok=True)
    for name in ('patch', 'pipeline', 'avoidance'):
        bench = (ROOT/('tests/vision/tb_flow_'+name+'.v')).read_text(encoding='utf-8')
        (folder/('tb_flow_'+name+'.v')).write_text(bench.replace('tests/optical_flow', 'build/tests/optical_flow'), encoding='utf-8')
    script = ROOT/'build/tests/flow_regression.py'
    script.write_text(source, encoding='utf-8')
    oracle_inputs=[ROOT/'fpga/vision/src/optical_flow'/name for name in
                   ('frame_memory.sv','flow_lk_patch.v','flow_frame_store.v','flow_grid.v',
                    'flow_grid_pyramid.v','flow_avoidance.v','flow_pipeline.v')]
    oracle_inputs += [ROOT/'fpga/vision/src/vision/vision_divider.v',ROOT/'vendor/vision/lk_model.py',
                      ROOT/'vendor/reference/test_optical_flow_original.py']
    oracle_inputs += [ROOT/'tests/vision'/('tb_flow_'+n+'.v') for n in ('patch','pipeline','avoidance')]
    oracle_hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in oracle_inputs}
    identity=ROOT/'build/tests/optical_flow/source_identity.json'
    if not args.events_only:
        subprocess.run([sys.executable, str(script), '--stage', '2'], cwd=ROOT, check=True)
        identity.write_text(json.dumps(oracle_hashes,indent=2)+'\n',encoding='utf-8')
    if not identity.exists() or json.loads(identity.read_text(encoding='utf-8'))!=oracle_hashes:
        raise SystemExit('Oracle source identity missing/stale; run without --events-only')
    oracle=json.loads((ROOT/'build/tests/optical_flow/result.json').read_text(encoding='utf-8'))
    assert oracle['passed'] and oracle['safety_checks'] and len(oracle['pyramid_cases'])==13
    subprocess.run(['iverilog', '-g2012', '-s', 'tb_perception_merge', '-o', 'build/tests/perception_merge.vvp',
                    'tests/vision/tb_perception_merge.v', 'fpga/system/event_packet_mux.v',
                    'fpga/vision/src/optical_flow/flow_event_bridge.v'], cwd=ROOT, check=True)
    subprocess.run(['vvp', 'build/tests/perception_merge.vvp'], cwd=ROOT, check=True)
    subprocess.run(['iverilog','-g2012','-s','tb_perception_deadlines','-o','build/tests/perception_deadlines.vvp',
                    'tests/vision/tb_perception_deadlines.v','fpga/vision/src/vision/vision_v23_bridge.v',
                    'fpga/vision/src/vision/vision_result_core.v'],cwd=ROOT,check=True)
    subprocess.run(['vvp','build/tests/perception_deadlines.vvp'],cwd=ROOT,check=True)
    sys.path.insert(0, str(ROOT/'vendor/vision'))
    from optical_flow_protocol import parse_packet
    from vision_v23_protocol import parse_event
    events = [bytes.fromhex(line) for line in (ROOT/'build/tests/perception_events.hex').read_text().splitlines()]
    statuses = [parse_packet(row) for row in events if row[:4] == b'OFST']
    targets = [parse_event(row) for row in events if row[:4] == b'FSTA']
    assert targets and statuses and any(row['stale'] and row['stop_request'] for row in statuses)
    inputs=list((ROOT/'fpga/vision/src/optical_flow').glob('*'))+[ROOT/'fpga/system/event_packet_mux.v',ROOT/'tests/vision/tb_perception_merge.v',ROOT/'vendor/vision/lk_model.py',ROOT/'tools/test_perception.py']
    inputs+=oracle_inputs
    inputs += [ROOT/'tests/vision/tb_perception_deadlines.v',ROOT/'fpga/vision/src/vision/vision_v23_bridge.v',
               ROOT/'fpga/vision/src/vision/vision_result_core.v']
    record=dict(passed=True,hardware_tested=False,oracle_reused=args.events_only,
        optical_flow_oracle=oracle,decoded_tracking_packets=len(targets),decoded_flow_status_packets=len(statuses),
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs if p.is_file()})
    (ROOT/'build/tests/perception_result.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print('PASS mixed RTL packets decoded by production Python parsers')


if __name__ == '__main__':
    main()
