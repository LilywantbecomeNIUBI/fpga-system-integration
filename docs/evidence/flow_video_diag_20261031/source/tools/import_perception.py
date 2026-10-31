"""Explicit pinned Git import; reads committed blobs, never copies a dirty worktree."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
REVISION = 'df6f60d07873150d9ea796580adbc17960945f2c'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    args = parser.parse_args()
    manifest = ROOT/'docs/perception_sources.json'
    if manifest.exists():
        raise SystemExit('Snapshot exists; review adaptations before importing again')
    git = ['git', '-C', str(args.source.resolve())]
    head = subprocess.check_output(git+['rev-parse', REVISION], text=True).strip()
    files = {}
    def copy(source, destination):
        data = subprocess.check_output(git+['show', head+':'+source])
        path = ROOT/destination
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        files[destination] = dict(source=source, sha256=hashlib.sha256(data).hexdigest())
    paths = subprocess.check_output(git+['ls-tree', '-r', '--name-only', head, 'src'], text=True).splitlines()
    for source in paths:
        if source.endswith('.v'):
            copy(source, 'fpga/vision/'+source)
    copy('reference/optical_flow/upstream/rtl/frame_memory.sv', 'fpga/vision/src/optical_flow/frame_memory.sv')
    copy('reference/optical_flow/upstream/LICENSE', 'vendor/notices/optical-flow-fpga/LICENSE')
    copy('reference/optical_flow/model.py', 'vendor/vision/lk_model.py')
    for source in ('optical_flow_protocol.py', 'optical_flow_state.py'):
        copy('pc/'+source, 'vendor/vision/'+source)
    for source in ('vision_dashboard.py', 'vision_dashboard_common.py', 'vision_v23_protocol.py',
                   'vision_protocol.py', 'vision_feedback.py', 'vision_models.py', 'vision_tensorrt.py'):
        copy('pc/'+source, 'vendor/vision/'+source)
    for source in ('vision_dashboard_agent.py', 'vision_dashboard_service.py'):
        copy('nano/'+source, 'nano/vision/'+source)
    for name in ('tb_vision_lk_tracker.v', 'tb_vision_v23_network.v'):
        copy('sim/'+name, 'tests/vision/'+name)
    for name in ('tb_flow_patch.v', 'tb_flow_pipeline.v', 'tb_flow_avoidance.v', 'tb_flow_events.v'):
        copy('sim/optical_flow/'+name, 'tests/vision/'+name)
    copy('scripts/test_optical_flow.py', 'vendor/reference/test_optical_flow_original.py')
    for name in ('vision_lk_tracker.md', 'optical_flow/README.md'):
        copy('docs/'+name, 'vendor/reference/'+name.replace('/', '_'))
    record = dict(repository='fpga-vision', remote='https://github.com/LilywantbecomeNIUBI/fpga-vision.git',
                  head=head, import_method='git show exact committed blobs', files=files)
    manifest.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    # Keep the old provenance; this manifest identifies each explicit replacement.
    old_path = ROOT/'docs/integration_sources.json'
    old = json.loads(old_path.read_text(encoding='utf-8'))
    for path in files:
        if path in old['files']:
            old['files'][path].update(adapted=True, superseded_by='docs/perception_sources.json')
    old_path.write_text(json.dumps(old, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('Imported', len(files), 'committed files at', head)


if __name__ == '__main__':
    main()
