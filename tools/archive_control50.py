"""Archive the reviewed control50 candidate and evidence, excluding local secrets."""
import gzip,hashlib,json,shutil
from pathlib import Path
from build_system import ROOT
def main():
    root=Path((ROOT/'runs/commissioning/control50_latest.txt').read_text())
    dest=ROOT/'docs/evidence/control50_migration_20261031';dest.mkdir(parents=True,exist_ok=True)
    def copy(src,rel):
        target=dest/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,target)
    def compress(src,rel):
        target=dest/rel;target.parent.mkdir(parents=True,exist_ok=True)
        with src.open('rb') as i,gzip.open(target,'wb',compresslevel=9) as o:shutil.copyfileobj(i,o)
    identity=json.loads((root/'source_identity.json').read_text())
    for rel,h in identity.items():
        p=ROOT/rel;assert hashlib.sha256(p.read_bytes()).hexdigest()==h,rel
        assert p.suffix in ('.v','.sv','.sdc','.cst')
        copy(p,'source/'+rel)
    extras=['config/pnr.json','docs/LK_CONTROL50_MIGRATION_20261031.md','docs/LK_STABLE_CLOCK_PROPOSAL_20261031.md',
            'docs/integration_sources.json','docs/perception_sources.json','vendor/vision/lk_model.py',
            'tools/archive_control50.py','tools/build_system.py','tools/gowin_runner.py','tools/pnr_control50.py','tools/audit_control50.py',
            'tools/audit_control50_timing.py','tools/audit_lk_state.py','tools/test_frame_cdc_synth.py',
            'tools/test_lk_math_synth.py','tools/test_flow_recovery_synth.py','tools/test_stable_cdc.py',
            'tools/test_mailbox_contract.py','tools/observe_control50.py','tools/test_system.py','tools/test_full_top.py','tools/test_perception.py',
            'tests/tb_full_system.sv','tests/ethernet_pll_model.v','tests/imu/icm42688_model.sv']
    extras += [p.relative_to(ROOT).as_posix() for p in (ROOT/'tests/vision').glob('*.v')]
    for rel in extras:copy(ROOT/rel,'source/'+rel)
    for n in ['comparison.json','source_identity.json','board_selection.json','program_result.json','programmer.txt',
              'nano_launch.json','raw_archive.json','raw_packet_audit.json','digital_audit_interim.json','verification_audit.json']:
        if (root/n).exists():copy(root/n,n)
    compress(root/'frozen.vg','synthesis/system.vg.gz')
    for place in [2,3,4]:
        work=root/('place_'+str(place))
        for name in ['result.json','timing_audit.json','build.tcl','system.sdc','system.cst']:
            copy(work/name,'place_'+str(place)+'/'+name)
        for name in ['system.tr','system_tr_content.html']:
            compress(work/'impl/pnr'/name,'place_'+str(place)+'/'+name+'.gz')
        for name in ['system.log','system.rpt.txt','cmd.do']:
            copy(work/'impl/pnr'/name,'place_'+str(place)+'/'+name)
    for n in ['stable_cdc','mailbox_contract','control50_audit','lk_state_audit','frame_cdc_synth',
              'flow_recovery_synth','lk_math_synth','perception_result','system','full_top_result']:
        copy(ROOT/'build/tests'/(n+'.json'),'tests/'+n+'.json')
    for n in ['stable_cdc_0.txt','stable_cdc_1.txt','stable_cdc_2.txt','frame_cdc_synth.txt','lk_math_synth.txt',
              'flow_recovery_synth.log','full_top.log']:
        copy(ROOT/'build/tests'/n,'tests/'+n)
    copy(ROOT/'build/tests/optical_flow/result.json','tests/optical_flow_result.json')
    copy(ROOT/'build/tests/optical_flow/source_identity.json','tests/optical_flow_source_identity.json')
    for n in ['stable50_perception_final.txt','stable50_system_final.txt','stable50_mailbox_contract.txt']:
        copy(ROOT/'runs/commissioning'/n,'tests/'+n)
    for folder in ['concurrent_300s','extended_monitor']:
        if (root/folder/'result.json').exists():
            copy(root/folder/'result.json',folder+'/result.json');compress(root/folder/'observations.jsonl',folder+'/observations.jsonl.gz')
    if (root/'board_raw_events.jsonl.gz').exists():copy(root/'board_raw_events.jsonl.gz','board_raw_events.jsonl.gz')
    # The allowlist above never includes session_token.json, local configs,
    # commissioning scripts with credentials, or the vendor simulation library.
    files={p.relative_to(dest).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
           for p in dest.rglob('*') if p.is_file() and p.name!='manifest.json'}
    manifest=dict(files=files,build_folder=str(root),bitstream_sha256=json.loads((root/'program_result.json').read_text())['bitstream_sha256'],
                  excluded=['local credentials/configs','vendor primitive simulation library','40MB bitstream (kept in build; hash archived)'])
    (dest/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('Archived',len(files),'files in',dest)
if __name__=='__main__':main()
