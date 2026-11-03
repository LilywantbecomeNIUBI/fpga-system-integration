import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import sys
from unittest.mock import patch,Mock
from types import SimpleNamespace
from tools.task4.incremental import attach,fallback_warning


class IncrementalBuild(unittest.TestCase):
    def make(self,root,name):
        folder=root/name;folder.mkdir()
        (folder/'manifest.json').write_text(json.dumps(dict(system=True,top='system_mecanum_top',pin_profile='mecanum_p7')))
        (folder/'sensor.cst').write_text('same pins\n');(folder/'sensor.sdc').write_text('same clocks\n')
        (folder/'build.tcl').write_text('open_project sensor.gprj\nrun all\nexit\n')
        return folder

    def test_reference_is_frozen_and_explicitly_used(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);old=self.make(root,'previous');new=self.make(root,'new')
            pr=old/'impl/pnr/sensor.pr';pr.parent.mkdir(parents=True);pr.write_bytes(b'completed reference')
            (old/'result.json').write_text(json.dumps(dict(actual_exit_code=0,violations=dict(setup=1,hold=0))))
            record=attach(new,pr)
            self.assertEqual(record['incremental_pnr']['reference_sha256'],hashlib.sha256(pr.read_bytes()).hexdigest())
            self.assertFalse(record['incremental_pnr']['full_rebuild_fallback'])
            self.assertIn('set_option -inc_pnr {incremental_base/sensor.pr}',(new/'build.tcl').read_text())
            pr.write_bytes(b'later build')
            self.assertEqual((new/'incremental_base/sensor.pr').read_bytes(),b'completed reference')

    def test_incompatible_or_incomplete_reference_cannot_start_pnr(self):
        for change in ('top','constraints','timeout','missing_pr'):
            with self.subTest(change=change),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);old=self.make(root,'previous');new=self.make(root,'new')
                pr=old/'impl/pnr/sensor.pr';pr.parent.mkdir(parents=True);pr.write_bytes(b'completed reference')
                result=dict(actual_exit_code=0)
                if change=='timeout':result.update(actual_exit_code=1,timed_out=True)
                (old/'result.json').write_text(json.dumps(result))
                if change=='top':
                    m=json.loads((old/'manifest.json').read_text());m['top']='system_task4_top';(old/'manifest.json').write_text(json.dumps(m))
                if change=='constraints':(old/'sensor.cst').write_text('different pins')
                if change=='missing_pr':pr.unlink()
                with self.assertRaises(ValueError):attach(new,old)
                self.assertNotIn('-inc_pnr',(new/'build.tcl').read_text())

    def test_vendor_clean_build_fallback_is_terminated_and_recorded(self):
        repo=Path(__file__).resolve().parents[2]
        with patch.object(sys,'path',[str(repo/'tools/task4'),*sys.path]):
            from tools.task4 import build_ethernet as runner
        warning='WARN (PA2136): Placement and routing matching(0.10) is less than the threshold value(80) needed to run incremental flow, the default placement and routing flow will be run.'
        for already_exited in (False,True):
            with self.subTest(already_exited=already_exited),tempfile.TemporaryDirectory() as tmp:
                out=Path(tmp);record=dict(system=True,incremental_pnr=dict(requested=True))
                child=Mock();child.returncode=0 if already_exited else None
                child.poll.side_effect=lambda:child.returncode
                child.terminate.side_effect=lambda:setattr(child,'returncode',-1)
                child.wait.return_value=-1
                def start(*args,**kwargs):
                    kwargs['stdout'].write(warning+'\n');kwargs['stdout'].flush();return child
                helper=SimpleNamespace(attach=lambda *args:record,fallback_warning=fallback_warning)
                with patch.object(sys,'argv',['build_ethernet.py','--system','--mecanum','--incremental-from',str(out)]),patch.dict(sys.modules,{'incremental':helper}),patch.object(runner,'prepare',return_value=(out,record)),patch.object(runner.subprocess,'run',return_value=SimpleNamespace(stdout='')),patch.object(runner.subprocess,'Popen',side_effect=start),patch.object(runner.time,'sleep'):
                    with self.assertRaisesRegex(RuntimeError,'fallback is prohibited'):runner.main()
                self.assertEqual(child.terminate.call_count,0 if already_exited else 1)
                result=json.loads((out/'result.json').read_text())
                self.assertFalse(result['passed']);self.assertTrue(result['incremental_fallback_rejected'])
                self.assertIn('PA2136',result['abort_reason'])

    def test_explicit_fast_full_build_does_not_attach_old_reference(self):
        repo=Path(__file__).resolve().parents[2]
        with patch.object(sys,'path',[str(repo/'tools/task4'),*sys.path]):
            from tools.task4 import build_ethernet as runner
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);record=dict(system=True)
            (out/'build.tcl').write_text('run all\nexit\n')
            helper=SimpleNamespace(attach=Mock(side_effect=AssertionError('old reference must not be used')))
            with patch.object(sys,'argv',['build_ethernet.py','--system','--mecanum','--full-rebuild','--place','0','--route-option','2','--prepare-only']),patch.dict(sys.modules,{'incremental':helper}),patch.object(runner,'prepare',return_value=(out,record)) as prepare,patch.object(runner.subprocess,'Popen') as process:
                runner.main()
            helper.attach.assert_not_called();process.assert_not_called()
            self.assertEqual(prepare.call_args.kwargs['place'],0)
            self.assertEqual(prepare.call_args.kwargs['route'],2)
            m=json.loads((out/'manifest.json').read_text())
            self.assertFalse(m['incremental_pnr']['requested']);self.assertTrue(m['incremental_pnr']['explicit_full_rebuild'])
            self.assertIn('set_option -inc_pnr 0',(out/'build.tcl').read_text())


if __name__=='__main__':unittest.main()
