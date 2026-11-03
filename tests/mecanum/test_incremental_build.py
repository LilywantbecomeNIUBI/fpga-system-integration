import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from tools.task4.incremental import attach


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


if __name__=='__main__':unittest.main()
