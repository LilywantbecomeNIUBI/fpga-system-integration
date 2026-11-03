"""Freeze-only checks; no Gowin, programmer, sockets or serial devices."""
from pathlib import Path
import json, re, sys, unittest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/task4'))
from tools.task4.build_ethernet import prepare


class BuildProfiles(unittest.TestCase):
    def test_navigation_cdc_constraints_are_opt_in(self):
        from tools.mecanum.build import constraints
        plain=constraints()[2];navigation=constraints(navigation=True)[2]
        self.assertNotIn('navigation_safety',plain)
        self.assertIn('navigation_safety_cdc/held*/Q',navigation)
        self.assertIn('10.0',navigation)
        self.assertNotEqual(plain,navigation)

    def test_real_capture_network_and_teammate_pins_in_both_modes(self):
        for raw in (False,True):
            folder,meta=prepare(system=True,raw=raw,mecanum=True,clock_cleanup=True)
            cst=(folder/'sensor.cst').read_text()
            self.assertIn('ethernet_raw.lidar_display/capture_crc' if raw else 'ethernet_scan.lidar_display/capture',cst)
            self.assertNotIn('ethernet_scan.lidar_display/capture' if raw else 'ethernet_raw.lidar_display/capture_crc',cst)
            self.assertNotIn('chassis/capture',cst)
            self.assertEqual(meta['top'],'system_mecanum_top')
            top=(folder/'source/fpga/mecanum/system_mecanum_top.sv').read_text()
            self.assertIn('LIDAR_RAW=%d'%raw,top)
            mapping={p['signal']:p['ball'] for p in json.loads((folder/'pin_audit.json').read_text())['pins']}
            for p in json.loads((ROOT/'hardware/mecanum.json').read_text())['pins']:
                self.assertEqual(mapping[p['signal']],p['ball'])
            team=(ROOT/'vendor/mecanum_team/project/mecanum.cst').read_text()
            for statement,name in re.findall(r'(IO_PORT "([^"]+)"[^;]+;)',team):
                new={'imu_scl':'imu_i2c_scl','imu_sda':'imu_i2c_sda'}.get(name,name)
                self.assertIn(statement.replace('"'+name+'"','"'+new+'"'),cst)
            self.assertNotIn('left_enc_a',mapping)


if __name__=='__main__':unittest.main()
