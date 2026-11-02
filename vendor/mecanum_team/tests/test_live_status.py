import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from live_status import LiveStatusWriter

class LiveStatusTests(unittest.TestCase):
    def test_export_distinguishes_cached_offline_and_demo_data(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'live_status.json';writer=LiveStatusWriter(path,interval=0)
            writer.write(dict(connected=True,demo=False,motor=dict(pwm=[0,0,0,0]),motor_age_ms=12))
            first=json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(first['export_sequence'],1);self.assertEqual(first['motor_age_ms'],12)
            writer.write(dict(connected=False,demo=True,motor_age_ms=5000),force=True)
            last=json.loads(path.read_text(encoding='utf-8'))
            self.assertFalse(last['connected']);self.assertTrue(last['demo'])
            self.assertEqual(last['export_sequence'],2);self.assertIn('exported_at_utc',last)
            self.assertEqual(list(Path(directory).glob('*.tmp')),[])

if __name__=='__main__':unittest.main()
