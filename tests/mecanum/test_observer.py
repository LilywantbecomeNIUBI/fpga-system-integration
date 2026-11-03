import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from host.mecanum.live_status import LiveStatusWriter,TelemetryJournal
from tools.mecanum.observe_live import select_status

class ObserverTests(unittest.TestCase):
    def test_stale_other_gui_cannot_hide_fresh_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'live_status.json'
            good=dict(connected=True,demo=False,motor={'armed':False},pose={'healthy':True},motor_age_ms=5,pose_age_ms=5)
            with patch('host.mecanum.live_status.os.getpid',return_value=22):LiveStatusWriter(path,interval=0).write(good)
            with patch('host.mecanum.live_status.os.getpid',return_value=11):LiveStatusWriter(path,interval=0).write(dict(good,motor_age_ms=10000))
            selected=select_status(Path(tmp));self.assertEqual(selected['state'],'fresh');self.assertEqual(selected['pid'],22)

    def test_demo_and_disconnected_are_not_real_feedback(self):
        for connected,demo in [(True,True),(False,False)]:
            with tempfile.TemporaryDirectory() as tmp:
                LiveStatusWriter(Path(tmp)/'live_status.json',interval=0).write(dict(connected=connected,demo=demo,motor={'x':0},pose={'x':0},motor_age_ms=0,pose_age_ms=0))
                self.assertEqual(select_status(Path(tmp))['state'],'no_fresh_telemetry')

    def test_journal_records_commands_and_feedback(self):
        with tempfile.TemporaryDirectory() as tmp:
            journal=TelemetryJournal(tmp)
            journal.record('command',op=0,seq=1);journal.record('telemetry',row={'kind':'motor','armed':False})
            path=journal.path;journal.close()
            rows=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual([r['kind'] for r in rows],['command','telemetry'])
            self.assertFalse(rows[1]['row']['armed'])

if __name__=='__main__':unittest.main()
