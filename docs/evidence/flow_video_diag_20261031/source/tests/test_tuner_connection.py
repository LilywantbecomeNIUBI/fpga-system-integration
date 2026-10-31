import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from dashboard import GatewayClient
from motor_tuner import TunerSession


class ConnectionTests(unittest.TestCase):
    def test_disconnect_discards_lease_and_recovery_never_replays_drive(self):
        gateway=GatewayClient('http://192.168.50.20:8873','x'*32,True)
        gateway.lease='old-owner'
        gateway.request=Mock(side_effect=TimeoutError('offline'))
        gateway.poll()
        self.assertIsNone(gateway.lease)
        self.assertFalse(gateway.snapshot()['online'])
        self.assertIsNone(gateway.snapshot()['data'])
        self.assertEqual(gateway.request.call_args_list[-1].args[1]['action'],'release')
        gateway.request=Mock(return_value={'profile':'system_integration_v1','control':{'phase':'stopped'}})
        gateway.poll()
        self.assertTrue(gateway.snapshot()['online'])
        gateway.request.assert_called_once_with('/api/state')
        self.assertIsNone(gateway.lease)

    def test_switch_validates_address_releases_old_and_preserves_credentials(self):
        # No polling worker or real network in this test.
        session=TunerSession.__new__(TunerSession)
        session.gateway=GatewayClient('http://192.168.50.20:8873','x'*32,True)
        session.gateway.lease='old'
        session.gateway.request=Mock(return_value={})
        session.connection_lock=threading.RLock()
        session.reconnect_event=threading.Event()
        session.connection_generation=0
        session.connection_attempts=5
        with tempfile.TemporaryDirectory() as folder:
            session.config_path=Path(folder)/'local.json'
            session.config_path.write_text(json.dumps({'gateway_url':session.gateway.url,'token_file':'secret.json'}))
            for invalid in ('https://172.20.10.2:8873','http://example.com:8873','http://8.8.8.8:8873',
                            'http://user:password@192.168.50.20:8873','http://192.168.50.20:80',
                            'http://192.168.50.20:8873/api/control'):
                with self.assertRaises(ValueError):session.configure_connection({'gateway_url':invalid})
            self.assertEqual(session.gateway.lease,'old')
            result=session.configure_connection({'gateway_url':'http://172.20.10.2:8873/'})
            self.assertEqual(result['generation'],1)
            self.assertEqual(session.gateway.url,'http://172.20.10.2:8873')
            self.assertIsNone(session.gateway.lease)
            self.assertFalse(session.gateway.snapshot()['online'])
            self.assertTrue(session.reconnect_event.is_set())
            saved=json.loads(session.config_path.read_text())
            self.assertEqual(saved['token_file'],'secret.json')
            self.assertNotIn('token',saved)
            session.gateway.request.assert_called_once_with('/api/control',{'action':'release','lease':'old'})
