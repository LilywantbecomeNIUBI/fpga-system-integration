import sys,unittest,socket
from pathlib import Path
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'host/mecanum'))
from transport import RouterLink
from console import packet

class RouterTests(unittest.TestCase):
    def test_router_binds_without_sending_or_importing_serial(self):
        status,command=Mock(),Mock()
        with patch('transport.socket.socket',side_effect=[status,command]):
            with RouterLink(bind='192.168.50.10') as link:
                status.bind.assert_called_once_with(('192.168.50.10',6205))
                command.send.assert_not_called()
                command.connect.assert_called_once_with(('192.168.50.30',6206))
                frame=packet(0,1);link.write(frame)
                command.send.assert_called_once_with(frame)
                with self.assertRaises(ValueError):link.write(frame+frame)
            status.close.assert_called_once();command.close.assert_called_once()

    def test_failed_command_open_closes_status_socket(self):
        status=Mock()
        with patch('transport.socket.socket',side_effect=[status,OSError('failed')]):
            with self.assertRaises(OSError):RouterLink().__enter__()
        status.close.assert_called_once()

    def test_monitor_timeout_is_empty(self):
        link=RouterLink();link.udp=Mock();link.udp.recvfrom.side_effect=socket.timeout
        self.assertEqual(link.read(256),b'')

    def test_endpoint_slot_selects_existing_protocol_without_touching_other_slot(self):
        from tools.task4 import telemetry_endpoint as endpoint
        import ipaddress
        for slot,op in [('pc',1),('nano',7)]:
            device=Mock()
            device.recv.return_value=endpoint.REPLY.pack(b'NSTA',1,op|128,0,1,23,
                ipaddress.IPv4Address('192.168.50.10').packed,6102,bytes(6),0,1)
            context=Mock();context.__enter__=Mock(return_value=device);context.__exit__=Mock(return_value=False)
            with patch.object(endpoint.socket,'socket',return_value=context),patch.object(endpoint.secrets,'randbits',return_value=23):
                state=endpoint.transact('192.168.50.10',slot=slot)
            self.assertEqual(state['slot'],slot)
            self.assertEqual(endpoint.REQUEST.unpack(device.send.call_args.args[0])[2],op)

if __name__=='__main__':unittest.main()
