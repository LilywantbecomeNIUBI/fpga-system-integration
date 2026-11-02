"""Wire-compatible CRC acceleration and lossless buffered recording."""
import json,random,tempfile,time,unittest
from pathlib import Path
from host.motor_protocol import crc16
from tools.task4.protocol import crc16 as nav_crc16
from nano.gateway import Recorder


def reference(data):
    value=0xffff
    for byte in data:
        value^=byte<<8
        for _ in range(8):value=((value<<1)^0x1021)&65535 if value&0x8000 else (value<<1)&65535
    return value


class ThroughputTests(unittest.TestCase):
    def test_fast_crc_matches_independent_bitwise_reference(self):
        rng=random.Random(6205)
        samples=[b'',b'123456789']+[bytes([i]) for i in range(256)]
        samples+=[bytes(rng.randrange(256) for _ in range(n)) for n in (14,46,48,64,255,512,4096)]
        self.assertEqual(crc16(b'123456789'),0x29b1)
        for data in samples:
            for form in (data,bytearray(data),list(data),memoryview(data)):
                self.assertEqual(crc16(form),reference(data));self.assertEqual(nav_crc16(form),reference(data))

    def test_buffered_recorder_preserves_all_offsets_and_tail_on_close(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=Recorder(Path(temp)/'record');frames=[]
            for i in range(1500):
                while recorder.queue.qsize()>500:time.sleep(.001)
                data=i.to_bytes(4,'little')+bytes(44);frames.append(data)
                recorder.put('uart',data);recorder.put('udp_telemetry',dict(sequence=i))
            recorder.close()
            self.assertFalse(recorder.thread.is_alive());self.assertIsNone(recorder.error);self.assertEqual(recorder.drops,0)
            self.assertEqual((recorder.path/'fpga_uart.bin').read_bytes(),b''.join(frames))
            rows=[json.loads(line) for line in (recorder.path/'events.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),3000)
            for i in range(1500):
                self.assertEqual(rows[2*i]['data'],dict(offset=i*48,length=48,file='fpga_uart.bin'))
                self.assertEqual(rows[2*i+1]['data'],dict(sequence=i))


if __name__=='__main__':unittest.main()
