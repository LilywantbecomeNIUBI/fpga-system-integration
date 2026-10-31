import json,struct,sys,tempfile,threading,time,unittest
from pathlib import Path
from urllib.request import urlopen,Request
from urllib.error import HTTPError
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'host'))
from task4_console import Session,make_server
from protocol import crc16

def frame(fault=0,permit=True,packets=10,plans=1):
 b=b'\xc3\x3c\x04\x01'+struct.pack('<BBHiiihhhhHIIII',1|(4 if permit else 0),0,fault,65536,0,0,120,0,8,8,1500,plans,packets,0,0)
 return b+struct.pack('<H',crc16(b))
class Checks(unittest.TestCase):
 def test_stream_and_expiry(self):
  s=Session();data=frame()
  for part in [data[:7],data[7:]]:s.ingest(part,now=100)
  s.ingest(frame(packets=210,plans=101),now=101)
  a=s.snapshot(101);self.assertTrue(a['online']);self.assertEqual(a['rows']['nav']['x_mm'],1)
  self.assertIn('200.0',next(c for c in a['checks'] if c['name']=='雷达包推进')['detail'])
  self.assertFalse(s.snapshot(102)['online'])
  s.ingest(frame(fault=32,permit=False),now=102);self.assertIn('导航心跳租约到期',next(c for c in s.snapshot(102)['checks'] if c['name']=='FPGA输出许可')['detail'])
  with self.assertRaises(ValueError):s.send_stop()
 def test_unknown_and_crc(self):
  s=Session();self.assertFalse(s.snapshot()['online']);bad=bytearray(frame());bad[10]^=1;s.ingest(bytes(bad)+frame(permit=False))
  self.assertEqual(s.snapshot()['nav_bad_frames'],1)
  details=' '.join(c['detail'] for c in s.snapshot()['checks']);self.assertIn('无法区分',details);self.assertIn('未上传',details)
 def test_record_replay(self):
  with tempfile.TemporaryDirectory() as d:
   s=Session('live',out=Path(d)/'capture');s.ingest(frame());s.close()
   r=Session('replay',replay=Path(d)/'capture/chunks.jsonl');r.start();r.worker.join(2)
   self.assertTrue(r.finished);self.assertEqual(r.snapshot()['rows']['nav']['plans'],1);self.assertFalse(r.snapshot()['online'])
 def test_follow_and_reset(self):
  with tempfile.TemporaryDirectory() as d:
   path=Path(d)/'chunks.jsonl';path.write_text('',encoding='utf-8');s=Session('follow',replay=path);s.start()
   with path.open('a',encoding='utf-8') as f:f.write(json.dumps(dict(elapsed_s=0,hex=frame(packets=100).hex()))+'\n');f.flush()
   end=time.monotonic()+1
   while 'nav' not in s.rows and time.monotonic()<end:time.sleep(.01)
   self.assertTrue(s.snapshot()['online']);self.assertIsNone(s.connection);s.close()
  s=Session();s.ingest(frame(packets=100,plans=100),now=10);s.ingest(frame(packets=1,plans=1),now=11)
  self.assertEqual(len(s.history),1);self.assertTrue(any('计数回退' in e['message'] for e in s.events))
 def test_stop_packet(self):
  class Port:
   def write(self,data):self.data=data
   def flush(self):pass
  s=Session('live');s.connection=Port();s.send_stop();self.assertEqual(s.connection.data[3],0);self.assertEqual(len(s.connection.data),16)
 def test_http(self):
  s=Session();srv=make_server(s,0);thread=threading.Thread(target=srv.serve_forever,daemon=True);thread.start();base='http://127.0.0.1:'+str(srv.server_port)
  try:
   self.assertEqual(json.load(urlopen(base+'/api/state'))['mode'],'offline')
   html=urlopen(base).read().decode();self.assertNotIn('__KEY__',html)
   with self.assertRaises(HTTPError) as e:urlopen(Request(base+'/api/stop',data=b'{}',method='POST'))
   self.assertEqual(e.exception.code,403)
  finally:srv.shutdown();srv.server_close();s.close()
if __name__=='__main__':unittest.main()
