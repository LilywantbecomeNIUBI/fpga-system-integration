"""Offline receiver/recording throughput measurement on this computer only."""
from pathlib import Path
import json,sys,tempfile,time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from nano.gateway import Gateway,Recorder
from shared.telemetry_udp import HEADER


def main():
    out=ROOT/'build/lidar_raw_ethernet_20261102'
    data=(out/'raw_recorded_frames.bin').read_bytes();assert len(data)==2000*48
    frames=[data[i:i+48] for i in range(0,len(data),48)]
    results={}
    for record in (False,True):
        with tempfile.TemporaryDirectory() as temp:
            gateway=Gateway(dict(telemetry_transport='udp',lidar_source='fpga'))
            if record:gateway.recorder=Recorder(Path(temp)/'record')
            began=time.perf_counter();processing=0.
            try:
                for i,frame in enumerate(frames):
                    if record:
                        delay=began+i/3600.-time.perf_counter()
                        if delay>0:time.sleep(delay)
                    tick=int(1000*(time.perf_counter()-began))
                    packet=HEADER.pack(b'TLM1',1,bytes(3),i,tick)+frame
                    step=time.perf_counter()
                    assert gateway.ingest_udp(packet,('192.168.50.30',6205))
                    processing+=time.perf_counter()-step
            finally:
                if record:gateway.recorder.close()
            received=time.perf_counter()-began
            if record:
                assert gateway.recorder.error is None and gateway.recorder.drops==0
                assert (gateway.recorder.path/'fpga_uart.bin').read_bytes()==data
                assert len((gateway.recorder.path/'events.jsonl').read_text().splitlines())==4000
            results['recording' if record else 'decoding']=dict(datagrams=2000,elapsed_seconds=received,
                datagrams_per_second=2000/received,offered_datagrams_per_second=3600 if record else 'unpaced burst',
                ingest_wall_seconds=processing,queue_drops=gateway.recorder.drops if record else 0)
    result=dict(passed=True,hardware_tested=False,measurement_host='development Windows PC; not a Nano measurement',results=results)
    (out/'receiver_benchmark.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
