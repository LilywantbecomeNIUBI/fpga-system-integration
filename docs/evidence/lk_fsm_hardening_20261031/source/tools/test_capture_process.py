"""Loopback smoke: real CV row datagrams -> independent assembler -> bounded frame IPC."""
from pathlib import Path
import json,socket,struct,sys,time
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'vendor/vision')]
from shared.capture_process import CaptureProcess

def main():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reservation:
        reservation.bind(('127.0.0.1',0));port=reservation.getsockname()[1]
    capture=CaptureProcess('127.0.0.1',port=port,source='127.0.0.1')
    try:
        limit=time.monotonic()+10
        while not capture.ready and time.monotonic()<limit:
            if capture.error:raise RuntimeError(capture.error)
            time.sleep(.02)
        assert capture.ready,'Worker failed to bind'
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as source:
            for fid in range(30):
                started=time.monotonic()
                for row in range(320):
                    packet=struct.pack('<2sBB6H',b'CV',2,1,fid,row,0,320,320,320)+b'\x00\xf8'*320
                    source.sendto(packet,('127.0.0.1',port))
                time.sleep(max(0,1/30-(time.monotonic()-started)))
        limit=time.monotonic()+3
        while time.monotonic()<limit and (capture.frame() is None or capture.frame()[0]<29):time.sleep(.01)
        frame=capture.frame();assert frame is not None and frame[0]>=27,capture.status()
        assert frame[1].shape==(320,320,3)
        assert tuple(frame[1][160,160])==(0,0,248)
        assert capture.status()['receiver_frames']>=25,capture.status()
        print('PASS isolated UDP assembler: real 30fps row stream, exact RGB565 image, recent bounded IPC frame')
        print(json.dumps(capture.status(),indent=2))
    finally:capture.close()

if __name__=='__main__':main()
