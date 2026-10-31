"""Local, bounded PC-camera recorder; no motor or network operations."""
import json,threading,time
from pathlib import Path

class WheelCamera:
    def __init__(self,index,folder):
        import cv2
        self.cv2=cv2;self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        self.capture=cv2.VideoCapture(index,cv2.CAP_DSHOW)
        if not self.capture.isOpened():raise RuntimeError('Wheel camera could not open')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,640);self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        ok,first=self.capture.read()
        if not ok:self.capture.release();raise RuntimeError('Wheel camera has no frame')
        self.path=self.folder/'wheel_camera.avi'
        height,width=first.shape[:2]
        self.writer=cv2.VideoWriter(str(self.path),cv2.VideoWriter_fourcc(*'MJPG'),30,(width,height))
        if not self.writer.isOpened():self.capture.release();raise RuntimeError('Video writer failed')
        self.times=[];self.stop=threading.Event();self.error=None;self.last=None
        self.save(first)
        cv2.imwrite(str(self.folder/'camera_before.jpg'),first)
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def save(self,frame):
        self.writer.write(frame);self.times.append(time.monotonic());self.last=frame
    def run(self):
        try:
            # A hard duration bound also applies if the caller fails to close.
            until=time.monotonic()+15
            while not self.stop.is_set() and time.monotonic()<until:
                ok,frame=self.capture.read()
                if not ok:raise RuntimeError('Camera frame read failed')
                self.save(frame)
        except Exception as exc:self.error=str(exc)
        finally:
            self.capture.release();self.writer.release()
            if self.last is not None:self.cv2.imwrite(str(self.folder/'camera_after.jpg'),self.last)
            (self.folder/'camera_timestamps.json').write_text(json.dumps(dict(
                video=str(self.path),monotonic=self.times,error=self.error,
                note='AVI nominal 30fps; use per-frame monotonic timestamps for actual timing'),indent=2),encoding='utf-8')
    def close(self):
        self.stop.set();self.thread.join(3)
        if self.thread.is_alive():raise RuntimeError('Camera recorder has not finished')
        return dict(video=str(self.path),frames=len(self.times),error=self.error)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index',type=int,default=1)
    parser.add_argument('--seconds',type=float,default=2)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 0<args.seconds<=10:parser.error('Duration must be 0..10 seconds')
    camera=WheelCamera(args.index,args.output)
    time.sleep(args.seconds)
    print(json.dumps(camera.close()))
