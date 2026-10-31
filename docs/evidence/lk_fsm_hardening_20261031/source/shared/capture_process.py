"""Independent UDP assembly process; bounded IPC keeps only complete recent frames."""
import multiprocessing, queue, threading, time
from collections import deque
from shared.live_capture import Capture


def _publish(channel, value):
    try:channel.put_nowait(value);return True
    except queue.Full:
        try:channel.get_nowait()
        except queue.Empty:pass
        try:channel.put_nowait(value);return True
        except queue.Full:return False


def _worker(channel, stop, config):
    capture=None
    try:
        capture=Capture(**config)
        _publish(channel,dict(ready=True))
        previous=None
        while not stop.wait(.003):
            frame=capture.frame()
            if capture.error:raise RuntimeError(capture.error)
            if not frame:continue
            key=(capture.epoch_seen,frame[0])
            if key==previous:continue
            if _publish(channel,dict(frame=frame,status=capture.status())):previous=key
    except Exception as exc:
        _publish(channel,dict(error=repr(exc)))
    finally:
        if capture:capture.close()
        channel.cancel_join_thread()


class CaptureProcess:
    def __init__(self,bind,port=6102,source='192.168.50.30',encode_preview=False,buffer_kib=4096):
        context=multiprocessing.get_context('spawn')
        self.channel=context.Queue(maxsize=1);self.stop=context.Event()
        self.lock=threading.Lock();self.latest=None;self.error=None;self.ready=False;self.delivered=0
        self.last_status={};self.first=None;self.last=None
        self.delivered_times=deque(maxlen=120)
        self.process=context.Process(target=_worker,args=(self.channel,self.stop,
            dict(bind=bind,port=port,source=source,encode_preview=encode_preview,buffer_kib=buffer_kib)))
        self.process.daemon=True;self.process.start()
        self.thread=threading.Thread(target=self.receive,daemon=True);self.thread.start()

    def receive(self):
        while not self.stop.is_set():
            try:item=self.channel.get(timeout=.1)
            except queue.Empty:
                if not self.process.is_alive():self.error='UDP assembly worker exited';return
                continue
            if item.get('ready'):self.ready=True
            if item.get('error'):self.error=item['error'];return
            if 'frame' in item:
                self.ready=True
                with self.lock:
                    self.latest=item['frame'];self.last_status=item['status'];self.delivered+=1
                    now=time.monotonic()
                    if self.first is None:self.first=now
                    self.last=now
                    self.delivered_times.append(now)

    def frame(self):
        with self.lock:return self.latest

    def status(self):
        with self.lock:
            status=dict(self.last_status)
            child_frames=status.get('frames',0)
            recent=[t for t in self.delivered_times if time.monotonic()-t<2]
            receiver_fps=status.get('fps',0)
            status.update(frames=self.delivered,receiver_frames=child_frames,
                receiver_fps=receiver_fps,fps=(len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0.,
                packets=status.get('packets',0),bad=status.get('bad',0),missing_frames=status.get('missing_frames',0),
                frame_id=self.latest[0] if self.latest else None,
                age_ms=max(0,time.monotonic()-self.latest[2])*1000 if self.latest else None,
                error=self.error,receiver_process=True,receiver_pid=self.process.pid,receiver_ready=self.ready)
            return status

    def close(self):
        self.stop.set();self.thread.join(1);self.process.join(3)
        if self.process.is_alive():self.process.terminate();self.process.join(2)
        self.channel.close()
