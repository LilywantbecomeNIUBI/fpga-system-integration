"""Live transport adapter; frozen image/YOLO algorithms remain in vendor/vision."""
from collections import deque
import socket, struct, time
import cv2
import numpy as np
from camera_protocol import SegmentAssembler, newer, HEADER
from vision_dashboard_common import Capture as BaseCapture, jpeg


class EpochAssembler:
    """Confirm two complete advancing frames after a silent backward epoch jump."""
    def __init__(self, raw=True):
        self.raw=raw;self.active=SegmentAssembler(raw=raw);self.candidate=None
        self.last_arrival=None;self.candidate_first=None;self.reset_events=0

    def push(self, packet, now=None):
        now=time.monotonic() if now is None else now
        gap=self.last_arrival is not None and now-self.last_arrival>.5
        self.last_arrival=now
        # The normal packet path needs only the vendor assembler's one header decode.
        if not gap and self.candidate is None:return self.active.push(packet)
        fid=HEADER.unpack_from(packet)[3] if len(packet)>=16 and packet[:4]==b'CV\x02\x01' else None
        old=self.active.last_completed
        if self.candidate is not None and fid is not None and old is not None and fid>=old and newer(fid,old):
            self.candidate=None;self.candidate_first=None
        if (self.candidate is None and gap and fid is not None and old is not None
                and fid<old):
            self.candidate=SegmentAssembler(raw=self.raw);self.candidate_first=None
        if self.candidate is not None:
            result=self.candidate.push(packet)
            if result is not None:
                current=self.candidate.last_completed
                if self.candidate_first is not None and newer(current,self.candidate_first):
                    self.active=self.candidate;self.candidate=None;self.reset_events+=1
                    return result
                self.candidate_first=current
            return None
        return self.active.push(packet)

    def __getattr__(self, name):
        return getattr(self.active,name,0)


class Capture(BaseCapture):
    def __init__(self, bind, port=6102, source='192.168.50.30', encode_preview=True, buffer_kib=4096):
        self.kernel_queue_ms=deque(maxlen=256);self.frame_times=deque(maxlen=120);self.decode_ms=0.;self.epoch_seen=0
        self.buffer_kib=buffer_kib;self.catchup_packets=0;self.catchup_events=0
        super().__init__(bind,port,source,encode_preview)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,buffer_kib*1024)

    def receive(self):
        self.assembler=EpochAssembler(raw=True)
        ioctl=None
        try:
            import fcntl
            ioctl=fcntl.ioctl
        except ImportError:pass
        count=0;queue_ms=0.;waiting_first=False;storage=bytearray(2048);view=memoryview(storage)
        try:
            while not self.stop.is_set():
                try:length,peer=self.sock.recvfrom_into(storage)
                except socket.timeout:continue
                if peer[0]!=self.source:continue
                packet=view[:length]
                count+=1
                if ioctl is not None and count%32==0:
                    try:
                        # Linux asm-generic/sockios.h SIOCGSTAMPNS: last datagram's kernel arrival.
                        sec,nsec=struct.unpack('@ll',ioctl(self.sock.fileno(),0x8907,bytes(struct.calcsize('@ll'))))
                        age=(time.time()-sec-nsec/1e9)*1000
                        if 0<=age<10000:queue_ms=age;self.kernel_queue_ms.append(age)
                    except OSError:pass
                    if queue_ms>80:
                        # Drop queued OLD packets quickly, then resume at a whole frame.
                        # A smaller kernel buffer instead drops arbitrary rows and wastes frames.
                        self.catchup_events+=1;started=time.monotonic();self.sock.setblocking(False)
                        try:
                            while not self.stop.is_set() and time.monotonic()-started<.15:
                                try:self.sock.recvfrom_into(storage);self.catchup_packets+=1
                                except BlockingIOError:break
                        finally:self.sock.settimeout(.1)
                        queue_ms=0.;waiting_first=True
                        continue
                if waiting_first:
                    if len(packet)<16 or packet[:4]!=b'CV\x02\x01' or HEADER.unpack_from(packet)[4]!=0:continue
                    waiting_first=False
                raw=self.assembler.push(packet)
                if raw is None:continue
                if raw.shape!=(320,320):raise RuntimeError('Expected 320x320 RGB565 frame')
                t=time.monotonic()
                image=cv2.cvtColor(raw.view(np.uint8).reshape(320,320,2),cv2.COLOR_BGR5652BGR)
                encoded=jpeg(image) if self.encode_preview else None
                stamp=time.monotonic();self.decode_ms=(stamp-t)*1000
                if self.epoch_seen!=self.assembler.reset_events:
                    self.first=None;self.frame_times.clear();self.epoch_seen=self.assembler.reset_events
                if self.first is None:self.first=stamp
                self.last=stamp
                self.frame_times.append(stamp)
                # Keep freshness honest when the kernel queue is delayed.
                with self.lock:self.latest=(self.assembler.last_completed,image,stamp-queue_ms/1000,encoded)
        except Exception as exc:
            if not self.stop.is_set():self.error=str(exc)

    def status(self):
        result=super().status();entries=list(self.kernel_queue_ms);values=sorted(entries)
        recent=[t for t in list(self.frame_times) if time.monotonic()-t<2]
        result['fps']=(len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0.
        item=self.frame()
        if item:result['age_ms']=max(0,time.monotonic()-item[2])*1000
        result.update(receive_buffer_bytes=self.sock.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF),
            queue_ms_last=entries[-1] if entries else None,
            queue_ms_p50=values[len(values)//2] if values else None,
            queue_ms_p95=values[min(len(values)-1,int(len(values)*.95))] if values else None,
            decode_ms=self.decode_ms,epoch_resets=self.epoch_seen,
            catchup_events=self.catchup_events,catchup_packets=self.catchup_packets)
        return result
