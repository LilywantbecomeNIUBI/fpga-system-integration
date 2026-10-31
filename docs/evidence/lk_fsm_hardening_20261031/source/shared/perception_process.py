"""Independent read-only UDP event consumer; latest snapshots cross the process boundary."""
import copy
import json
import multiprocessing
import os
from pathlib import Path
from queue import Empty,Full
import socket
import struct
import time
from shared.perception import PerceptionEvents


def _publish(queue,value):
    try:queue.put_nowait(value);return
    except Full:pass
    try:queue.get_nowait()
    except Empty:pass
    try:queue.put_nowait(value)
    except Full:pass


def _receive(bind,source,queue,stop,record,relay):
    queue.cancel_join_thread()
    events=PerceptionEvents();sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    raw=relay_socket=None;packets=bad=0;last=0;queue_age=0;relay_stale_drops=relay_errors=0
    try:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024)
        sock.bind((bind,6202));sock.settimeout(.02)
        if relay:
            relay_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            relay_socket.bind((bind,6203))
            relay_socket.setblocking(False)
        if record:raw=Path(record).open('a',buffering=65536)
        try:import fcntl
        except ImportError:fcntl=None
        while not stop.is_set():
            try:
                payload,peer=sock.recvfrom(2048)
                if peer!=(source,6202):continue
                received=time.monotonic();packets+=1
                if fcntl is not None and payload[:4]!=b'OFLW':
                    try:
                        sec,nsec=struct.unpack('@ll',fcntl.ioctl(sock.fileno(),0x8907,b'\0'*struct.calcsize('@ll')))
                        queue_age=max(0,time.time()-(sec+nsec/1e9));received-=queue_age
                    except (OSError,struct.error):pass
                try:
                    events.ingest(payload,now=received)
                    if relay:
                        # Do not present queued old control as newly received control.
                        if payload[:4]==b'OFST' and queue_age>=.15:relay_stale_drops+=1
                        else:
                            try:relay_socket.sendto(payload,relay)
                            except OSError:relay_errors+=1
                except ValueError:bad+=1
                if raw and payload[:4]!=b'OFLW':
                    raw.write(json.dumps(dict(time=time.time(),received_mono=received,hex=payload.hex()))+'\n')
            except socket.timeout:pass
            now=time.monotonic()
            if now-last>=1/30.:
                last=now
                _publish(queue,dict(stamp=now,flow=events.flow_snapshot(now),event=events.tracking,
                    event_stamp=events.tracking_stamp,stats=dict(packets=packets,bad=bad,queue_ms=queue_age*1000,
                    receive_buffer_bytes=sock.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF),pid=os.getpid(),relay_stale_drops=relay_stale_drops,relay_errors=relay_errors),error=None))
                if raw:raw.flush()
    except Exception as exc:_publish(queue,dict(error=repr(exc)))
    finally:
        sock.close()
        if relay_socket:relay_socket.close()
        if raw:raw.close()


class PerceptionProcess:
    def __init__(self,bind,source,record=None,relay=None):
        ctx=multiprocessing.get_context('spawn');self.queue=ctx.Queue(maxsize=1);self.stop=ctx.Event()
        self.latest=None;self.fallback=PerceptionEvents()
        self.process=ctx.Process(target=_receive,args=(bind,source,self.queue,self.stop,str(record) if record else None,relay),daemon=True)
        self.process.start()

    def snapshot(self,now=None):
        now=time.monotonic() if now is None else now
        while True:
            try:self.latest=self.queue.get_nowait()
            except Empty:break
        data=self.latest or {};flow=copy.copy(data.get('flow') or self.fallback.flow_snapshot(now))
        elapsed=max(0,now-data.get('stamp',now))*1000
        flow['control']=dict(flow['control']);c=flow['control']
        if c['age_ms'] is not None:c['age_ms']+=elapsed
        if data.get('error') or not self.process.is_alive() or c['age_ms'] is None or c['age_ms']>=200:
            c.update(fresh=False,confident=False,state='unknown',stop_request=True,steer=0,speed_limit=0)
        if flow.get('diagnostic'):flow['diagnostic']=dict(flow['diagnostic'],age_ms=flow['diagnostic']['age_ms']+elapsed)
        stamp=data.get('event_stamp')
        return dict(flow=flow,event=data.get('event'),event_age_ms=max(0,now-stamp)*1000 if stamp is not None else None,
                    stats=data.get('stats'),error=data.get('error'))

    def close(self):
        self.stop.set();self.process.join(3)
        if self.process.is_alive():self.process.terminate();self.process.join(2)
        self.queue.close();self.queue.cancel_join_thread()
