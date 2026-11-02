"""Byte-exact 360-point telemetry/UDP/video regression; no hardware access."""
from pathlib import Path
import hashlib
import ipaddress
import json
import struct
import subprocess
import sys
import zlib

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/lidar_eth360_20261102'
sys.path.insert(0,str(ROOT))
from host.motor_protocol import crc16
from shared.streams import MixedStream
from shared.lidar_display import FpgaLidar
from shared.telemetry_udp import TelemetryUDP,HEADER


def checksum(data):
    data+=b'\x00'*(len(data)%2)
    total=sum(struct.unpack('!%dH'%(len(data)//2),data))
    while total>>16:total=(total&65535)+(total>>16)
    return ~total&65535


def ethernet(body):
    body=body.ljust(60,b'\x00')
    return b'\x55'*7+b'\xd5'+body+struct.pack('<I',zlib.crc32(body))


def fixtures():
    local=bytes.fromhex('020a3501fec0');peer=bytes.fromhex('021122334455')
    ip=ipaddress.IPv4Address
    lip,pip=ip('192.168.50.30').packed,ip('192.168.50.20').packed
    payload=struct.pack('!4sBBHI4sHH',b'NCFG',1,5,0,123,pip,6104,0)
    udp=struct.pack('!4H',44000,6200,28,0)+payload
    header=struct.pack('!BBHHHBBH4s4s',0x45,0,48,1,0,64,17,0,pip,lip)
    header=header[:10]+struct.pack('!H',checksum(header))+header[12:]
    command=ethernet(local+peer+b'\x08\x00'+header+udp)
    arp=ethernet(local+peer+b'\x08\x06'+struct.pack('!HHBBH',1,0x800,6,4,2)+peer+pip+local+lip)
    for name,data in (('set_endpoint',command),('arp_endpoint',arp)):
        (OUT/(name+'.hex')).write_text('\n'.join('%02x'%b for b in data))
    body=b'\xc3\x3c\x01'+bytes(43)
    frame=body+struct.pack('<H',crc16(body))
    (OUT/'motor.hex').write_text(frame[::-1].hex())


def main():
    OUT.mkdir(parents=True,exist_ok=True);fixtures()
    sources=['tests/navigation/tb_lidar_ethernet.sv','fpga/navigation/lidar_scan_360_source.sv',
             'fpga/motor/frame_transport.sv','fpga/motor/uart_byte.sv','fpga/system/telemetry_cdc.sv',
             'fpga/vision/src/network_runtime_dual.v','fpga/vision/src/eth_udp_gmii/eth_udp_tx_gmii.v',
             'fpga/vision/src/eth_udp_gmii/ip_checksum.v','fpga/vision/src/eth_udp_gmii/crc32_d8.v']
    for name,cmd in [('compile',['iverilog','-g2012','-Wall','-s','tb_lidar_ethernet','-o',str(OUT/'test.vvp')]+sources),
                     ('simulation',['vvp',str(OUT/'test.vvp')])]:
        result=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=120)
        (OUT/(name+'.log')).write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        print(result.stdout, end='')
    packets=[];packet=[]
    for line in (OUT/'tx.hex').read_text().splitlines():
        if line=='END':
            packets.append(bytes(packet));packet=[]
        else:packet.append(int(line,16))
    udp_decoder=TelemetryUDP();mixed=MixedStream();lidar=FpgaLidar();video=0;records=[];scans=[]
    for packet in packets:
        assert packet[:8]==b'\x55'*7+b'\xd5'
        frame=packet[8:]
        assert zlib.crc32(frame[:-4])==struct.unpack('<I',frame[-4:])[0],'Ethernet FCS'
        if frame[12:14]!=b'\x08\x00':continue
        assert checksum(frame[14:34])==0,'IPv4 checksum'
        source,destination,length,_=struct.unpack_from('!4H',frame,34)
        payload=frame[42:42+length-8]
        if source==5000:
            assert len(payload)==1456 and payload==bytes((i&255)^(i>>8) for i in range(1456))
            video+=1
        elif source==6205:
            assert destination==6205 and frame[30:34]==ipaddress.IPv4Address('192.168.50.20').packed
            tick=HEADER.unpack_from(payload)[-1]
            inner=udp_decoder.feed(payload,('192.168.50.30',6205),now=1+tick/1000.)
            assert inner is not None
            for kind,row in mixed.feed(inner):
                records.append(kind);previous=lidar.scans
                lidar.ingest(kind,row,now=10+len(records)*.00001)
                if lidar.scans>previous:scans.append(lidar.scan)
    assert video==1 and 'motor' in records
    assert len(scans)==2,(len(scans),records.count('lidar_scan'),lidar.incomplete)
    for index,scan in enumerate(scans):
        assert len(scan['points'])==360
        for i,p in enumerate(scan['points']):
            angle=0.0 if index==1 and i==0 else i+.37
            assert p==[angle,(index+1)*1000+i,77],(index,i,p)
    assert mixed.bad_crc==0 and udp_decoder.missing==0 and udp_decoder.rejected==0
    summary=dict(passed=True,programmed=False,hardware_tested=False,scans=len(scans),points_per_scan=360,
                 telemetry_datagrams=udp_decoder.accepted,video_packets=video,
                 ethernet_fcs_errors=0,ipv4_checksum_errors=0,mixed_crc_errors=mixed.bad_crc,
                 measured_angles_verified=True,source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources})
    (OUT/'regression.json').write_text(json.dumps(summary,indent=2))
    print('PASS Ethernet wire audit: two exact 360-point scans, measured angles, video bytes, FCS/IP/inner CRC')


if __name__=='__main__':main()
