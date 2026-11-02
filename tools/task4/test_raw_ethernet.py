"""Original recorded STL packets -> actual RTL -> UDP -> full-point host scan.

Offline only. The existing recording is used as a deterministic sensor fixture.
"""
from pathlib import Path
import hashlib,json,struct,subprocess,sys,zlib
from test_ethernet import checksum,fixtures,ROOT
OUT=ROOT/'build/lidar_raw_ethernet_20261102'
sys.path.insert(0,str(ROOT))
from shared.streams import MixedStream,LidarStream,crc8
from shared.lidar_display import FpgaLidar
from shared.telemetry_udp import HEADER,TelemetryUDP


def make_fixture():
    OUT.mkdir(exist_ok=True,parents=True)
    # Reuse the independent NCFG/ARP/video fixtures in a different output folder.
    import test_ethernet
    previous=test_ethernet.OUT;test_ethernet.OUT=OUT
    try:fixtures()
    finally:test_ethernet.OUT=previous
    source=Path('C:/Users/leo/Documents/ChatGPT/FPGA/STL27L_check_20261025/COM8_921600_5s.bin')
    raw=source.read_bytes();packets=[];pos=0
    while len(packets)<1000:
        pos=raw.find(b'\x54\x2c',pos)
        if pos<0:raise ValueError('Not enough raw STL packets')
        candidate=raw[pos:pos+47]
        if len(candidate)==47 and crc8(candidate[:46])==candidate[46]:packets.append(candidate);pos+=47
        else:pos+=1
    bad=bytearray(packets[300]);bad[-1]^=1
    data=b''.join(packets[:300])+bytes(bad)+b''.join(packets[300:])
    (OUT/'input.hex').write_text('\n'.join('%02x'%b for b in data));(OUT/'input.len').write_text(str(len(data)))
    return packets,data,dict(path=str(source),sha256=hashlib.sha256(raw).hexdigest())


def audit_packets(path):
    packets=[];current=[]
    for line in path.read_text().splitlines():
        if line=='END':packets.append(bytes(current));current=[]
        else:current.append(int(line,16))
    decoded=[];video=0;transport=TelemetryUDP();mixed=MixedStream();lidar=FpgaLidar();scans=[];recorded=[]
    for packet in packets:
        assert packet[:8]==b'\x55'*7+b'\xd5'
        frame=packet[8:];assert zlib.crc32(frame[:-4])==struct.unpack('<I',frame[-4:])[0]
        if frame[12:14]!=b'\x08\x00':continue
        assert checksum(frame[14:34])==0
        source,port,length,_=struct.unpack_from('!4H',frame,34);payload=frame[42:42+length-8]
        if source==5000:
            assert payload==bytes((i&255)^(i>>8) for i in range(1456));video+=1
        elif source==6202:
            assert len(payload)==32
            decoded.append(('perception_event',dict(magic=payload[:4].decode('ascii'))))
        elif source==6205:
            tick=HEADER.unpack_from(payload)[-1]
            inner=transport.feed(payload,('192.168.50.30',6205),now=1+tick/1000.)
            assert inner is not None and port==6205
            recorded.append(inner)
            for kind,row in mixed.feed(inner):
                decoded.append((kind,row));before=lidar.scans
                lidar.ingest(kind,row,now=1+tick/1000.)
                if lidar.scans>before:scans.append(lidar.scan)
    assert transport.missing==transport.rejected==mixed.bad_crc==0
    return decoded,scans,video,recorded


def main():
    originals,data,fixture=make_fixture()
    sources=['tests/navigation/tb_lidar_raw_ethernet.sv','fpga/navigation/lidar_points_source.sv',
        'fpga/navigation/lidar/stl27l_node_parser.v','fpga/motor/frame_transport.sv','fpga/motor/uart_byte.sv',
        'fpga/system/telemetry_cdc.sv','fpga/vision/src/network_runtime_dual.v',
        'fpga/vision/src/eth_udp_gmii/eth_udp_tx_gmii.v','fpga/vision/src/eth_udp_gmii/ip_checksum.v','fpga/vision/src/eth_udp_gmii/crc32_d8.v']
    for name,command in [('raw_compile',['iverilog','-g2012','-Wall','-s','tb_lidar_raw_ethernet','-o',str(OUT/'raw.vvp')]+sources),
                         ('raw_sim',['vvp',str(OUT/'raw.vvp')])]:
        result=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=150)
        (OUT/(name+'.log')).write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        print(result.stdout,end='',flush=True)
    rows,scans,video,recorded=audit_packets(OUT/'tx.hex')
    raw_rows=[row for kind,row in rows if kind=='lidar_raw']
    events=sum(kind=='perception_event' for kind,row in rows)
    assert len(raw_rows)==2000 and video==1 and events>1000
    # Independently decode each original packet, using the parser's documented
    # nearest-centidegree interpolation, then compare every emitted tuple.
    for packet_index,original in enumerate(originals):
        start=struct.unpack_from('<H',original,4)[0];end=struct.unpack_from('<H',original,42)[0]
        span=(end-start)%36000
        for half in (0,1):
            row=raw_rows[packet_index*2+half]
            assert row['packet_sequence']==packet_index+1 and row['half']==half
            assert row['discontinuity']==(packet_index==300)
            expected=[]
            for i in range(half*6,half*6+6):
                angle=(start+(i*span+5)//11)%36000
                d,q=struct.unpack_from('<HB',original,6+i*3);expected.append([angle/100.,d,q])
            assert row['points']==expected,(packet_index,half)
    reference=LidarStream();expected_scans=reference.feed(data)
    assert reference.bad_crc==1 and len(scans)==len(expected_scans),(len(scans),len(expected_scans))
    for actual,expected in zip(scans,expected_scans):
        assert len(actual['points'])==len(expected['points'])
        for a,e in zip(actual['points'],expected['points']):
            assert a[1:]==e[1:] and abs((a[0]-e[0]+180)%360-180)<=.005001
    result=dict(passed=True,programmed=False,hardware_tested=False,fixture=fixture,
                native_packets=1000,point_frames=len(raw_rows),all_points_checked=12000,
                full_scans=len(scans),points_per_scan=[len(scan['points']) for scan in scans],
                video_packets=video,continuous_perception_event_packets=events,deliberate_native_crc_errors=1,ethernet_crc_errors=0,
                received_inner_crc_errors=0,source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources})
    (OUT/'raw_regression.json').write_text(json.dumps(result,indent=2))
    # Exact accepted frames remain compatible with mixed-stream offline replay.
    (OUT/'raw_recorded_frames.bin').write_bytes(b''.join(recorded))
    print('PASS raw Ethernet audit: all 12000 points, original distances/intensities, measured angles, complete native-density scans, video and CRC')


if __name__=='__main__':main()
