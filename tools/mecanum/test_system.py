"""Full production Mecanum top -> real UDP packets -> host codecs, offline.

PHY initialization and camera bytes are injected as in the existing system
regression. This tests integration, not physical PHY startup or camera wiring.
"""
from pathlib import Path
import collections, json, struct, subprocess, sys
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/task4')]
from test_raw_ethernet import OUT, audit_packets
from test_ethernet import fixtures
from test_display import packet


def main():
    folder=Path(sys.argv[1]).resolve()
    meta=json.loads((folder/'manifest.json').read_text())
    assert meta['top']=='system_mecanum_top'
    import test_ethernet
    old=test_ethernet.OUT
    OUT.mkdir(parents=True,exist_ok=True)
    test_ethernet.OUT=OUT
    try:fixtures()
    finally:test_ethernet.OUT=old
    originals=[packet((i%36)*1000) for i in range(1000)]
    bad=bytearray(originals[300]);bad[-1]^=1
    data=b''.join(originals[:300])+bytes(bad)+b''.join(originals[300:])
    (OUT/'input.hex').write_text('\n'.join('%02x'%b for b in data))
    (OUT/'input.len').write_text(str(len(data)))
    source=(ROOT/'tests/navigation/tb_system_raw_ethernet.sv').read_text()
    source=source.replace('tb_system_raw_ethernet','tb_system_mecanum')
    source=source.replace('system_task4_top #(.CLK_HZ(1000000),.TELEMETRY_ETHERNET(1),.LIDAR_RAW(1))',
                          'system_mecanum_top #(.CLK_HZ(1000000),.LIDAR_RAW(0))')
    source=source.replace('wire l1,l2,r1,r2,legacy,tx;wire [5:0] old_pwm;wire old_cal;',
        'wire tx,legacy,legacy2;wire [3:0] in1,in2;wire [5:0] old_pwm,old_pwm2;wire [1:0] old_cal;')
    source=source.replace(".left_enc_a(1'b0),.left_enc_b(1'b0),.right_enc_a(1'b0),.right_enc_b(1'b0)",".enc_a(4'b0),.enc_b(4'b0)")
    source=source.replace('.left_in1(l1),.left_in2(l2),.right_in1(r1),.right_in2(r2)', '.in1(in1),.in2(in2)')
    source=source.replace('.legacy_m1_disable(legacy),.legacy_m1_pwm_disabled(old_pwm)',
        '.legacy_m1_disable(legacy),.legacy_m2_disable(legacy2),.legacy_m1_pwm_disabled(old_pwm),.legacy_m2_pwm_disabled(old_pwm2)')
    source=source.replace('{l1,l2,r1,r2,legacy,old_pwm,old_cal}!==12\'d0',"{in1,in2,legacy,legacy2,old_pwm,old_pwm2,old_cal}!==24'd0")
    start=source.index('    defparam dut.imu.POWERUP_US=100;')
    end=source.index('    defparam dut.camera.stream.WIDTH=16;')
    source=source[:start]+'''    defparam dut.chassis.imu.reader.BOOT_US=100;
    defparam dut.chassis.imu.reader.RESET_US=1200;
    defparam dut.chassis.imu.reader.SETTLE_US=500;
    defparam dut.chassis.IMU_CAL_LOG2=2;
'''+source[end:]
    start=source.index('    mpu6050_model #')
    end=source.index('    reg [7:0] rx_data',start)
    source=source[:start]+'''    mpu6050_model sensor(.scl(scl),.sda(sda),.present(1'b1),.identity(8'h70),
        .freeze(1'b0),.corrupt(1'b0),.gx(16'sd0),.gy(16'sd0),.gz(16'sd100));
'''+source[end:]
    source=source.replace('dut.ethernet_raw.lidar_display.forwarded!=1000 || dut.ethernet_raw.lidar_display.dropped!=0',
        'dut.ethernet_scan.lidar_display.completed==0 || dut.ethernet_scan.lidar_display.input_loss_count!=0')
    source=source.replace('system_raw_top_tx.hex','system_mecanum_tx.hex')
    source=source.replace('seven UDP states','Mecanum UDP states')
    bench=folder/'tb_system_mecanum.sv';bench.write_text(source)
    files=[folder/'source'/p for p in meta['source_sha256'] if Path(p).name!='ethernet_pll.v']
    prim='C:/Gowin/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v'
    for name,cmd in [('compile',['iverilog','-g2012','-s','tb_system_mecanum','-s','GSR','-o',str(folder/'system.vvp'),
        str(bench),'tests/ethernet_pll_model.v','vendor/mecanum_team/tests/tb_mpu6050.sv',prim,*map(str,files)]),
        ('simulation',['vvp',str(folder/'system.vvp')])]:
        p=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=600)
        (folder/(name+'.log')).write_text(p.stdout+p.stderr,encoding='utf-8')
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
        print(p.stdout,end='',flush=True)
    rows,scans,video,frames=audit_packets(OUT/'system_mecanum_tx.hex')
    counts=collections.Counter(k for k,r in rows)
    assert {'mecanum_motor','mecanum_pose','lidar_scan','lidar_diagnostic'}<=set(counts)
    assert counts['motor']==counts['lidar_raw']==0 and video==1 and scans
    motors=[r for k,r in rows if k=='mecanum_motor']
    poses=[r for k,r in rows if k=='mecanum_pose']
    assert all(not r['armed'] and r['pwm']==(0,0,0,0) for r in motors)
    assert any(r['healthy'] and r['calibrated'] and r['who']==0x70 for r in poses)
    expected=[[i+0.0,0,0] for i in range(360)]
    for p in originals[:36]:
        start=struct.unpack_from('<H',p,4)[0]
        for i in range(12):
            angle=start+(i*999+5)//11
            distance,quality=struct.unpack_from('<HB',p,6+i*3)
            slot=angle//100
            if not expected[slot][1] or distance<expected[slot][1]:
                expected[slot]=[angle/100.,distance,quality]
    assert any(s['points']==expected for s in scans),'No complete scan matched independent nearest-return reference'
    diagnostic=[r for k,r in rows if k=='lidar_diagnostic'][-1]
    assert diagnostic['packets']==1000 and diagnostic['crc_errors']==1
    result=dict(passed=True,counts=dict(counts),complete_scans=len(scans),video_packets=video,
        motor_outputs_low=True,uart_idle=True,imu6500_calibration=True,
        nearest_reference_verified=True,programmed=False,hardware_tested=False,
        source_sha256=meta['source_sha256'])
    (folder/'system_test_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print('PASS full Mecanum top: four-wheel/pose UDP, MPU6500, 360-point scans, video, CRCs and unarmed outputs')


if __name__=='__main__':main()
