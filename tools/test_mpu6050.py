"""Replacement IMU bus regression. No hardware access."""
from pathlib import Path
import hashlib,json
import sys
from test_system import run, ROOT

def main():
    (ROOT/'build/tests').mkdir(parents=True,exist_ok=True)
    run('mpu6050_compile',['iverilog','-g2012','-Wall','-s','tb_mpu6050_reader','-o','build/tests/mpu6050.vvp',
        'tests/imu/tb_mpu6050_reader.sv','tests/imu/mpu6050_model.sv','fpga/imu/imu_1khz_core.v',
        'fpga/imu/mpu6050_reader.v','fpga/imu/imu_fast_i2c_master.v'])
    results={}
    results['bus']=run('mpu6050',['vvp','build/tests/mpu6050.vvp'])
    run('mpu6500_compile',['iverilog','-g2012','-s','tb_mpu6500_reader','-o','build/tests/mpu6500.vvp',
        'tests/imu/tb_mpu6500_reader.sv','tests/imu/mpu6050_model.sv','fpga/imu/imu_1khz_core.v',
        'fpga/imu/mpu6050_reader.v','fpga/imu/imu_fast_i2c_master.v',
        'fpga/system/imu_status_source.sv','fpga/motor/frame_transport.sv'])
    results['mpu6500_bus']=run('mpu6500',['vvp','build/tests/mpu6500.vvp'])
    results['host']=run('mpu6050_host',[sys.executable,'-m','unittest','discover','-s','tests','-p','test_imu*.py','-v'])
    run('mpu6050_bringup_compile',['iverilog','-g2012','-Wall','-s','tb_mpu6050_bringup','-o','build/tests/mpu6050_bringup.vvp',
        'tests/imu/tb_mpu6050_bringup.sv','tests/imu/mpu6050_model.sv','fpga/imu/mpu6050_bringup_top.sv',
        'fpga/imu/imu_1khz_core.v','fpga/imu/mpu6050_reader.v','fpga/imu/imu_fast_i2c_master.v',
        'fpga/imu/imu_fast_uart_stream.v','fpga/motor/uart_byte.sv'])
    results['bringup']=run('mpu6050_bringup',['vvp','build/tests/mpu6050_bringup.vvp'])
    sys.path.insert(0,str(ROOT))
    from tools.verify_mpu6050 import evaluate
    results['bringup_uart']=evaluate((ROOT/'build/tests/mpu6050_bringup_uart.bin').read_bytes(),require_motor_telemetry=False)
    assert results['bringup_uart']['passed'],results['bringup_uart']
    paths=['fpga/imu/mpu6050_reader.v','fpga/imu/imu_1khz_core.v','fpga/imu/imu_fast_i2c_master.v',
        'tests/imu/mpu6050_model.sv','tests/imu/tb_mpu6050_reader.sv','tests/test_imu_mpu6050.py',
        'shared/imu_protocol.py','shared/imu_state.py','tools/verify_mpu6050.py',
        'fpga/imu/mpu6050_bringup_top.sv','fpga/imu/imu_fast_uart_stream.v','tests/imu/tb_mpu6050_bringup.sv',
        'fpga/system/imu_status_source.sv','tests/imu/tb_mpu6500_reader.sv','tests/test_imu_mpu6500.py']
    record=dict(passed=True,cases=results,hardware_tested=False,
        source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths})
    (ROOT/'build/tests/mpu6050_result.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':main()
