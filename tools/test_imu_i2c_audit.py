"""Compare both I2C engines with delayed pad signals, independently of hardware."""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    out = ROOT / 'build/imu_i2c_audit_20261102'
    out.mkdir(parents=True, exist_ok=True)
    model = (ROOT / 'tests/imu/mpu6050_model.sv').read_text()
    model = model.replace('module mpu6050_model', 'module mpu_i2c_delayed_model')
    model = model.replace('input reset_stuck', 'input reset_stuck, input [1:0] nack_at')
    model = model.replace('assign sda=drive_low', 'assign #(3000,3000) sda=drive_low')
    model = model.replace("ack_enable=present && value[7:1]==7'h68;",
                          "ack_enable=present && value[7:1]==7'h68 && !(nack_at==2 && value[0]);")
    model = model.replace('RX_REG: begin pointer=value;',
                          'RX_REG: begin ack_enable=present && nack_at!=1; pointer=value;')
    delayed = out / 'delayed_slave.sv'
    delayed.write_text(model)
    sources = ['tests/imu/tb_i2c_controller_compare.sv', 'fpga/imu/imu_whoami_i2c.v',
               'fpga/imu/imu_fast_i2c_master.v']
    commands = [['iverilog', '-g2012', '-s', 'tb_i2c_controller_compare', '-o',
                 str(out / 'compare.vvp'), *sources, str(delayed)],
                ['vvp', str(out / 'compare.vvp')]]
    for name, command in zip(('compile', 'regression'), commands):
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=90)
        (out / (name + '.log')).write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        if name == 'regression':
            print(result.stdout, end='')
    record = dict(passed=True, hardware_tested=False, scl_rise_delay_ns=700,
                  slave_data_delay_ns=3000,
                  source_sha256={p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
                                 for p in sources + ['tests/imu/mpu6050_model.sv']})
    (out / 'result.json').write_text(json.dumps(record, indent=2) + '\n')


if __name__ == '__main__':
    main()
