create_clock -name sys50 -period 20 [get_ports {clk50m}]
# Asynchronous pins feed only their input synchronizers. I2C is timed by the
# 50 MHz clock-enable FSM; neither SCL nor SDA is an internal clock.
set_false_path -from [get_ports {reset_n imu_i2c_scl imu_i2c_sda}]
# UART/I2C/status outputs have no board-synchronous receiving clock. Their
# protocol timing is verified separately; bench rise times remain unmeasured.
set_false_path -to [get_ports {uart_tx imu_i2c_scl imu_i2c_sda led[*]}]
