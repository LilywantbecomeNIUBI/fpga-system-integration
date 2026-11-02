create_clock -name sys50 -period 20 [get_ports {clk50m}]
set_false_path -from [get_ports {reset_n}]
set_false_path -to [get_pins {imu/reader/bus/scl_sync_0_s0/D}]
set_false_path -to [get_pins {imu/reader/bus/sda_sync_0_s0/D}]
