create_clock -name sys50 -period 20 [get_ports {clk50m}]
set_false_path -from [get_ports {reset_n}]
