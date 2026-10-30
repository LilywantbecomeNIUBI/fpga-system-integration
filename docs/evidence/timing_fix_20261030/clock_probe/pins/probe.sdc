create_clock -name sys50 -period 20 [get_ports {clk50m}]
set_false_path -from [get_ports {reset_n}]
create_generated_clock -name control50 -source [get_ports {clk50m}] -divide_by 1 [get_pins {pll/PLL_inst/CLKOUT0}]
create_generated_clock -name eth125 -source [get_ports {clk50m}] -multiply_by 5 -divide_by 2 [get_pins {pll/PLL_inst/CLKOUT1}]
