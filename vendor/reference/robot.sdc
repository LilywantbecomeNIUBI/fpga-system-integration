# Independent camera oscillator. Initial IO delay assumptions: validate on the board.
create_clock -name sys50 -period 20 [get_ports {clk50m}]
create_clock -name cam_pclk -period 11.9047 [get_ports {camera_pclk}]
create_generated_clock -name control50 -source [get_ports {clk50m}] -divide_by 1 [get_nets {camera/clk50}]
create_generated_clock -name eth125 -source [get_ports {clk50m}] -multiply_by 5 -divide_by 2 [get_nets {camera/clk125}]
set_clock_groups -asynchronous -group [get_clocks {cam_pclk}] -group [get_clocks {sys50 control50 eth125}]
set_input_delay -clock cam_pclk -clock_fall -max 2.0 [get_ports {camera_data[*] camera_href camera_vsync}]
set_input_delay -clock cam_pclk -clock_fall -min 0.0 [get_ports {camera_data[*] camera_href camera_vsync}]
set_false_path -from [get_ports {reset_n}]
# These three flops intentionally receive an asynchronous reset; only release_pipe[2]
# releases the receiving domain. Do not exempt the downstream data paths.
set_false_path -to [get_pins {camera/perception_reset/release_pipe*/CLEAR}]

# Status bits cross clock domains through two flops; exempt only first-stage D.

# Divided PCLK enters the frequency meter through two flops.

# RGMII RX clock is independent of FPGA TX PLL; mailbox CDC is handshake-protected.
create_clock -name eth_rx125 -period 8 [get_ports {rgmii_rxc}]
set_clock_groups -asynchronous -group [get_clocks {eth_rx125}] -group [get_clocks {sys50 control50 eth125 cam_pclk}]

# RTL8211F R43/R44 pull-ups select internal RX/TX delay. Initial RX next-data transition assumption 2 ns +/-0.2 ns.
# These IO margins still require an oscilloscope/PVT audit; do not treat a board packet test as an eye diagram.
set_input_delay -clock eth_rx125 -max 2.2 [get_ports {rgmii_rxd[*] rgmii_rxctl}]
set_input_delay -clock eth_rx125 -min 1.8 [get_ports {rgmii_rxd[*] rgmii_rxctl}]
set_input_delay -clock eth_rx125 -clock_fall -add_delay -max 2.2 [get_ports {rgmii_rxd[*] rgmii_rxctl}]
set_input_delay -clock eth_rx125 -clock_fall -add_delay -min 1.8 [get_ports {rgmii_rxd[*] rgmii_rxctl}]

# Only the asynchronous CLEAR inputs of reset-release synchronizers are exempted.
# Their final outputs release the network domains synchronously; all downstream data paths remain timed.
set_false_path -to [get_pins {camera/runtime_network.network_reset/release_pipe*/CLEAR}]
set_false_path -to [get_pins {camera/runtime_network.receive_reset/release_pipe*/CLEAR}]

# UART diagnostic words use two synchronization stages; these samples are for observation only.
# New IMU input CDC: first stage only; all downstream logic remains timed.
set_false_path -to [get_pins {imu/core/reader/bus/scl_sync_0_s0/D}]
set_false_path -to [get_pins {imu/core/reader/bus/sda_sync_0_s0/D}]
# No new output/reset false paths. Asynchronous I2C/UART external timing
# requires protocol simulation and hardware electrical validation.
