# Independent camera oscillator. Initial IO delay assumptions: validate on the board.
create_clock -name sys50 -period 20 [get_ports {clk50m}]
create_clock -name cam_pclk -period 11.9047 [get_ports {camera_pclk}]
create_generated_clock -name control50 -source [get_ports {clk50m}] -divide_by 1 [get_pins {camera/pll/PLL_inst/CLKOUT0}]
create_generated_clock -name eth125 -source [get_ports {clk50m}] -multiply_by 5 -divide_by 2 [get_pins {camera/pll/PLL_inst/CLKOUT1}]
set_clock_groups -asynchronous -group [get_clocks {cam_pclk}] -group [get_clocks {sys50 eth125}]
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
set_false_path -to [get_pins {imu/reader/bus/scl_sync_0_s0/D}]
set_false_path -to [get_pins {imu/reader/bus/sda_sync_0_s0/D}]
# No new output/reset false paths. Asynchronous I2C/UART external timing
# requires protocol simulation and hardware electrical validation.

# control50 <-> cam_pclk is intentionally NOT covered by set_clock_groups.
# Bound every crossing in these two domains (Gray pointers and fault handshake).
# This Gowin version includes clock latency; verify the routed exception paths,
# including Gray Q -> first-stage D delays, rather than assuming datapath_only.
set_max_delay -from [get_clocks {cam_pclk}] -to [get_clocks {control50}] 10.0
set_min_delay -from [get_clocks {cam_pclk}] -to [get_clocks {control50}] 0.0
set_max_delay -from [get_clocks {control50}] -to [get_clocks {cam_pclk}] 10.0
set_min_delay -from [get_clocks {control50}] -to [get_clocks {cam_pclk}] 0.0
# The FIRST synchronizer stage deliberately samples an unrelated clock. Its
# hold relationship is undefined. Keep max-delay/setup bounds above; do not
# exempt the second stage or synchronous paths within either domain.
set_false_path -hold -to [get_pins {camera/frames_to_control50/fifo/ws0*/D camera/frames_to_control50/fifo/rs0*/D camera/frames_to_control50/fault0*/D camera/frames_to_control50/ack0*/D}]
# Bundled-data mailboxes hold the entire bus until an acknowledgment returns.
# req has TWO destination synchronizer stages; data is captured on the next
# edge (at least 16ns at eth125, 40ns at control50 after launch). Bound only
# held -> destination data to 10ns, not the request or ordinary synchronous
# paths. No assumption of a 4ns single-cycle 50:125 clock phase is required.
set_max_delay -from [get_pins {camera/flow_mailbox/held*/Q}] -to [get_pins {camera/flow_mailbox/dst_data*/D}] 10.0
set_max_delay -from [get_pins {camera/observation_mailbox/held*/Q}] -to [get_pins {camera/observation_mailbox/dst_data*/D}] 10.0
set_max_delay -from [get_pins {camera/seed_mailbox/held*/Q}] -to [get_pins {camera/seed_mailbox/dst_data*/D}] 10.0
# Only reset synchronizer asynchronous assertion pins are excepted.
set_false_path -to [get_pins {camera/cap_reset/release_pipe*/CLEAR}]
set_false_path -to [get_pins {camera/front_reset/release_pipe*/CLEAR}]
set_false_path -to [get_pins {camera/video_reset/release_pipe*/CLEAR}]
# Archive all bounded CDC paths, not only the default worst synchronous paths.
report_exceptions -setup -from_clock [get_clocks {cam_pclk}] -to_clock [get_clocks {control50}] -max_paths 100
report_exceptions -setup -from_clock [get_clocks {control50}] -to_clock [get_clocks {cam_pclk}] -max_paths 100
report_exceptions -setup -from [get_pins {camera/flow_mailbox/held*/Q camera/observation_mailbox/held*/Q camera/seed_mailbox/held*/Q}] -max_paths 800
report_timing -setup -from_clock [get_clocks {control50}] -to_clock [get_clocks {control50}] -max_paths 25
report_timing -setup -from_clock [get_clocks {cam_pclk}] -to_clock [get_clocks {cam_pclk}] -max_paths 25
set_false_path -to [get_pins {chassis/rx/rx_meta_s0/D}]
set_false_path -to [get_pins {chassis/core/enc_l/ab_meta_0_s0/D}]
set_false_path -to [get_pins {chassis/core/enc_l/ab_meta_1_s0/D}]
set_false_path -to [get_pins {chassis/core/enc_r/ab_meta_0_s0/D}]
set_false_path -to [get_pins {chassis/core/enc_r/ab_meta_1_s0/D}]
