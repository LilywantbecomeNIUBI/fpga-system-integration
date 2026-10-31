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
