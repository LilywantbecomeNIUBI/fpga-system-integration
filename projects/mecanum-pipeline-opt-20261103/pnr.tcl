set here [file dirname [file normalize [info script]]]
cd $here
open_project sensor.gprj
set_option -top_module system_mecanum_top
set_option -verilog_std sysv2017
set_option -output_base_name sensor
set_option -max_fanout 32
set_option -replicate_resources 1
set_option -timing_driven 1
set_option -retiming_resource bsram
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -place_option 2
set_option -route_option 1
if {![file exists {reference_baseline/impl/pnr/sensor.pr}]} {error "Run prepare_reference.py first; clean PnR fallback is forbidden"}
set_option -inc_place 0
set_option -inc_pnr {reference_baseline/impl/pnr/sensor.pr}
run pnr
exit
