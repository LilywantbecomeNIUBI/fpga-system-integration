set here [file dirname [file normalize [info script]]]
cd $here
open_project diag.gprj
set_option -top_module m2_sensor_diag_top
set_option -verilog_std sysv2017
set_option -output_base_name diag
set_option -max_fanout 32
set_option -replicate_resources 0
set_option -timing_driven 1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -place_option 0
set_option -route_option 2
run all
exit
