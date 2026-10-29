set here [file dirname [file normalize [info script]]]
cd $here
open_project system.gprj
set_option -top_module system_top
set_option -verilog_std v2001
set_option -output_base_name system
set_option -max_fanout 16
set_option -replicate_resources 1
set_option -retiming_resource bsram
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -place_option 3
set_option -route_option 2
run all
