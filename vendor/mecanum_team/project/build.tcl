set here [file dirname [file normalize [info script]]]
cd $here
open_project mecanum.gprj
set_option -top_module mecanum_top
set_option -verilog_std sysv2017
set_option -output_base_name mecanum
set_option -max_fanout 32
set_option -replicate_resources 1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
run all
exit
