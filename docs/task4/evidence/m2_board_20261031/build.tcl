set here [file dirname [file normalize [info script]]]
cd $here
open_project task4.gprj
set_option -top_module system_task4_top
set_option -verilog_std sysv2017
set_option -output_base_name task4
set_option -max_fanout 32
set_option -replicate_resources 0
set_option -timing_driven 1
set_option -retiming_resource bsram
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -place_option 3
set_option -route_option 0
run syn
file copy -force {E:/Gowin_project/fpga_project/team_repos/fpga-system-integration/build/task4/system_/20261031T040352Z/impl/gwsynthesis/task4.vg} {E:/Gowin_project/fpga_project/team_repos/fpga-system-integration/build/task4/system_/20261031T042625Z_p3r0/impl/gwsynthesis/task4.vg}
run pnr
exit
