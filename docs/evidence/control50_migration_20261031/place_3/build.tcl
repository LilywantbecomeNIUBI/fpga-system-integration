set here [file dirname [file normalize [info script]]]
cd $here
open_project system.gprj
set_option -top_module system_top
set_option -verilog_std sysv2017
set_option -output_base_name system
set_option -max_fanout 32
set_option -replicate_resources 0
set_option -timing_driven 1
set_option -retiming_resource bsram
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -place_option 3
set_option -route_option 1
run syn
file copy -force {E:/Gowin_project/fpga_project/team_repos/fpga-system-integration/build/control50_20261031T011613Z/frozen.vg} {E:/Gowin_project/fpga_project/team_repos/fpga-system-integration/build/control50_20261031T011613Z/place_3/impl/gwsynthesis/system.vg}
run pnr
exit
