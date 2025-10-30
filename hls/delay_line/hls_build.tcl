# This script sets up and compiles an HLS project.
#
# vitis_hls hls_build.tcl
# vitis_hls -p hls_proj/hls.app

open_project -reset delay_line
set_top delay_line
add_files delay_line.hpp
add_files delay_line.cpp
add_files -tb delay_line_tb.cpp
open_solution -reset "solution1"
set_part {xc7a100t-csg324-1}
create_clock -period 10 -name default
config_export -format ip_catalog -rtl verilog
csynth_design
csim_design -clean
cosim_design
export_design -rtl verilog -format ip_catalog -description "delay_line." -vendor "caccolillo" -display_name "delay_line" -output "../delay_line.zip"
exit
