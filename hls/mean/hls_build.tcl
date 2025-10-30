# This script sets up and compiles an HLS project.
#
# vitis_hls hls_build.tcl
# vitis_hls -p hls_proj/hls.app

open_project -reset mean
set_top mean
add_files mean.hpp
add_files mean.cpp
add_files -tb mean_tb.cpp
open_solution -reset "solution1"
set_part {xc7a100t-csg324-1}
create_clock -period 10 -name default
config_export -format ip_catalog -rtl verilog
csynth_design
csim_design -clean
cosim_design
export_design -rtl verilog -format ip_catalog -description "mean." -vendor "caccolillo" -display_name "mean" -output "../mean.zip"
exit
