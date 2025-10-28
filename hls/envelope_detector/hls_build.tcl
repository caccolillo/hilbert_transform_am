# This script sets up and compiles an HLS project.
#
# vitis_hls hls_build.tcl
# vitis_hls -p hls_proj/hls.app

open_project -reset envelope_detector
set_top envelope_detector
add_files envelope_detector.hpp
add_files envelope_detector.cpp
add_files -tb envelope_detector_tb.cpp
open_solution -reset "solution1"
set_part {xc7a100t-csg324-1}
create_clock -period 10 -name default
config_export -format ip_catalog -rtl verilog
csynth_design
csim_design -clean
cosim_design
export_design -rtl verilog -format ip_catalog -description "Envelope detector." -vendor "caccolillo" -display_name "ENVELOPE_DETECTOR" -output "../env_det.zip"
exit
