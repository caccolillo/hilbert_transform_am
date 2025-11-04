# Reset and open project
open_project -reset am_demodulator

# Add source files
add_files am_demodulator.hpp
add_files am_demodulator.cpp

# Add testbench
add_files -tb am_demodulator_tb.cpp
add_files -tb plot_csv.py

# Set top-level function
set_top am_demodulator

# Open solution and configure
open_solution -reset "solution1"
set_part {xc7a100t-csg324-1}
create_clock -period 10 -name default

# Export RTL and synthesize
config_export -format ip_catalog
csynth_design
csim_design -clean
export_design -rtl verilog -format ip_catalog -description "am_demodulator" -vendor "caccolillo" -display_name "am_demodulator" -output "../am_demodulator.zip"
cosim_design
# Exit HLS
exit

