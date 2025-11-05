# Set the reference directory for source file relative paths (by default the value is script directory path)
set origin_dir "."


# Set the project name
set _xil_proj_name_ "project_1"

# Set the directory path for the original project from where this script was exported
set orig_proj_dir "[file normalize "$origin_dir/project_1"]"



# Create project
create_project ${_xil_proj_name_} ./${_xil_proj_name_} -part xc7a100tcsg324-1

# Set the directory path for the new project
set proj_dir [get_property directory [current_project]]

# Set project properties
set obj [current_project]
set_property -name "board_part" -value "digilentinc.com:arty-a7-100:part0:1.1" -objects $obj
set_property -name "default_lib" -value "xil_defaultlib" -objects $obj
set_property -name "enable_resource_estimation" -value "0" -objects $obj
set_property -name "enable_vhdl_2008" -value "1" -objects $obj
set_property -name "ip_cache_permissions" -value "read write" -objects $obj
set_property -name "ip_output_repo" -value "$proj_dir/${_xil_proj_name_}.cache/ip" -objects $obj
set_property -name "mem.enable_memory_map_generation" -value "1" -objects $obj
set_property -name "platform.board_id" -value "arty-a7-100" -objects $obj
set_property -name "revised_directory_structure" -value "1" -objects $obj
set_property -name "sim.central_dir" -value "$proj_dir/${_xil_proj_name_}.ip_user_files" -objects $obj
set_property -name "sim.ip.auto_export_scripts" -value "1" -objects $obj
set_property -name "simulator_language" -value "Mixed" -objects $obj
set_property -name "sim_compile_state" -value "1" -objects $obj
set_property -name "target_language" -value "VHDL" -objects $obj
set_property -name "webtalk.activehdl_export_sim" -value "52" -objects $obj
set_property -name "webtalk.modelsim_export_sim" -value "52" -objects $obj
set_property -name "webtalk.questa_export_sim" -value "52" -objects $obj
set_property -name "webtalk.riviera_export_sim" -value "52" -objects $obj
set_property -name "webtalk.vcs_export_sim" -value "52" -objects $obj
set_property -name "webtalk.xcelium_export_sim" -value "2" -objects $obj
set_property -name "webtalk.xsim_export_sim" -value "52" -objects $obj
set_property -name "webtalk.xsim_launch_sim" -value "142" -objects $obj



# Set IP repository paths
set obj [get_filesets sources_1]
if { $obj != {} } {
   set_property "ip_repo_paths" "[file normalize "$origin_dir/../../"]" $obj

   # Rebuild user ip_repo's index before adding any source files
   update_ip_catalog -rebuild
}


#add simulation sources
set_property SOURCE_SET sources_1 [get_filesets sim_1]
add_files -fileset sim_1 -norecurse ./tb_receiver.sv

#add design sources
#add_files -norecurse {./clocked_comparator_25bit.vhd ./mux2to1_32bit.vhd ./sinewave_generator.vhd}
#update_compile_order -fileset sources_1

#build block design
source ./bd.tcl

#create hdl wrapper
make_wrapper -files [get_files ./project_1/project_1.srcs/sources_1/bd/design_1/design_1.bd] -top
add_files -norecurse ./project_1/project_1.gen/sources_1/bd/design_1/hdl/design_1_wrapper.vhd
update_compile_order -fileset sources_1

#add waveform configuration file
add_files -fileset sim_1 -norecurse ./tb_design_1_wrapper_behav.wcfg
set_property xsim.view ./tb_design_1_wrapper_behav.wcfg [get_filesets sim_1]

#package project as an IP-core
update_compile_order -fileset sources_1
ipx::package_project -root_dir ./ -vendor user.org -library user -taxonomy /UserIP
ipx::merge_project_changes hdl_parameters [ipx::current_core]
set_property core_revision 2 [ipx::current_core]
ipx::create_xgui_files [ipx::current_core]
ipx::update_checksums [ipx::current_core]
ipx::check_integrity [ipx::current_core]
ipx::save_core [ipx::current_core]
exit

