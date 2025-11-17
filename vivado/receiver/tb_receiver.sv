////Note: to find the <component_name> for the VIP instance, use the following Tcl command and find the output corresponding to the AXI VIP instance.
////The attached test bench assumes that the AXI component name is design_1_axi_vip_0_0 (the default for the first AXI VIP added to a BD)
 
////get_ips *vip*

////design_1_axi4stream_vip_0_0 design_1_axi4stream_vip_1_0



////design_1_axi4stream_vip_0_0  //master

////design_1_axi4stream_vip_1_0  //slave



///***************************************************************************************************
//* Description: 
//* This testbench contains example test lists for one design which consists of one Master AXI4STREAM
//* VIP, one Slave AXI4STREAM VIP and one Passthrough AXI4STREAM VIP.
//* In the following scenarios,it demonstrates how Master AXI4STREAM VIP create transactions, 
//* Slave AXI4STREAM VIP generate ready(when TREADY is on) and how Passthrough AXI4STREAM VIP
//* switch into run time master/slave mode.
//* This testbench also has two simple scoreboards to do self-checking: 
//* One scoreboard checks master AXI4STREAM VIP against passthrough AXI4STREAM VIP
//* One scoreboard checks slave AXI4STREAM VIP  against passthrough AXI4STREAM VIP
//****************************************************************************************************
//* Description of How Master VIP works:
//* This file contains example on how Master VIP create a simple transaction 
//* For Master VIP to work correctly, user environment MUST have the lists of item below and 
//* follow this order.
//*    1. import two packages.
//*       import axi4stream_vip_pkg::* 
//*       import ex_sim_axi4stream_vip_mst_0_pkg::*;
//*    2. delcare "component_name"_mst_t agent
//*    3. new agent (passing instance IF correctly)
//*    4. set vif_proxy dummy drive type 
//*    5. start_master
//*    6. create_transaction
//*    7. Fill in transaction( two methods. randomization and API)
//*    8. send transaction
//* if user wants to create his own ready signal, please refer task user_gen_rready 
//****************************************************************************************************
//* Description of how Slave VIP works: 
//* This file contains example on how Slave VIP genearte ready signal when TREADY is on 
//* For Slave VIP to work correctly, user environment MUST have the lists of item below and
//* follow this order.
//*    1. import two packages.
//*       import axi4stream_vip_pkg::* 
//*       import ex_sim_axi4stream_vip_slv_0_pkg::*;
//*    2. delcare "component_name"_slv_t agent
//*    3. new agent (passing instance IF correctly)
//*    4. set vif_proxy dummy drive type 
//*    5. start_slave
//* As for ready generation, when TREADY is on, if user enviroment doesn't do anything, it will
//* randomly generate ready siganl if user wants to create his own ready signal,
//* please refer task slv_gen_tready 
//****************************************************************************************************

//`timescale 1ns / 1ps

///***************************************************************************************************
//* As described above, this design has all three VIPs. so it includes all three packages plus 
//* axi4stream_vip_pkg
//***************************************************************************************************/
//import axi4stream_vip_pkg::*;
//import design_1_axi4stream_vip_0_0_pkg::*;  //import master package
//import design_1_axi4stream_vip_1_0_pkg::*;  //import slave package

//module axi4stream_vip_0_exdes_tb(
//  );


//  /***************************************************************************************************
//  * Verbosity level which specifies how much debug information to be printed out
//  * 0         - No information will be printed out
//  * 400       - All information will be printed out
//  ***************************************************************************************************/
//  // Master VIP agent verbosity level
//  xil_axi4stream_uint                           mst_agent_verbosity = 0;
//  // Slave VIP agent verbosity level
//  xil_axi4stream_uint                           slv_agent_verbosity = 0;
//  // Passthrough VIP agent verbosity level
//  xil_axi4stream_uint                           passthrough_agent_verbosity = 0;
//  /***************************************************************************************************
//  * Parameterized agents which customer needs to declare according to AXI4STREAM VIP configuration
//  * If AXI4STREAM VIP is being configured in master mode, "component_name"_mst_t has to declared 
//  * If AXI4STREAM VIP is being configured in slave mode, "component_name"_slv_t has to be declared 
//  * If AXI4STREAM VIP is being configured in pass-through mode,"component_name"_passthrough_t has to be declared
//  * "component_name can be easily found in vivado bd design: click on the instance, 
//  * then click CONFIG under Properties window and Component_Name will be shown
//  * More details please refer PG277 for more details
//  ***************************************************************************************************/
//  design_1_axi4stream_vip_0_0_mst_t                              mst_agent;
//  design_1_axi4stream_vip_1_0_slv_t                              slv_agent;
     
//  // Clock signal
//  bit                                     clock;
//  // Reset signal
//  bit                                     reset;

//  // instantiate bd
//  design_1_wrapper DUT(
//  );

//  initial begin
//    reset <= 1'b1;
//  end
  
//  always #10 clock <= ~clock;

//  //Main process
//  initial begin
//    /*mst_monitor_transaction = new("master monitor transaction");
//    slv_monitor_transaction = new("slave monitor transaction");*/

//    /***************************************************************************************************
//    * The hierarchy path of the AXI4STREAM VIP's interface is passed to the agent when it is newed. 
//    * Method to find the hierarchy path of AXI4STREAM VIP is to run simulation without agents being newed,
//    * message like "Xilinx AXI4STREAM VIP Found at Path: 
//    * my_ip_exdes_tb.DUT.ex_design.axi4stream_vip_mst.inst" will be printed out.
//    ***************************************************************************************************/
//    mst_agent = new("master vip agent", DUT.design_1_i.axi4stream_vip_0.inst.IF);
//    slv_agent = new("slave vip agent", DUT.design_1_i.axi4stream_vip_1.inst.IF);
//    $timeformat (-12, 1, " ps", 1);

//    /***************************************************************************************************
//    * When bus is in idle, it must drive everything to 0.otherwise it will 
//    * trigger false assertion failure from axi_protocol_chekcer
//    ***************************************************************************************************/
    
//    mst_agent.vif_proxy.set_dummy_drive_type(XIL_AXI4STREAM_VIF_DRIVE_NONE);
//    slv_agent.vif_proxy.set_dummy_drive_type(XIL_AXI4STREAM_VIF_DRIVE_NONE);

//    /***************************************************************************************************
//    * Set tag for agents for easy debug,if not set here, it will be hard to tell which driver is filing 
//    * if multiple agents are called in one testbench
//    ***************************************************************************************************/
    
//    mst_agent.set_agent_tag("Master VIP");
//    slv_agent.set_agent_tag("Slave VIP");
//    // set print out verbosity level.
//    mst_agent.set_verbosity(mst_agent_verbosity);
//    slv_agent.set_verbosity(slv_agent_verbosity);

//    /***************************************************************************************************
//    * Master,slave agents start to run 
//    * Turn on passthrough agent monitor 
//    ***************************************************************************************************/
    
//    mst_agent.start_master();
//    slv_agent.start_slave();

 
//endmodule


`timescale 1ns / 1ps

import axi4stream_vip_pkg::*;
import design_1_axi4stream_vip_0_0_pkg::*;
import design_1_axi4stream_vip_1_0_pkg::*;

module axi4stream_vip_0_exdes_tb();

  design_1_axi4stream_vip_0_0_mst_t mst_agent;
  design_1_axi4stream_vip_1_0_slv_t slv_agent;

  axi4stream_transaction wr_transaction;
  axi4stream_monitor_transaction slv_mon_tr;

  bit clock = 0;
  bit reset = 1;

  design_1_wrapper DUT();

  always #10 clock = ~clock;

  initial begin
    reset = 1;
    #100 reset = 0;
  end

  initial begin
    wait(reset == 0);
    #200;

    mst_agent = new("master", DUT.design_1_i.axi4stream_vip_0.inst.IF);
    slv_agent = new("slave",  DUT.design_1_i.axi4stream_vip_1.inst.IF);

    mst_agent.set_verbosity(0);
    slv_agent.set_verbosity(0);

    mst_agent.start_master();
    slv_agent.start_slave();

    #200;

    // Monitor thread - cannot access protected members
    fork
      begin
        slv_agent.monitor.item_collected_port.get(slv_mon_tr);
        $display("==================================================");
        $display("Slave received a transaction at time %0t", $time);
        $display("==================================================");

        // Use the VIP wrapper's print method, if it exists
        // Uncomment if available:
        // slv_mon_tr.print_transaction();

        $display("Transaction received (data hidden due to protected members).");
        $display("==================================================");
      end
    join_none;

    // Send transactions
    wr_transaction = mst_agent.driver.create_transaction();
    wr_transaction.set_data_beat(32'hDEADBEEF);
    wr_transaction.set_keep_beat(4'hF);
    wr_transaction.set_last(1);
    mst_agent.driver.send(wr_transaction);
    #300;

    wr_transaction = mst_agent.driver.create_transaction();
    wr_transaction.set_data_beat(32'hCAFEBABE);
    wr_transaction.set_keep_beat(4'hF);
    wr_transaction.set_last(1);
    mst_agent.driver.send(wr_transaction);
    #300;

    wr_transaction = mst_agent.driver.create_transaction();
    wr_transaction.set_data_beat(32'h11111111);
    wr_transaction.set_keep_beat(4'hF);
    wr_transaction.set_data_beat(32'h22222222);
    wr_transaction.set_keep_beat(4'hF);
    wr_transaction.set_data_beat(32'h33333333);
    wr_transaction.set_keep_beat(4'hF);
    wr_transaction.set_last(1);
    mst_agent.driver.send(wr_transaction);
    #1000;

    $display("Test complete");
    $finish;
  end

endmodule



