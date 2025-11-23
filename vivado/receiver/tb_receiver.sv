////Note: to find the <component_name> for the VIP instance, use the following Tcl command and find the output corresponding to the AXI VIP instance.
////The attached test bench assumes that the AXI component name is design_1_axi_vip_0_0 (the default for the first AXI VIP added to a BD)
 
////get_ips *vip*



`timescale 1ns / 1ps

import axi4stream_vip_pkg::*;
import design_1_axi4stream_vip_0_0_pkg::*;
import design_1_axi4stream_vip_1_0_pkg::*;

module axi4stream_vip_0_exdes_tb();

  design_1_axi4stream_vip_0_0_mst_t mst_agent;
  design_1_axi4stream_vip_1_0_slv_t slv_agent;

  bit clock = 0;
  bit reset = 0;

  // Declare variables for the task
  bit[15:0] test_data[];
  axi4stream_transaction wr_transaction;

  //instantiate DUT
  design_1_wrapper DUT(
    .clock(clock),
    .reset(reset)
  );

  //generate clock
  always #10 clock = ~clock;

  // Generate active-low reset for 100 clock cycles
  initial begin
    reset = 0;        // active low
    repeat (100) @(posedge clock);  // wait 100 clock cycles
    reset = 1;        // deactivate reset
  end


/****************************************************************************************************************
 Task send_a_packet sends a packet with data from an input array of 16-bit words.
 The final beat has Tlast set to 1.
 This task is used with AXI4STREAM VIP configured to have TLAST and TDATA WIDTH = 16 bits (2 bytes)
***************************************************************************************************************/
task automatic send_a_packet(
  input bit[15:0] data_array[],
  input int num_elements,
  input axi4stream_transaction wr_transaction
);
  bit[2*8-1:0]                            data_beat;
  bit[2-1:0]                              keep_beat;
  
  // Validate input
  if(num_elements <= 0) begin
    $display("Error: num_elements must be greater than 0");
    return;
  end
  
  if(num_elements > data_array.size()) begin
    $display("Error: num_elements (%0d) exceeds data_array size (%0d)", num_elements, data_array.size());
    return;
  end
  

  
  for(int i=0; i<num_elements; i++) begin
    // Get the 16-bit data directly from the array
    data_beat = data_array[i];
    
    // Create and configure transaction
    wr_transaction = mst_agent.driver.create_transaction("Master VIP write transaction");
    //wr_transaction.set_driver_return_item_policy(XIL_AXI4STREAM_AT_ACCEPT_RETURN);
    SEND_PACKET_FAILURE: assert(wr_transaction.randomize());
    wr_transaction.set_data_beat(data_beat);
    wr_transaction.set_last(0);
    
    // Set TLAST on final beat
    if(i == num_elements-1) begin
      wr_transaction.set_last(1);  
    end     
    
    // Send transaction
    mst_agent.driver.send(wr_transaction);
  end
endtask : send_a_packet


/****************************************************************************************************************
 Task generate_sine_wave generates a sine wave matching ap_fixed<16,4> format used in HLS.
 
 Parameters:
   amplitude    - Peak amplitude (recommend 0.0 to 7.999 for ap_fixed<16,4> range)
   frequency    - Normalized frequency (0.0 to 0.5, where 0.5 = Nyquist frequency)
   num_samples  - Number of samples to generate
   data_array   - Output array to populate with sine wave samples
   
 The sine wave is generated as: sample[n] = amplitude * sin(2*pi*frequency*n)
 Values are converted to ap_fixed<16,4> format (4 integer bits, 12 fractional bits)
 with scaling factor of 2^12 = 4096 for the fractional part.
***************************************************************************************************************/
task automatic generate_sine_wave(
  input real amplitude,
  input real frequency,
  input int num_samples,
  output bit[15:0] data_array[]
);
  real pi = 3.14159265358979323846;
  real sine_value;
  real scaled_value;
  int signed fixed_point_value;
  
  // Validate inputs
  if(num_samples <= 0) begin
    $display("Error: num_samples must be greater than 0");
    return;
  end
  
  if(amplitude < 0 || amplitude > 7.999) begin
    $display("Warning: amplitude should be between 0 and 7.999 for ap_fixed<16,4> format");
  end
  
  if(frequency < 0 || frequency > 0.5) begin
    $display("Warning: frequency should be between 0 and 0.5 to avoid aliasing");
  end
  
  // Allocate array
  data_array = new[num_samples];
  
  // Generate sine wave samples in ap_fixed<16,4> format
  // Format: 1 sign bit, 3 integer bits, 12 fractional bits
  // Scale factor: 2^12 = 4096
  for(int i = 0; i < num_samples; i++) begin
    sine_value = amplitude * $sin(2.0 * pi * frequency * i);
    scaled_value = sine_value * 4096.0;  // Scale by 2^12 for fractional part
    fixed_point_value = int'(scaled_value);
    data_array[i] = fixed_point_value[15:0];  // Store as 16-bit value
  end
  
  $display("Generated %0d sine wave samples: amplitude=%.3f, frequency=%.4f (ap_fixed<16,4> format)", 
           num_samples, amplitude, frequency);
endtask : generate_sine_wave

//send test data
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



//  // Test 1: Send simple packet
//  test_data = '{16'hDEAD, 16'hBEEF};
//  send_a_packet(test_data, 2, wr_transaction);
//  #300;

//  // Test 2: Send longer packet
//  test_data = '{16'h1111, 16'h2222, 16'h3333, 16'h4444};
//  send_a_packet(test_data, 4, wr_transaction);
//  #500;

//  // Test 3: Send only part of array
//  test_data = '{16'hAAAA, 16'hBBBB, 16'hCCCC, 16'hDDDD, 16'hEEEE};
//  send_a_packet(test_data, 3, wr_transaction);  // Only first 3
//  #300;

//  // Test 4: Send 1000 random values in range 0x1000 to 0xF000
//  test_data = new[1000];
//  for(int i = 0; i < 1000; i++) begin
//    test_data[i] = $urandom_range(16'hF000, 16'h1000);
//  end
//  send_a_packet(test_data, 1000, wr_transaction);
//  #50000;

   
  // Test 5: Send sine wave with configurable parameters
  // These parameters match the ap_fixed<16,4> format used in the C++ testbench
  // Amplitude range: 0.0 to 7.999 (with 4 integer bits)
  $display("\n=== Test 5: Sine Wave Generation (ap_fixed<16,4> format) ===");
  
  // Low frequency sine wave: amplitude=5.2, frequency=0.02 (200 samples per cycle), 10000 samples
  generate_sine_wave(5.2, 0.02, 10000, test_data);
  send_a_packet(test_data, 10000, wr_transaction);
  #25000;
  
//  // Medium frequency sine wave: amplitude=0.8, frequency=0.05 (20 samples per cycle), 10000 samples
//  generate_sine_wave(0.8, 0.05, 10000, test_data);
//  send_a_packet(test_data, 10000, wr_transaction);
//  #15000;
  
//  // Higher frequency sine wave: amplitude=0.5, frequency=0.1 (10 samples per cycle), 10000 samples
//  generate_sine_wave(0.5, 0.1, 10000, test_data);
//  send_a_packet(test_data, 10000, wr_transaction);
//  #10000;
  
  $display("Test complete");
  $finish;
end

endmodule



