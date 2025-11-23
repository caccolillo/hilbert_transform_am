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
  int num_samples = 1000;

  // Declare variables for the TX task
  bit[15:0] test_data[];
  axi4stream_transaction wr_transaction;

  // Declare variables for the RX task  
  bit[15:0] received_data[];
  int num_received;
  axi4stream_transaction rd_transaction;


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



/****************************************************************************************************************
 Task receive_packet_blocking reads data from the slave VIP monitor and stores it in an output array.
 This task blocks until data arrives - use when you're confident the DUT will produce output.
 
 Parameters:
   max_elements    - Maximum number of elements to read (prevents runaway on missing TLAST)
   data_array      - Output array populated with received data (sized to actual received count)
   elements_read   - Output count of how many elements were actually received
   rd_transaction  - Pre-declared transaction object for receiving monitor data
   
 The task reads until TLAST is detected or max_elements is reached.
 Each transaction is read from the slave VIP's monitor using a blocking get() call.
 
 Usage Example:
   bit[15:0] rx_data[];
   int count;
   axi4stream_transaction rd_transaction;
   receive_packet_blocking(10000, rx_data, count, rd_transaction);
   $display("Received %0d samples", count);
***************************************************************************************************************/
task automatic receive_packet_blocking(
  input int max_elements,
  output bit[15:0] data_array[],
  output int elements_read,
  input axi4stream_transaction rd_transaction
);
  bit[15:0] temp_data[];
  bit packet_complete = 0;
  
  // Validate input
  if(max_elements <= 0) begin
    $display("Error: max_elements must be greater than 0");
    elements_read = 0;
    return;
  end
  
  // Allocate temporary array
  temp_data = new[max_elements];
  elements_read = 0;
  
  $display("Waiting to receive packet (max %0d elements)...", max_elements);
  
  // Read data from slave VIP monitor
  while(!packet_complete && elements_read < max_elements) begin
    // Blocking get - waits for transaction
    slv_agent.monitor.item_collected_port.get(rd_transaction);
    
    // Store data
    temp_data[elements_read] = rd_transaction.get_data_beat();
    elements_read++;
    
    // Check for TLAST
    if(rd_transaction.get_last() == 1) begin
      packet_complete = 1;
      $display("Received complete packet: %0d elements (TLAST detected)", elements_read);
    end
  end
  
  // Check if we hit max without TLAST
  if(!packet_complete && elements_read >= max_elements) begin
    $display("Warning: Reached max_elements (%0d) without detecting TLAST", max_elements);
  end
  
  // Copy to sized output array
  data_array = new[elements_read];
  for(int i = 0; i < elements_read; i++) begin
    data_array[i] = temp_data[i];
  end
  
endtask : receive_packet_blocking


/****************************************************************************************************************
 Task save_data_to_file writes received data array to a text file.
 
 Parameters:
   filename        - Name of the output file (e.g., "logfile.txt")
   data_array      - Array of 16-bit data to write
   num_elements    - Number of elements in the array to write
   format_type     - Output format: 0=simple hex, 1=detailed, 2=CSV
   
 Format types:
   0 - Simple: One hex value per line (0x1234)
   1 - Detailed: Index, Hex, Signed Decimal, Real value with header
   2 - CSV: Comma-separated values for import into Excel/MATLAB
   
 Usage Example:
   save_data_to_file("logfile.txt", received_data, num_received, 1);
***************************************************************************************************************/
task automatic save_data_to_file(
  input string filename,
  input bit[15:0] data_array[],
  input int num_elements,
  input int format_type = 1
);
  integer file_handle;
  real fixed_point_value;
  
  // Validate inputs
  if(num_elements <= 0) begin
    $display("Error: num_elements must be greater than 0");
    return;
  end
  
  if(num_elements > data_array.size()) begin
    $display("Error: num_elements (%0d) exceeds data_array size (%0d)", num_elements, data_array.size());
    return;
  end
  
  // Open file for writing
  file_handle = $fopen(filename, "w");
  
  if (file_handle == 0) begin
    $display("Error: Could not open %s for writing", filename);
    return;
  end
  
  $display("Writing %0d samples to %s (format type %0d)...", num_elements, filename, format_type);
  
  // Write data based on format type
  case(format_type)
    0: begin  // Simple hex format
      for(int i = 0; i < num_elements; i++) begin
        $fwrite(file_handle, "%04h\n", data_array[i]);
      end
    end
    
    1: begin  // Detailed format with header
      $fwrite(file_handle, "# Received AXI Stream Data\n");
      $fwrite(file_handle, "# Total samples: %0d\n", num_elements);
      $fwrite(file_handle, "# Format: Index, Hex, Signed Decimal, ap_fixed<16,4> Real Value\n");
      $fwrite(file_handle, "#\n");
      
      for(int i = 0; i < num_elements; i++) begin
        fixed_point_value = $signed(data_array[i]) / 4096.0;  // Convert from ap_fixed<16,4>
        $fwrite(file_handle, "%0d, 0x%04h, %0d, %.6f\n", 
                i, data_array[i], $signed(data_array[i]), fixed_point_value);
      end
    end
    
    2: begin  // CSV format
      $fwrite(file_handle, "Index,Hex,Decimal,Real\n");
      
      for(int i = 0; i < num_elements; i++) begin
        fixed_point_value = $signed(data_array[i]) / 4096.0;
        $fwrite(file_handle, "%0d,0x%04h,%0d,%.6f\n", 
                i, data_array[i], $signed(data_array[i]), fixed_point_value);
      end
    end
    
    default: begin
      $display("Error: Invalid format_type (%0d). Valid options: 0, 1, 2", format_type);
      $fclose(file_handle);
      return;
    end
  endcase
  
  $fclose(file_handle);
  $display("Successfully wrote %0d samples to %s", num_elements, filename);
  
endtask : save_data_to_file


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
  generate_sine_wave(5.2, 0.02, num_samples, test_data);
  // Send sinewave over AXI stream master channle
  send_a_packet(test_data, num_samples, wr_transaction);
  // Receive demodulated data from AXI stream slave channel
  receive_packet_blocking(num_samples, received_data, num_received, rd_transaction);
  // Simple hex format data saving
  save_data_to_file("logfile.txt", received_data, num_received, 0);
  #25000;
  

  
  $display("Test complete");
  $finish;
end

endmodule



