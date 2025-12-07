
////Note: to find the <component_name> for the VIP instance, use the following Tcl command and find the output corresponding to the AXI VIP instance.
////The attached test bench assumes that the AXI component name is design_1_axi_vip_0_0 (the default for the first AXI VIP added to a BD)
 
////get_ips *vip*



`timescale 1ns / 1ps


module tb_receiver_with_zynq;


  parameter DMA_BASE = 32'hA000_0000;
  // ============================================================
  // AXI DMA Direct Register Mode Offsets (PG021)
  // ============================================================
 
  parameter int C_USE_M_AXI_HP0      = 'h01;
  parameter int C_USE_M_AXI_HP1      = 'h01;
  parameter int C_USE_S_AXI_HP0      = 'h01;
  parameter int C_USE_S_AXI_HP1      = 'h01;
  // -------------------------
  // MM2S (Memory → Stream)
  // -------------------------
  parameter int MM2S_DMACR      = 'h00;   // Control
  parameter int MM2S_DMASR      = 'h04;   // Status
  parameter int MM2S_SA         = 'h18;   // Source Address (LSB)
  parameter int MM2S_SA_MSB     = 'h1C;   // Source Address MSB (if enabled)
  parameter int MM2S_LENGTH     = 'h28;   // Transfer Length

  // -------------------------
  // S2MM (Stream → Memory)
  // -------------------------
  parameter int S2MM_DMACR      = 'h30;   // Control
  parameter int S2MM_DMASR      = 'h34;   // Status
  parameter int S2MM_DA         = 'h48;   // Destination Address (LSB)
  parameter int S2MM_DA_MSB     = 'h4C;   // Destination Address MSB
  parameter int S2MM_LENGTH     = 'h58;   // Transfer Length

  // -------------------------
  // Optional: reserved offsets (for completeness)
  // -------------------------
  parameter int MM2S_RSVD0      = 'h08;
  parameter int MM2S_RSVD1      = 'h0C;
  parameter int MM2S_RSVD2      = 'h10;
  parameter int MM2S_RSVD3      = 'h14;
  parameter int MM2S_RSVD4      = 'h20;

  parameter int S2MM_RSVD0      = 'h38;
  parameter int S2MM_RSVD1      = 'h3C;
  parameter int S2MM_RSVD2      = 'h40;
  parameter int S2MM_RSVD3      = 'h44;
  parameter int S2MM_RSVD4      = 'h50;


  parameter RAM_BUFER1 = 32'h0000_0000;
  parameter RAM_BUFER2 = 32'h0000_9000;
  parameter RAM_BUFER_SIZE = 32'h0000_0200;


  logic [31:0] my_data = 32'hDEADBEEF;


  bit clock = 0;
  bit reset = 0;
  int num_samples = 100;
  int response = 100;
  logic [31:0] read_data;

  // Declare variables for the TX task
  bit[15:0] test_data[];

  // Declare variables for the RX task  
  bit[15:0] received_data[];
  int num_received;


  //instantiate DUT
  mpsoc_preset_wrapper DUT(
  );

  //generate clock
  always #10 clock = ~clock;




  // ============================================================
  // AXI DMA Direct Register Mode Tasks
  // Using parameterized register offsets
  // ============================================================

// =========================================================
// TASK: DMA Status Read - Read and display both channel status
// =========================================================
task automatic dma_status_read(
    input logic [31:0] base
);
    logic [31:0] s2mm_status;
    logic [31:0] mm2s_status;
    begin
        dma_reg_read(base + S2MM_DMASR, s2mm_status);
        $display("S2MM status reg: 0x%08h", s2mm_status);
        
        dma_reg_read(base + MM2S_DMASR, mm2s_status);
        $display("MM2S status reg: 0x%08h", mm2s_status);
    end
endtask

// =========================================================
// TASK: Wait for MM2S Channel Idle
// =========================================================
task automatic dma_mm2s_idle(
    input logic [31:0] base
);
    logic [31:0] mm2s_status;
    begin
        do begin
            dma_reg_read(base + MM2S_DMASR, mm2s_status);
        end while (!(mm2s_status & 32'h0000_0002)); // Wait for Idle bit (bit 1)
        
        $display("[%0t] MM2S channel is IDLE", $time);
    end
endtask

// =========================================================
// TASK: Wait for S2MM Channel Idle
// =========================================================
task automatic dma_s2mm_idle(
    input logic [31:0] base
);
    logic [31:0] s2mm_status;
    begin
        do begin
            dma_reg_read(base + S2MM_DMASR, s2mm_status);
        end while (!(s2mm_status & 32'h0000_0002)); // Wait for Idle bit (bit 1)
        
        $display("[%0t] S2MM channel is IDLE", $time);
    end
endtask



// =========================================================
// TASK: AXI DMA Register Write
// =========================================================
task automatic dma_reg_write(
    input  logic [31:0] addr,
    input  logic [31:0] data
);
    begin
        reg resp;
        tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.write_data(addr,4,data,resp);
        $display("[%0t] DMA WRITE  addr=0x%08h  data=0x%08h  resp=%0d",$time, addr, data, resp);
    end
endtask

// =========================================================
// TASK: AXI DMA Register Read
// =========================================================
task automatic dma_reg_read(
    input  logic [31:0] addr,
    output logic [31:0] data
);
    begin
        reg resp;
        tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.read_data(addr,4,data,resp);
        $display("[%0t] DMA READ   addr=0x%08h  data=0x%08h  resp=%0d", $time, addr, data, resp);
    end
endtask

  
  //  Programming Sequence (Direct Register Mode)
  
  
  //  S2MM (Stream → Memory)
  //  1. Write S2MM_DMACR = 0x1        (RS=1)
  //  2. Write S2MM_DA = dest_addr
  //  3. Write S2MM_LENGTH = N_bytes   (waits for AXIS data)
  //  4. Poll S2MM_DMASR.Idle == 1 
  
  
  
  //  MM2S (Memory → Stream)    
  
  //  1. Write MM2S_DMACR = 0x1        (RS=1)
  //  2. Write MM2S_SA = source_addr
  //  3. Write MM2S_LENGTH = N_bytes   (starts transfer)
  //  4. Poll MM2S_DMASR.Idle == 1


// -------------------------------------------------------------
// MM2S Transfer (Memory → Stream)
// -------------------------------------------------------------
task automatic dma_mm2s_transfer(
    input logic [31:0] base,
    input logic [31:0] src_addr,
    input logic [31:0] length
);
    logic [31:0] status;

    // RESET channel
    dma_reg_write(base + MM2S_DMACR, 32'h0000_0004); // Reset=1
    do begin
        dma_reg_read(base + MM2S_DMACR, status);
    end while(status[2] == 1); // wait reset done

    // ENABLE channel
    dma_reg_write(base + MM2S_DMACR, 32'h0000_0001); // RS=1

    // ADDRESS
    dma_reg_write(base + MM2S_SA, src_addr);

    // START
    dma_reg_write(base + MM2S_LENGTH, length);

    // POLL IDLE
    do begin
        dma_reg_read(base + MM2S_DMASR, status);
    end while (status[1] == 0);
endtask



// -------------------------------------------------------------
// S2MM Transfer (Stream → Memory)
// -------------------------------------------------------------
task automatic dma_s2mm_transfer(
    input logic [31:0] base,
    input logic [31:0] dst_addr,
    input logic [31:0] length
);
    logic [31:0] status;

    // RESET channel
    dma_reg_write(base + S2MM_DMACR, 32'h0000_0004); // Reset=1
    do begin
        dma_reg_read(base + S2MM_DMACR, status);
    end while(status[2] == 1);

    // ENABLE channel
    dma_reg_write(base + S2MM_DMACR, 32'h0000_0001); // RS=1

    // ADDRESS
    dma_reg_write(base + S2MM_DA, dst_addr);

    // START
    dma_reg_write(base + S2MM_LENGTH, length);

    // POLL IDLE
//    do begin
//        dma_reg_read(base + S2MM_DMASR, status);
//    end while (status[1] == 0);
endtask




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


task automatic zynq_vip_init(ref bit clock);

  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b1);
  repeat(20) @(posedge clock);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b0);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.fpga_soft_reset(32'h1);
  repeat(20) @(posedge clock);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b1);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.fpga_soft_reset(32'h0);
  repeat(200) @(posedge clock);
  //tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.set_stop_on_error(1'b1); 
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.set_debug_level_info(1'b1); 
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.set_slave_profile("S_AXI_HP0",2'b01); ; 
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.set_slave_profile("S_AXI_HP1",2'b01); ; 
  repeat(200) @(posedge clock);
 
  
endtask : zynq_vip_init


/****************************************************************************************************************
 Task generate_am_signal generates an AM modulated signal matching the C++ testbench.
 
 The AM signal is: (1 + modulation_index * message) * carrier * normalization
 where:
   - message = cos(2*pi*message_freq*t)
   - carrier = cos(2*pi*carrier_freq*t)
   - normalization = 1 / (1 + modulation_index)
 
 Parameters:
   sample_rate      - Sample rate in Hz (e.g., 480000.0)
   carrier_freq     - Carrier frequency in Hz (e.g., 100000.0)
   message_freq     - Message/modulating frequency in Hz (e.g., 1000.0)
   modulation_index - Modulation depth 0.0 to 1.0 (e.g., 0.8)
   num_samples      - Number of samples to generate
   data_array       - Output array populated with AM signal samples
   
 Values are converted to ap_fixed<16,4> format (4 integer bits, 12 fractional bits)
 with scaling factor of 2^12 = 4096 for the fractional part.
***************************************************************************************************************/
task automatic generate_am_signal(
  input real sample_rate,
  input real carrier_freq,
  input real message_freq,
  input real modulation_index,
  input int num_samples,
  output bit[15:0] data_array[]
);
  real pi = 3.14159265358979323846;
  real t;
  real message;
  real carrier;
  real am_signal;
  real max_env;
  real norm;
  real scaled_value;
  int signed fixed_point_value;
  
  // Validate inputs
  if(num_samples <= 0) begin
    $display("Error: num_samples must be greater than 0");
    return;
  end
  
  if(modulation_index < 0 || modulation_index > 1.0) begin
    $display("Warning: modulation_index should be between 0 and 1.0");
  end
  
  // Calculate normalization factor (matches C++ testbench)
  max_env = 1.0 + modulation_index;
  norm = 1.0 / max_env;
  
  // Allocate array
  data_array = new[num_samples];
  
  // Generate AM signal samples
  for(int i = 0; i < num_samples; i++) begin
    // Calculate time
    t = i / sample_rate;
    
    // Generate message signal (cosine wave)
    message = $cos(2.0 * pi * message_freq * t);
    
    // Generate carrier signal (cosine wave)
    carrier = $cos(2.0 * pi * carrier_freq * t);
    
    // AM modulation: (1 + m*message) * carrier * normalization
    am_signal = (1.0 + modulation_index * message) * carrier * norm;
    
    // Convert to ap_fixed<16,4> format
    scaled_value = am_signal * 4096.0;  // Scale by 2^12 for fractional part
    fixed_point_value = int'(scaled_value);
    data_array[i] = fixed_point_value[15:0];
  end
  
  $display("Generated %0d AM signal samples:", num_samples);
  $display("  Sample Rate: %.1f Hz", sample_rate);
  $display("  Carrier Frequency: %.1f Hz", carrier_freq);
  $display("  Message Frequency: %.1f Hz", message_freq);
  $display("  Modulation Index: %.2f", modulation_index);
  $display("  Format: ap_fixed<16,4>");
  
endtask : generate_am_signal

////send test data
initial begin
  // Initialize signals
  reset = 0;
  
  //Reset MPSoC and PL  
  zynq_vip_init(clock);


  
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.pre_load_mem(1,32'h0000_0000, 1000000);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.read_mem(32'h0000_0000, 4, read_data);
  $display("DDR read = %08h", read_data);

  // Write specific data to DDR
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.write_mem(
    my_data,            // data to write
    32'h0000_0000,      // start address  
    4                   // number of bytes
  );

  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.read_mem(32'h0000_0000, 4, read_data);
  $display("DDR read = %08h", read_data);

  for (int i = 0; i < 100; i++) begin
  
    // Reset channels
    dma_reg_write(DMA_BASE + S2MM_DMACR, 32'h0000_0004);
    dma_reg_write(DMA_BASE + MM2S_DMACR, 32'h0000_0004);
    
    // Halt channels
    dma_reg_write(DMA_BASE + S2MM_DMACR, 32'h0000_0000);
    dma_reg_write(DMA_BASE + MM2S_DMACR, 32'h0000_0000);
    
    // Set addresses
    dma_reg_write(DMA_BASE + MM2S_SA, RAM_BUFER1);
    dma_reg_write(DMA_BASE + S2MM_DA, RAM_BUFER2);
    
    // Enable with interrupts masked (0xf001 = IRQThreshold=0xF, RS=1)
    dma_reg_write(DMA_BASE + S2MM_DMACR, 32'h0000_F001);
    dma_reg_write(DMA_BASE + MM2S_DMACR, 32'h0000_F001);
    
    // Start transfers
    dma_reg_write(DMA_BASE + S2MM_LENGTH, RAM_BUFER_SIZE);
    dma_reg_write(DMA_BASE + MM2S_LENGTH, RAM_BUFER_SIZE);
    
    $display("Started DMA transfer");
    dma_status_read(DMA_BASE);
    
    // Wait for completion
    dma_mm2s_idle(DMA_BASE);
    dma_s2mm_idle(DMA_BASE);
    
    $display("DMA transfer completed");
    dma_status_read(DMA_BASE);  
  
  
//    dma_s2mm_transfer(DMA_BASE,RAM_BUFER2,RAM_BUFER_SIZE);
//    dma_mm2s_transfer(DMA_BASE,RAM_BUFER1,RAM_BUFER_SIZE);  
    // Wait for some time
    repeat(400) @(posedge clock);
  end
  
  $display("\n=== Test: AM Modulated Signal Generation ===");
  
  // Generate and send data
  generate_am_signal(
    .sample_rate(480000.0),
    .carrier_freq(100000.0),
    .message_freq(1000.0),
    .modulation_index(0.6),
    .num_samples(num_samples),
    .data_array(test_data)
  );
  
  
  #25000;
  $display("Test complete");
  $finish;
end

endmodule
