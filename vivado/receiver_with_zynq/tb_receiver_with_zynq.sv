
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
  parameter RAM_BUFER_SIZE1 = 32'h0000_0096;  // MM2S: 150 bytes 
  parameter RAM_BUFER_SIZE2 = 32'h0000_000a;  // S2MM: 10 bytes 

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
  mpsoc_preset_wrapper DUT();

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


  // MM2S DMA programming task
  //  Programming Sequence (Direct Register Mode)
  //  MM2S (Memory → Stream)    
  //  1. Write MM2S_DMACR = 0x1        (RS=1)
  //  2. Write MM2S_SA = source_addr
  //  3. Write MM2S_LENGTH = N_bytes   (starts transfer)
  //  4. Poll MM2S_DMASR.Idle == 1    
  task automatic start_mm2s_dma(
    input logic [31:0] buffer_addr,
    input logic [31:0] buffer_size
  );
    begin
      // Reset MM2S channel
      dma_reg_write(DMA_BASE + MM2S_DMACR, 32'h0000_0004);

      // Halt channel
      dma_reg_write(DMA_BASE + MM2S_DMACR, 32'h0000_0000);

      // Set address (you used MM2S_SA - keeping exactly as provided)
      dma_reg_write(DMA_BASE + MM2S_SA, buffer_addr);

      // Enable with interrupts masked (IRQThreshold=0xF, RS=1)
      dma_reg_write(DMA_BASE + MM2S_DMACR, 32'h0000_F001);

      // Start transfers
      dma_reg_write(DMA_BASE + MM2S_LENGTH, buffer_size);
    end
  endtask


  // S2MM DMA programming task
  //  Programming Sequence (Direct Register Mode)
  //  S2MM (Stream → Memory)
  //  1. Write S2MM_DMACR = 0x1        (RS=1)
  //  2. Write S2MM_DA = dest_addr
  //  3. Write S2MM_LENGTH = N_bytes   (waits for AXIS data)
  //  4. Poll S2MM_DMASR.Idle == 1   
  task automatic start_s2mm_dma(
    input logic [31:0] buffer_addr,
    input logic [31:0] buffer_size
  );
    begin
      // Reset S2MM channel
      dma_reg_write(DMA_BASE + S2MM_DMACR, 32'h0000_0004);

      // Halt S2MM channel
      dma_reg_write(DMA_BASE + S2MM_DMACR, 32'h0000_0000);
    
      // Set destination address (S2MM_DA is odd! but using your original)
      dma_reg_write(DMA_BASE + S2MM_DA, buffer_addr);

      // Enable with interrupts masked (IRQThreshold=0xF, RS=1)
      dma_reg_write(DMA_BASE + S2MM_DMACR, 32'h0000_F001);

      // Start transfers
      dma_reg_write(DMA_BASE + S2MM_LENGTH, buffer_size);
    end
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

//Initialise PL and PL
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


// -------------------------------------------------------------
//  Task: write_samples_to_ddr
//  Packs 16-bit samples into 32-bit words and writes to Zynq MPSoC VIP DDR
//  Zynq VIP requires 32-bit aligned addresses
//
//  Arguments:
//    data_array[] : bit[15:0] samples (dynamic array)
//    base_addr    : starting DDR address (must be 32-bit aligned)
// -------------------------------------------------------------
task automatic write_samples_to_ddr(
  input bit [15:0] data_array[],
  input longint unsigned base_addr
);
  int num_samples = data_array.size();
  logic [31:0] word_data;
  longint unsigned addr;
  int num_words;

  if (num_samples == 0) begin
    $display("ERROR: data_array is empty (DDR write skipped)");
    return;
  end

  // Calculate number of 32-bit words needed (round up)
  num_words = (num_samples + 1) / 2;

  // Write samples as 32-bit words (2 samples per word)
  for (int i = 0; i < num_words; i++) begin
    addr = base_addr + (i * 4);  // Each word is 4 bytes
    
    // Pack two 16-bit samples into one 32-bit word
    if ((i * 2 + 1) < num_samples) begin
      // Both samples available
      word_data = {data_array[i*2 + 1], data_array[i*2]};
    end else begin
      // Only one sample left (odd number of samples)
      word_data = {16'h0000, data_array[i*2]};
    end
    
    tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.write_mem(
      word_data,
      addr,
      4  // 4 bytes per word
    );
  end

  $display("DDR WRITE: %0d samples (%0d bytes) written as %0d words starting at 0x%08x",
           num_samples, num_samples * 2, num_words, base_addr);
endtask : write_samples_to_ddr

//send test data
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
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.read_mem(32'h0000_0000, 4, read_data); //read back data from DDR
  $display("DDR read = %08h", read_data);

  $display("\n=== Test: AM Modulated Signal Generation ===");

  // Generate AM data
  generate_am_signal(
    .sample_rate(480000.0),
    .carrier_freq(100000.0),
    .message_freq(1000.0),
    .modulation_index(0.8),
    .num_samples(num_samples),
    .data_array(test_data)
  );
  
  //write it into DDR memory
  write_samples_to_ddr(test_data, RAM_BUFER1);

  //DMAs loop
  for (int i = 0; i < 100; i++) begin
    dma_status_read(DMA_BASE);   

    $display("Started DMA transfer");
    dma_status_read(DMA_BASE);   

    //MM2S transfer start
    start_mm2s_dma(RAM_BUFER1, RAM_BUFER_SIZE1);
    //polling on MM2S transfer end
    dma_mm2s_idle(DMA_BASE);    


    // Wait for some time
    repeat(400) @(posedge clock);

    //S2MM transfer start
    start_s2mm_dma(RAM_BUFER2, RAM_BUFER_SIZE2);
    //polling on S2MM transfer end
    dma_s2mm_idle(DMA_BASE);
  
    $display("DMA transfer completed");
    dma_status_read(DMA_BASE);  
  
    // Wait for some time
    repeat(40) @(posedge clock);
  end

  // ============================================================
  // Save DDR memory contents to files
  // ============================================================
  $display("\n=== Saving Memory Contents to Files ===");
  
  // Save RAM_BUFER1 (TX buffer - AM signal input)
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.peek_mem_to_file(
    "buffer1_tx_data.txt",     // Output filename
    RAM_BUFER1,                 // Start address (0x0000_0000)
    RAM_BUFER_SIZE1             // Size in bytes (150 bytes)
  );
  $display("Saved RAM_BUFER1 to buffer1_tx_data.txt (Address: 0x%08x, Size: %0d bytes)", 
           RAM_BUFER1, RAM_BUFER_SIZE1);
  
  // Save RAM_BUFER2 (RX buffer - received/processed data)
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.peek_mem_to_file(
    "buffer2_rx_data.txt",     // Output filename
    RAM_BUFER2,                 // Start address (0x0000_9000)
    RAM_BUFER_SIZE2             // Size in bytes (10 bytes)
  );
  $display("Saved RAM_BUFER2 to buffer2_rx_data.txt (Address: 0x%08x, Size: %0d bytes)", 
           RAM_BUFER2, RAM_BUFER_SIZE2);
  
  $display("Memory dump complete\n");
  // ============================================================
  
  
  
  
  
  #25000;
  $display("Test complete");
  $finish;
end

endmodule
