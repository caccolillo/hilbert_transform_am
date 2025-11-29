////Note: to find the <component_name> for the VIP instance, use the following Tcl command and find the output corresponding to the AXI VIP instance.
////The attached test bench assumes that the AXI component name is design_1_axi_vip_0_0 (the default for the first AXI VIP added to a BD)
 
////get_ips *vip*



`timescale 1ns / 1ps


module tb_receiver_with_zynq;
  parameter DMA_BASE = 32'hA000_0000;
  // ============================================================
  // AXI DMA Direct Register Mode Offsets (PG021)
  // ============================================================

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


  parameter RAM_BUFER1 = DMA_BASE + 32'h0000_0000; // start of buffer 1 in DDR
  parameter RAM_BUFER2 = DMA_BASE + 32'h0000_1000; // start of buffer 2 in DDR
  parameter RAM_BUFER_SIZE = 32'h0000_0100;        // 256 bytes

  bit clock = 0;
  bit reset = 0;
  int num_samples = 10000;

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
// TASK: AXI DMA 128-bit Register Write (HP0)
// ============================================================
task automatic dma_reg_write_128(
    input logic [31:0] addr,
    input logic [127:0] data,
    output logic resp
);
begin
    tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.write_data(addr,16,data,resp);
    $display("[%0t] DMA WRITE128  addr=0x%08h  data=0x%032h  resp=%0d", 
             $time, addr, data, resp);
end
endtask

// ============================================================
// TASK: AXI DMA 128-bit Register Read (HP0)
// ============================================================
task automatic dma_reg_read_128(
    input  logic [31:0] addr,
    output logic [127:0] data,
    output logic resp
);
begin
    tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.read_data(addr,16,data,resp);
    $display("[%0t] DMA READ128   addr=0x%08h  data=0x%032h  resp=%0d", 
             $time, addr, data, resp);
end
endtask



// =========================================================
// TASK: AXI DMA Register Write (128-bit for HP0)
// =========================================================
task automatic dma_reg_write(
    input  logic [31:0] addr,
    input  logic [31:0] data
);
    logic [127:0] data128;
    reg resp;

    // Place data in lowest 32 bits of 128-bit word
    data128 = {96'h0, data};

    // Use 16-byte access (128-bit) for AXI HP0
    tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.write_data(addr, 16, data128, resp);

    $display("[%0t] DMA WRITE  addr=0x%08h  data=0x%08h  resp=%0d",$time, addr, data, resp);
endtask

// =========================================================
// TASK: AXI DMA Register Read (128-bit for HP0)
// =========================================================
task automatic dma_reg_read(
    input  logic [31:0] addr,
    output logic [31:0] data
);
    logic [127:0] data128;
    reg resp;

    // Use 16-byte access (128-bit) for AXI HP0
    tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.read_data(addr, 16, data128, resp);

    // Extract lowest 32 bits
    data = data128[31:0];

    $display("[%0t] DMA READ   addr=0x%08h  data=0x%08h  resp=%0d",$time, addr, data, resp);
endtask




task automatic init_buffer_zero(
    input logic [31:0] base_addr,
    input int          buffer_size
);
    logic [127:0] zero_data;
    logic resp;
    int num_words;
    int i;
begin
    num_words = (buffer_size + 15) / 16;
    zero_data = 128'h0;

    $display("[%0t] Initializing buffer at 0x%08h, %0d bytes (%0d words)...", 
             $time, base_addr, buffer_size, num_words);

    for(i = 0; i < num_words; i++) begin
        dma_reg_write_128(base_addr + i*16, zero_data, resp);
        if(resp !== 0) $display("Warning: write failed at 0x%08h", base_addr + i*16);
    end

    $display("[%0t] Buffer initialization complete.", $time);
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
    
    // 1. Program source address
    dma_reg_write(base + MM2S_SA, src_addr);

    // 2. Enable DMA channel (Run/Stop = 1)
    dma_reg_write(base + MM2S_DMACR, 32'h0000_0001);

    // 3. Writing length starts the transfer
    dma_reg_write(base + MM2S_LENGTH, length);

    // 4. Wait for Idle = 1
    do begin
        dma_reg_read(base + MM2S_DMASR, status);
    end while (status[1] == 1'b0);  // bit 1 = Idle
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
    reg resp;
    
    // 1. Program destination address
    dma_reg_write(base + S2MM_DA, dst_addr);

    // 2. Start DMA channel (Run/Stop = 1)
    dma_reg_write(base + S2MM_DMACR, 32'h0000_0001);

    // 3. Writing LENGTH starts receiver
    dma_reg_write(base + S2MM_LENGTH, length);

    // 4. Wait for Idle = 1
    do begin
        dma_reg_read(base + S2MM_DMASR, status);
    end while (status[1] == 1'b0);  // bit 1 = Idle
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
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b1);
  repeat(20) @(posedge clock);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b0);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.fpga_soft_reset(32'h1);
  repeat(20) @(posedge clock);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b1);
  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.fpga_soft_reset(32'h0);
  #2000 ;  
  
  //Initialise DDR memory buffers
  init_buffer_zero(RAM_BUFER1, RAM_BUFER_SIZE);
  init_buffer_zero(RAM_BUFER2, RAM_BUFER_SIZE);

  
  

// ---------------------------------------------------------------------
// Parallel MM2S and S2MM transfers using fork...join
// Addresses must be 16-byte aligned for AXI HP0
// ---------------------------------------------------------------------
fork
    begin
        $display("[%0t] Starting MM2S transfer...", $time);
        dma_mm2s_transfer(DMA_BASE, RAM_BUFER1, RAM_BUFER_SIZE);
        $display("[%0t] MM2S transfer completed.", $time);
    end

    begin
        $display("[%0t] Starting S2MM transfer...", $time);
        dma_s2mm_transfer(DMA_BASE, RAM_BUFER2, RAM_BUFER_SIZE);
        $display("[%0t] S2MM transfer completed.", $time);
    end
join





  // Wait for some time
  repeat(10) @(posedge clock);
  
  // Release reset
  reset = 1;
  $display("Reset released at time %0t", $time);
  
  // Wait for reset to propagate
  repeat(20) @(posedge clock);
  
  // Let agents initialize
  repeat(10) @(posedge clock);
  
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



