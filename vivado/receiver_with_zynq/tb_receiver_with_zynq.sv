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
  parameter RAM_BUFER_SIZE = 32'h0000_0040;        // 64 bytes
  parameter DDR_MEM = 2'b01;
  
  bit clock = 0;
  bit reset = 0;
  int num_samples = 10000;
  integer succ;
  
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

// ============================================================
// TASK: Initialize buffer to zero
// ============================================================
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

// ============================================================
// TASK: Write test data to DDR memory
// ============================================================
task automatic write_test_data_to_ddr(
    input logic [31:0] base_addr,
    input bit[15:0] data_array[],
    input int num_samples
);
    logic [127:0] write_data;
    logic resp;
    int num_writes;
    int sample_idx;
    
    // Each 128-bit write contains 8 samples (16 bits each)
    num_writes = (num_samples + 7) / 8;
    
    $display("[%0t] Writing %0d samples (%0d 128-bit words) to DDR at 0x%08h", 
             $time, num_samples, num_writes, base_addr);
    
    sample_idx = 0;
    for(int i = 0; i < num_writes; i++) begin
        // Pack 8 samples into 128-bit word
        write_data = 128'h0;
        for(int j = 0; j < 8; j++) begin
            if(sample_idx < num_samples) begin
                write_data[j*16 +: 16] = data_array[sample_idx];
                sample_idx++;
            end
        end
        
        dma_reg_write_128(base_addr + i*16, write_data, resp);
        if(resp !== 0) begin
            $display("ERROR: Failed to write data at address 0x%08h", base_addr + i*16);
        end
    end
    
    $display("[%0t] Test data write complete. Wrote %0d samples.", $time, sample_idx);
endtask

// ============================================================
// TASK: Read data back from DDR memory
// ============================================================
task automatic read_data_from_ddr(
    input logic [31:0] base_addr,
    output bit[15:0] data_array[],
    input int num_samples
);
    logic [127:0] read_data;
    logic resp;
    int num_reads;
    int sample_idx;
    
    // Each 128-bit read contains 8 samples (16 bits each)
    num_reads = (num_samples + 7) / 8;
    
    // Allocate output array
    data_array = new[num_samples];
    
    $display("[%0t] Reading %0d samples (%0d 128-bit words) from DDR at 0x%08h", 
             $time, num_samples, num_reads, base_addr);
    
    sample_idx = 0;
    for(int i = 0; i < num_reads; i++) begin
        dma_reg_read_128(base_addr + i*16, read_data, resp);
        
        if(resp !== 0) begin
            $display("ERROR: Failed to read data at address 0x%08h", base_addr + i*16);
        end
        
        // Unpack 8 samples from 128-bit word
        for(int j = 0; j < 8; j++) begin
            if(sample_idx < num_samples) begin
                data_array[sample_idx] = read_data[j*16 +: 16];
                sample_idx++;
            end
        end
    end
    
    $display("[%0t] Data read complete. Read %0d samples.", $time, sample_idx);
endtask

// -------------------------------------------------------------
// MM2S Start (non-blocking)
// -------------------------------------------------------------
task automatic dma_mm2s_start(
    input logic [31:0] base,
    input logic [31:0] src_addr,
    input logic [31:0] length
);
    $display("\n[%0t] ========== MM2S Transfer Configuration ==========", $time);
    $display("  Source Address: 0x%08h", src_addr);
    $display("  Length: %0d bytes", length);
    
    // 1. Program source address
    dma_reg_write(base + MM2S_SA, src_addr);

    // 2. Enable DMA channel (Run/Stop = 1)
    dma_reg_write(base + MM2S_DMACR, 32'h0000_0001);

    // 3. Writing length starts the transfer
    dma_reg_write(base + MM2S_LENGTH, length);

    $display("[%0t] MM2S transfer initiated (non-blocking)", $time);
    $display("========================================================\n");
endtask

// -------------------------------------------------------------
// MM2S Wait for Completion
// -------------------------------------------------------------
task automatic dma_mm2s_wait_complete(
    input logic [31:0] base
);
    logic [31:0] status;
    int timeout_counter = 0;
    int poll_interval = 0;
    
    $display("[%0t] Waiting for MM2S completion...", $time);

    do begin
        dma_reg_read(base + MM2S_DMASR, status);
        
        // Print status every 1000 polls to avoid log spam
        if(poll_interval % 1000 == 0) begin
            $display("[%0t] MM2S_DMASR = 0x%08h [Poll #%0d]", $time, status, timeout_counter);
            $display("  Halted=%b, Idle=%b, DMAIntErr=%b, DMASlvErr=%b, DMADecErr=%b, IOC_Irq=%b",
                     status[0], status[1], status[4], status[5], status[6], status[12]);
        end
        
        // Check for errors
        if(status[4] || status[5] || status[6]) begin
            $display("\n*** ERROR: DMA Error detected in MM2S! ***");
            $display("  DMAIntErr=%b, DMASlvErr=%b, DMADecErr=%b", status[4], status[5], status[6]);
            $display("  DMASR=0x%08h", status);
            $finish;
        end
        
        // Add timeout to prevent infinite loop
        timeout_counter++;
        poll_interval++;
        if(timeout_counter > 100000) begin
            $display("\n*** ERROR: MM2S transfer timeout after %0d polls! ***", timeout_counter);
            $display("  Final DMASR=0x%08h", status);
            $display("  This likely means:");
            $display("    1. HP0 memory interface not connected properly");
            $display("    2. Stream interface stalled - check TREADY on AXI Stream");
            $display("    3. am_demodulator not accepting data");
            $display("    4. Clock domain crossing issue");
            $finish;
        end
        
        #100; // Small delay between polls
    end while (status[1] == 1'b0);  // bit 1 = Idle
    
    $display("[%0t] *** MM2S transfer COMPLETED ***", $time);
    $display("  Final status=0x%08h (Idle=%b, IOC_Irq=%b)", status, status[1], status[12]);
endtask

// -------------------------------------------------------------
// S2MM Start (non-blocking) - MUST BE CALLED BEFORE MM2S!
// -------------------------------------------------------------
task automatic dma_s2mm_start(
    input logic [31:0] base,
    input logic [31:0] dst_addr,
    input logic [31:0] length
);
    logic [31:0] status;
    
    $display("\n[%0t] ========== S2MM Transfer Configuration ==========", $time);
    $display("  Destination Address: 0x%08h", dst_addr);
    $display("  Length: %0d bytes", length);
    
    // 1. Program destination address
    dma_reg_write(base + S2MM_DA, dst_addr);

    // 2. Start DMA channel (Run/Stop = 1)
    dma_reg_write(base + S2MM_DMACR, 32'h0000_0001);

    // 3. Writing LENGTH starts receiver (makes it ready to accept data)
    dma_reg_write(base + S2MM_LENGTH, length);
    
    // 4. Verify S2MM is now running (not halted)
    dma_reg_read(base + S2MM_DMASR, status);
    $display("[%0t] S2MM Status after start: 0x%08h", $time, status);
    
    if(status[0] == 1'b1) begin
        $display("  WARNING: S2MM is HALTED! Status bit 0 = 1");
        $display("  This means S2MM did not start properly!");
    end else begin
        $display("  S2MM is RUNNING (Halted bit = 0)");
    end
    
    if(status[1] == 1'b0) begin
        $display("  S2MM is ACTIVE (Idle bit = 0) - ready to receive data");
    end else begin
        $display("  WARNING: S2MM shows IDLE immediately - unexpected!");
    end

    $display("[%0t] S2MM receiver armed and waiting for stream data", $time);
    $display("========================================================\n");
endtask

// -------------------------------------------------------------
// S2MM Wait for Completion
// -------------------------------------------------------------
task automatic dma_s2mm_wait_complete(
    input logic [31:0] base
);
    logic [31:0] status;
    int timeout_counter = 0;
    int poll_interval = 0;
    
    $display("[%0t] Waiting for S2MM completion...", $time);

    do begin
        dma_reg_read(base + S2MM_DMASR, status);
        
        // Print status every 1000 polls to avoid log spam
        if(poll_interval % 1000 == 0) begin
            $display("[%0t] S2MM_DMASR = 0x%08h [Poll #%0d]", $time, status, timeout_counter);
            $display("  Halted=%b, Idle=%b, DMAIntErr=%b, DMASlvErr=%b, DMADecErr=%b, IOC_Irq=%b",
                     status[0], status[1], status[4], status[5], status[6], status[12]);
        end
        
        // Check for errors
        if(status[4] || status[5] || status[6]) begin
            $display("\n*** ERROR: DMA Error detected in S2MM! ***");
            $display("  DMAIntErr=%b, DMASlvErr=%b, DMADecErr=%b", status[4], status[5], status[6]);
            $display("  DMASR=0x%08h", status);
            $finish;
        end
        
        // Check if S2MM is halted (should never happen during transfer)
        if(status[0] == 1'b1 && status[1] == 1'b0) begin
            $display("\n*** ERROR: S2MM became HALTED during transfer! ***");
            $display("  This usually means:");
            $display("    1. Data arrived before S2MM was configured");
            $display("    2. Internal DMA error occurred");
            $display("  DMASR=0x%08h", status);
            $finish;
        end
        
        // Add timeout to prevent infinite loop
        timeout_counter++;
        poll_interval++;
        if(timeout_counter > 100000) begin
            $display("\n*** ERROR: S2MM transfer timeout after %0d polls! ***", timeout_counter);
            $display("  Final DMASR=0x%08h", status);
            $display("  This likely means:");
            $display("    1. No stream data arriving (check TVALID)");
            $display("    2. TLAST never asserted (S2MM waits for TLAST)");
            $display("    3. am_demodulator not producing output");
            $display("    4. Stream data path broken");
            $display("    5. Clock domain issue");
            $finish;
        end
        
        #100; // Small delay between polls
    end while (status[1] == 1'b0);  // bit 1 = Idle
    
    $display("[%0t] *** S2MM transfer COMPLETED ***", $time);
    $display("  Final status=0x%08h (Idle=%b, IOC_Irq=%b)", status, status[1], status[12]);
endtask

/****************************************************************************************************************
 Task generate_sine_wave generates a sine wave matching ap_fixed<16,4> format used in HLS.
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
  for(int i = 0; i < num_samples; i++) begin
    sine_value = amplitude * $sin(2.0 * pi * frequency * i);
    scaled_value = sine_value * 4096.0;  // Scale by 2^12 for fractional part
    fixed_point_value = int'(scaled_value);
    data_array[i] = fixed_point_value[15:0];
  end
  
  $display("Generated %0d sine wave samples: amplitude=%.3f, frequency=%.4f (ap_fixed<16,4> format)", 
           num_samples, amplitude, frequency);
endtask : generate_sine_wave

/****************************************************************************************************************
 Task save_data_to_file writes received data array to a text file.
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
        fixed_point_value = $signed(data_array[i]) / 4096.0;
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
 Task generate_am_signal generates an AM modulated signal.
***************************************************************************************************************/
//task automatic generate_am_signal(
//  input real sample_rate,
//  input real carrier_freq,
//  input real message_freq,
//  input real modulation_index,
//  input int num_samples,
//  output bit[15:0] data_array[]
//);
//  real pi = 3.14159265358979323846;
//  real t;
//  real message;
//  real carrier;
//  real am_signal;
//  real max_env;
//  real norm;
//  real scaled_value;
//  int signed fixed_point_value;
  
//  // Validate inputs
//  if(num_samples <= 0) begin
//    $display("Error: num_samples must be greater than 0");
//    return;
//  end
  
//  if(modulation_index < 0 || modulation_index > 1.0) begin
//    $display("Warning: modulation_index should be between 0 and 1.0");
//  end
  
//  // Calculate normalization factor
//  max_env = 1.0 + modulation_index;
//  norm = 1.0 / max_env;
  
//  // Allocate array
//  data_array = new[num_samples];
  
//  // Generate AM signal samples
//  for(int i = 0; i < num_samples; i++) begin
//    // Calculate time
//    t = i / sample_rate;
    
//    // Generate message signal (cosine wave)
//    message = $cos(2.0 * pi * message_freq * t);
    
//    // Generate carrier signal (cosine wave)
//    carrier = $cos(2.0 * pi * carrier_freq * t);
    
//    // AM modulation
//    am_signal = (1.0 + modulation_index * message) * carrier * norm;
    
//    // Convert to ap_fixed<16,4> format
//    scaled_value = am_signal * 4096.0;
//    fixed_point_value = int'(scaled_value);
//    data_array[i] = fixed_point_value[15:0];
//  end
  
//  $display("Generated %0d AM signal samples:", num_samples);
//  $display("  Sample Rate: %.1f Hz", sample_rate);
//  $display("  Carrier Frequency: %.1f Hz", carrier_freq);
//  $display("  Message Frequency: %.1f Hz", message_freq);
//  $display("  Modulation Index: %.2f", modulation_index);
//  $display("  Format: ap_fixed<16,4>");
  
//endtask : generate_am_signal

task automatic generate_am_signal(
    input real carrier_freq,
    input real mod_freq,
    input real modulation_index,
    input int num_samples,
    output bit[15:0] data_array[]
);
    real pi = 3.141592653589793;
    real t;
    real carrier, mod_sig, am_sig;
    int signed fp;
    data_array = new[num_samples];

    for(int i=0; i<num_samples; i++) begin
        t = i;

        // baseband modulation signal (0..1)
        mod_sig = 0.5 * (1.0 + $sin(2*pi*mod_freq*t));

        // carrier
        carrier = $sin(2*pi*carrier_freq*t);

        // AM
        am_sig = (1.0 + modulation_index*mod_sig) * carrier;

        // convert to ap_fixed<16,4>
        fp = int'(am_sig * 4096.0);
        data_array[i] = fp[15:0];
    end

    $display("Generated AM signal: %0d samples", num_samples);
endtask

task automatic write_mem_am_data(
    input logic [31:0] start_addr,
    input bit[15:0] am_data[],
    input int num_samples
);
    int i;
    int succ;
    bit [127:0] burst;
    int num_words;

    num_words = (num_samples + 7)/8;

    $display("[%0t] Writing AM samples via write_mem()...", $time);

    for(i=0; i<num_words; i++) begin
        burst = 128'h0;

        // pack 8 × 16-bit samples
        for(int j=0; j<8; j++) begin
            int idx = i*8 + j;
            if (idx < num_samples)
                burst[j*16 +: 16] = am_data[idx];
        end

        // VIP backdoor write
        tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.write_mem(
            burst,
            start_addr + i*16,
            16
        );

        if (succ != 0)
            $display("ERROR: write_mem failed at address %08h", start_addr + i*16);
    end

    $display("[%0t] AM DDR initialisation completed (%0d samples).", 
             $time, num_samples);
endtask


// ============================================================
// Main Test Sequence
// ============================================================
//initial begin
//  // Initialize signals
//  reset = 0;
  
//  $display("\n");
//  $display("================================================================================");
//  $display("  Zynq UltraScale+ AXI DMA Testbench with AM Demodulator");
//  $display("================================================================================");
//  $display("  DMA Base Address: 0x%08h", DMA_BASE);
//  $display("  Buffer 1 (Source): 0x%08h", RAM_BUFER1);
//  $display("  Buffer 2 (Dest):   0x%08h", RAM_BUFER2);
//  $display("  Buffer Size: %0d bytes", RAM_BUFER_SIZE);
//  $display("  Number of Samples: %0d", num_samples);
//  $display("================================================================================\n");
  
//  //Reset MPSoC and PL  
//  $display("[%0t] Resetting Zynq MPSoC...", $time);
//  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b1);
//  repeat(20) @(posedge clock);
//  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b0);
//  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.fpga_soft_reset(32'h1);
//  repeat(20) @(posedge clock);
//  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.por_srstb_reset(1'b1);
//  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.fpga_soft_reset(32'h0);
//  #2000;
//  $display("[%0t] Reset sequence complete.\n", $time);
  
//  //Initialize DDR memory buffers
//  $display("[%0t] Initializing DDR memory buffers...", $time);
//  init_buffer_zero(RAM_BUFER1, RAM_BUFER_SIZE);
//  init_buffer_zero(RAM_BUFER2, RAM_BUFER_SIZE);
//  $display("\n[%0t] Buffer initialization complete.\n", $time);
  
//  // === OPTIONAL: Test with loopback first ===
//  // Uncomment this section to test basic DMA functionality without AM demodulator
//  // This will help isolate if the problem is in the DMA setup or the AM demodulator
//  /*
//  $display("\n========== LOOPBACK TEST MODE ===========");
//  $display("Testing DMA with direct loopback (bypassing AM demodulator)");
//  $display("If this fails, the problem is in DMA/HP0 configuration");
//  $display("If this works, the problem is in the AM demodulator");
//  $display("==========================================\n");
//  */
  
//  // Generate test data
//  $display("\n=== Test: AM Modulated Signal Generation ===");
//  generate_am_signal(
//    .sample_rate(480000.0),
//    .carrier_freq(100000.0),
//    .message_freq(1000.0),
//    .modulation_index(0.6),
//    .num_samples(num_samples),
//    .data_array(test_data)
//  );
  
//  // Write test data to RAM_BUFFER1
//  $display("\n[%0t] Writing test data to DDR memory...", $time);
//  //write_test_data_to_ddr(RAM_BUFER1, test_data, num_samples);
  
//  tb_receiver_with_zynq.DUT.mpsoc_preset_i.zynq_ultra_ps_e_0.inst.write_mem(test_data,RAM_BUFER1,num_samples);//,DDR_MEM,succ);

  
//  $display("[%0t] Test data write complete.\n", $time);
  
//  // Wait for some time
//  repeat(10) @(posedge clock);
  
//  // Release reset
//  reset = 1;
//  $display("[%0t] Reset released", $time);
  
//  // Wait for reset to propagate
//  repeat(20) @(posedge clock);
  
//  // Let agents initialize
//  repeat(10) @(posedge clock);

//  // Start S2MM FIRST (it must be ready to receive before MM2S sends)
//  $display("\n[%0t] ========================================", $time);
//  $display("[%0t] CRITICAL: Starting S2MM FIRST", $time);
//  $display("[%0t] S2MM must be ready before MM2S sends data!", $time);
//  $display("[%0t] ========================================\n", $time);
  
//  // Step 1: Configure and start S2MM receiver (non-blocking)
//  dma_s2mm_start(DMA_BASE, RAM_BUFER2, RAM_BUFER_SIZE/2);
  
//  // Step 2: Wait for S2MM to be ready
//  repeat(100) @(posedge clock);
  
//  // Step 3: Now start MM2S transmitter
//  $display("\n[%0t] S2MM ready, now starting MM2S...\n", $time);
//  dma_mm2s_start(DMA_BASE, RAM_BUFER1, RAM_BUFER_SIZE);
  
//  // Step 4: Wait for both to complete in parallel
//  fork
//    begin
//        dma_mm2s_wait_complete(DMA_BASE);
//    end
//    begin
//        dma_s2mm_wait_complete(DMA_BASE);
//    end
//  join

//  $display("\n[%0t] Both DMA transfers completed successfully!", $time);
  
//  // Read back data from RAM_BUFFER2
//  $display("\n[%0t] Reading back processed data from DDR...", $time);
//  read_data_from_ddr(RAM_BUFER2, received_data, num_samples);
//  num_received = num_samples;
  
//  // Save received data to file
//  $display("\n[%0t] Saving received data to file...", $time);
//  save_data_to_file("received_data.txt", received_data, num_received, 1);
  
//  // Wait for some additional time
//  repeat(100) @(posedge clock);
  
//  $display("\n");
//  $display("================================================================================");
//  $display("  Test COMPLETED Successfully!");
//  $display("================================================================================");
//  $display("  Total samples processed: %0d", num_received);
//  $display("  Output file: received_data.txt");
//  $display("================================================================================\n");
  
//  $finish;
//end

initial begin
    reset = 1;
    repeat(10) @(posedge clock);
    reset = 0;

    // ============================================================
    // 1. Generate AM-modulated data
    // ============================================================
    generate_am_signal(0.02,0.001,0.8,num_samples,test_data);

    // ============================================================
    // 2. Load into DDR using write_mem()
    // ============================================================
    write_mem_am_data(RAM_BUFER1, test_data, num_samples);

    // ============================================================
    // 3. Configure S2MM Receiver (before TX!)
    // ============================================================
    dma_s2mm_start(DMA_BASE, RAM_BUFER2, num_samples*2);

    // ============================================================
    // 4. Configure MM2S Transmitter
    // ============================================================
    dma_mm2s_start(DMA_BASE, RAM_BUFER1, num_samples*2);

    // ============================================================
    // 5. Wait for completion
    // ============================================================
    dma_mm2s_wait_complete(DMA_BASE);
    dma_s2mm_wait_complete(DMA_BASE);

    // ============================================================
    // 6. Read back received data
    // ============================================================
    read_data_from_ddr(RAM_BUFER2, received_data, num_samples);

    // save to file
    save_data_to_file("received_am_data.txt", received_data, num_samples, 1);

    $finish;
end


// ============================================================
// Clock Monitor (optional - helps debug hangs)
// ============================================================
initial begin
    int cycle_count = 0;
    forever begin
        @(posedge clock);
        cycle_count++;
        if(cycle_count % 50000 == 0) begin
            $display("[%0t] Heartbeat: %0d clock cycles elapsed...", $time, cycle_count);
        end
    end
end

endmodule