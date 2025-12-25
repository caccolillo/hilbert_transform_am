///*
// * DMA Test Application - Enhanced Diagnostic Version
// * Added detailed status polling and error detection
// */
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <string.h>
//#include <sys/mman.h>
//
//// ============================================================
//// AXI DMA Register Offsets (from PG021)
//// ============================================================
//#define MM2S_DMACR                  0x00
//#define MM2S_DMASR                  0x04
//#define MM2S_SA                     0x18
//#define MM2S_SA_MSB                 0x1C
//#define MM2S_LENGTH                 0x28
//
//#define S2MM_DMACR                  0x30
//#define S2MM_DMASR                  0x34
//#define S2MM_DA                     0x48
//#define S2MM_DA_MSB                 0x4C
//#define S2MM_LENGTH                 0x58
//
//// ============================================================
//// Status Register Bit Definitions
//// ============================================================
//#define STATUS_HALTED               0x00000001
//#define STATUS_IDLE                 0x00000002
//#define STATUS_SG_INCLDED           0x00000008
//#define STATUS_DMA_INTERNAL_ERR     0x00000010
//#define STATUS_DMA_SLAVE_ERR        0x00000020
//#define STATUS_DMA_DECODE_ERR       0x00000040
//#define STATUS_SG_INTERNAL_ERR      0x00000100
//#define STATUS_SG_SLAVE_ERR         0x00000200
//#define STATUS_SG_DECODE_ERR        0x00000400
//#define STATUS_IOC_IRQ              0x00001000
//#define STATUS_DELAY_IRQ            0x00002000
//#define STATUS_ERR_IRQ              0x00004000
//
//// Error mask - any of these bits indicate an error
//#define STATUS_ERROR_MASK           0x00004470
//
//// ============================================================
//// Control Register Values (matching testbench)
//// ============================================================
//#define RESET_DMA                   0x00000004
//#define HALT_DMA                    0x00000000
//#define ENABLE_DMA_MASKED_IRQ       0x0000F001  // IRQThreshold=0xF, RS=1
//
//// ============================================================
//// Memory Addresses and Buffer Sizes
//// ============================================================
//#define DMA_BASE_ADDR               0xA0000000
//#define SOURCE_ADDRESS              0x00000000     // RAM_BUFFER1
//#define DESTINATION_ADDRESS         0x00100000     // RAM_BUFFER2
//#define MM2S_TRANSFER_LENGTH        0x00000096     // 150 bytes (75 samples, 2 bytes each)
//#define S2MM_TRANSFER_LENGTH        0x0000000A     // 10 bytes (5 samples due to x15 decimation)
//
//// ============================================================
//// Function Prototypes
//// ============================================================
//unsigned int dma_write(unsigned int *virtual_addr, int offset, unsigned int value);
//unsigned int dma_read(unsigned int *virtual_addr, int offset);
//void dma_s2mm_status(unsigned int *virtual_addr);
//void dma_mm2s_status(unsigned int *virtual_addr);
//int dma_mm2s_idle(unsigned int *virtual_addr);
//int dma_s2mm_idle(unsigned int *virtual_addr);
//int check_dma_errors(unsigned int status, const char *channel);
//void start_mm2s_dma(unsigned int *dma_reg_addr, unsigned int buffer_addr, unsigned int buffer_size);
//void start_s2mm_dma(unsigned int *dma_reg_addr, unsigned int buffer_addr, unsigned int buffer_size);
//void load_test_data(void *address, int byte_length);
//void print_memory(void *address, int byte_count);
//void poll_dma_status(unsigned int *virtual_addr, int iterations);
//void print_status_inline(unsigned int status, const char *prefix);
//
//// ============================================================
//// Main Function
//// ============================================================
//int main()
//{
//    printf("=======================================================\n");
//    printf("AXI DMA Transfer Test - Enhanced Diagnostic Version\n");
//    printf("=======================================================\n\n");
//
//    // Open /dev/mem
//    printf("Opening /dev/mem...\n");
//    int dh = open("/dev/mem", O_RDWR | O_SYNC);
//    if (dh < 0) {
//        printf("ERROR: Failed to open /dev/mem\n");
//        return -1;
//    }
//
//    // Memory map DMA registers
//    printf("Mapping DMA control registers at 0x%08X...\n", DMA_BASE_ADDR);
//    unsigned int *dma_reg_addr = mmap(NULL, 65535, PROT_READ | PROT_WRITE,
//                                      MAP_SHARED, dh, DMA_BASE_ADDR);
//    if (dma_reg_addr == MAP_FAILED) {
//        printf("ERROR: Failed to map DMA registers\n");
//        close(dh);
//        return -1;
//    }
//
//    // Memory map source buffer
//    printf("Mapping source buffer at 0x%08X...\n", SOURCE_ADDRESS);
//    unsigned int *dma_source_addr = mmap(NULL, 65535, PROT_READ | PROT_WRITE,
//                                         MAP_SHARED, dh, SOURCE_ADDRESS);
//    if (dma_source_addr == MAP_FAILED) {
//        printf("ERROR: Failed to map source buffer\n");
//        munmap(dma_reg_addr, 65535);
//        close(dh);
//        return -1;
//    }
//
//    // Memory map destination buffer
//    printf("Mapping destination buffer at 0x%08X...\n", DESTINATION_ADDRESS);
//    unsigned int *dma_destination_addr = mmap(NULL, 65535, PROT_READ | PROT_WRITE,
//                                              MAP_SHARED, dh, DESTINATION_ADDRESS);
//    if (dma_destination_addr == MAP_FAILED) {
//        printf("ERROR: Failed to map destination buffer\n");
//        munmap(dma_reg_addr, 65535);
//        munmap(dma_source_addr, 65535);
//        close(dh);
//        return -1;
//    }
//
//    printf("\n=======================================================\n");
//    printf("Memory mapping successful\n");
//    printf("=======================================================\n\n");
//
//    // Load test data into source buffer
//    printf("Loading test data into source buffer...\n");
//    load_test_data(dma_source_addr, MM2S_TRANSFER_LENGTH);
//    printf("Source buffer contents (first 64 bytes):\n");
//    print_memory(dma_source_addr, 64);
//
//    // Clear destination buffer
//    printf("\nClearing destination buffer...\n");
//    memset(dma_destination_addr, 0, S2MM_TRANSFER_LENGTH);
//    printf("Destination buffer contents (before transfer):\n");
//    print_memory(dma_destination_addr, S2MM_TRANSFER_LENGTH);
//
//    // Check initial DMA status
//    printf("\n=======================================================\n");
//    printf("Initial DMA Status (Clock/Register Test)\n");
//    printf("=======================================================\n");
//    dma_mm2s_status(dma_reg_addr);
//    dma_s2mm_status(dma_reg_addr);
//
//    unsigned int mm2s_initial = dma_read(dma_reg_addr, MM2S_DMASR);
//    unsigned int s2mm_initial = dma_read(dma_reg_addr, S2MM_DMASR);
//
//    if (mm2s_initial == 0x00000000 && s2mm_initial == 0x00000000) {
//        printf("\nWARNING: Both status registers read as 0x00000000\n");
//        printf("This may indicate:\n");
//        printf("  - Clock is not running\n");
//        printf("  - Module is in reset\n");
//        printf("  - Wrong base address\n");
//        printf("  - Hardware not configured\n");
//    } else if (mm2s_initial == 0x00000001 && s2mm_initial == 0x00000001) {
//        printf("\nGOOD: DMA registers responding correctly (both HALTED)\n");
//        printf("Clock is running and DMA is accessible.\n");
//    }
//
//    // Start MM2S DMA transfer
//    printf("\n=======================================================\n");
//    printf("Starting MM2S DMA Transfer\n");
//    printf("=======================================================\n");
//    printf("Source Address: 0x%08X\n", SOURCE_ADDRESS);
//    printf("Transfer Length: %d bytes (150 bytes = 75 samples)\n", MM2S_TRANSFER_LENGTH);
//    start_mm2s_dma(dma_reg_addr, SOURCE_ADDRESS, MM2S_TRANSFER_LENGTH);
//    dma_mm2s_status(dma_reg_addr);
//
//    // Start S2MM DMA transfer
//    printf("\n=======================================================\n");
//    printf("Starting S2MM DMA Transfer\n");
//    printf("=======================================================\n");
//    printf("Destination Address: 0x%08X\n", DESTINATION_ADDRESS);
//    printf("Transfer Length: %d bytes (10 bytes = 5 samples after x15 decimation)\n", S2MM_TRANSFER_LENGTH);
//    start_s2mm_dma(dma_reg_addr, DESTINATION_ADDRESS, S2MM_TRANSFER_LENGTH);
//    dma_s2mm_status(dma_reg_addr);
//
//    // Poll DMA status during transfer
//    printf("\n=======================================================\n");
//    printf("Polling DMA Status During Transfer\n");
//    printf("=======================================================\n");
//    poll_dma_status(dma_reg_addr, 20);
//
//    // Wait for MM2S to complete
//    printf("\n=======================================================\n");
//    printf("Waiting for DMA Transfers to Complete\n");
//    printf("=======================================================\n");
//    printf("Waiting for MM2S channel...\n");
//    if (dma_mm2s_idle(dma_reg_addr) < 0) {
//        printf("ERROR: MM2S transfer failed\n");
//        goto cleanup;
//    }
//    printf("MM2S channel is IDLE\n");
//    dma_mm2s_status(dma_reg_addr);
//
//    // Wait for S2MM to complete
//    printf("\nWaiting for S2MM channel...\n");
//    if (dma_s2mm_idle(dma_reg_addr) < 0) {
//        printf("ERROR: S2MM transfer failed\n");
//        goto cleanup;
//    }
//    printf("S2MM channel is IDLE\n");
//    dma_s2mm_status(dma_reg_addr);
//
//    // Display final status
//    printf("\n=======================================================\n");
//    printf("DMA Transfer Complete\n");
//    printf("=======================================================\n");
//    dma_mm2s_status(dma_reg_addr);
//    dma_s2mm_status(dma_reg_addr);
//
//    // Verify transferred data
//    printf("\n=======================================================\n");
//    printf("Verifying Transferred Data\n");
//    printf("=======================================================\n");
//    printf("Destination buffer contents (after transfer - all %d bytes):\n", S2MM_TRANSFER_LENGTH);
//    print_memory(dma_destination_addr, S2MM_TRANSFER_LENGTH);
//
//    printf("\nNote: Cannot directly compare source and destination due to x15 decimation in HLS IP\n");
//    printf("Source: %d bytes (75 samples) -> Destination: %d bytes (5 samples)\n",
//           MM2S_TRANSFER_LENGTH, S2MM_TRANSFER_LENGTH);
//
//cleanup:
//    // Cleanup
//    printf("\n=======================================================\n");
//    printf("Cleaning up...\n");
//    printf("=======================================================\n");
//    munmap(dma_reg_addr, 65535);
//    munmap(dma_source_addr, 65535);
//    munmap(dma_destination_addr, 65535);
//    close(dh);
//    printf("Done.\n\n");
//
//    return 0;
//}
//
//// ============================================================
//// DMA Programming Functions (matching testbench)
//// ============================================================
//
//void start_mm2s_dma(unsigned int *dma_reg_addr, unsigned int buffer_addr, unsigned int buffer_size)
//{
//    printf("  [1] Resetting MM2S channel...\n");
//    dma_write(dma_reg_addr, MM2S_DMACR, RESET_DMA);
//
//    printf("  [2] Halting MM2S channel...\n");
//    dma_write(dma_reg_addr, MM2S_DMACR, HALT_DMA);
//
//    printf("  [3] Setting source address to 0x%08X...\n", buffer_addr);
//    dma_write(dma_reg_addr, MM2S_SA, buffer_addr);
//
//    printf("  [4] Enabling MM2S with masked interrupts (0x%08X)...\n", ENABLE_DMA_MASKED_IRQ);
//    dma_write(dma_reg_addr, MM2S_DMACR, ENABLE_DMA_MASKED_IRQ);
//
//    printf("  [5] Starting transfer with length %d bytes...\n", buffer_size);
//    dma_write(dma_reg_addr, MM2S_LENGTH, buffer_size);
//}
//
//void start_s2mm_dma(unsigned int *dma_reg_addr, unsigned int buffer_addr, unsigned int buffer_size)
//{
//    printf("  [1] Resetting S2MM channel...\n");
//    dma_write(dma_reg_addr, S2MM_DMACR, RESET_DMA);
//
//    printf("  [2] Halting S2MM channel...\n");
//    dma_write(dma_reg_addr, S2MM_DMACR, HALT_DMA);
//
//    printf("  [3] Setting destination address to 0x%08X...\n", buffer_addr);
//    dma_write(dma_reg_addr, S2MM_DA, buffer_addr);
//
//    printf("  [4] Enabling S2MM with masked interrupts (0x%08X)...\n", ENABLE_DMA_MASKED_IRQ);
//    dma_write(dma_reg_addr, S2MM_DMACR, ENABLE_DMA_MASKED_IRQ);
//
//    printf("  [5] Starting transfer with length %d bytes...\n", buffer_size);
//    dma_write(dma_reg_addr, S2MM_LENGTH, buffer_size);
//}
//
//// ============================================================
//// DMA Register Access Functions
//// ============================================================
//
//unsigned int dma_write(unsigned int *virtual_addr, int offset, unsigned int value)
//{
//    virtual_addr[offset >> 2] = value;
//    return 0;
//}
//
//unsigned int dma_read(unsigned int *virtual_addr, int offset)
//{
//    return virtual_addr[offset >> 2];
//}
//
//// ============================================================
//// DMA Synchronization Functions
//// ============================================================
//
//int dma_mm2s_idle(unsigned int *virtual_addr)
//{
//    unsigned int mm2s_status;
//    int timeout = 1000000;  // Timeout counter
//
//    do {
//        mm2s_status = dma_read(virtual_addr, MM2S_DMASR);
//
//        // Check for errors
//        if (check_dma_errors(mm2s_status, "MM2S") < 0) {
//            return -1;
//        }
//
//        // Timeout check
//        if (--timeout == 0) {
//            printf("ERROR: MM2S timeout waiting for idle\n");
//            printf("Final MM2S status: 0x%08X\n", mm2s_status);
//            return -1;
//        }
//
//    } while (!(mm2s_status & STATUS_IDLE));
//
//    return 0;
//}
//
//int dma_s2mm_idle(unsigned int *virtual_addr)
//{
//    unsigned int s2mm_status;
//    int timeout = 1000000;  // Timeout counter
//
//    do {
//        s2mm_status = dma_read(virtual_addr, S2MM_DMASR);
//
//        // Check for errors
//        if (check_dma_errors(s2mm_status, "S2MM") < 0) {
//            return -1;
//        }
//
//        // Timeout check
//        if (--timeout == 0) {
//            printf("ERROR: S2MM timeout waiting for idle\n");
//            printf("Final S2MM status: 0x%08X\n", s2mm_status);
//            return -1;
//        }
//
//    } while (!(s2mm_status & STATUS_IDLE));
//
//    return 0;
//}
//
//// ============================================================
//// Enhanced Status Polling Function
//// ============================================================
//
//void poll_dma_status(unsigned int *virtual_addr, int iterations)
//{
//    for (int i = 0; i < iterations; i++) {
//        unsigned int mm2s_status = dma_read(virtual_addr, MM2S_DMASR);
//        unsigned int s2mm_status = dma_read(virtual_addr, S2MM_DMASR);
//
//        printf("[%2d] MM2S: 0x%08X ", i, mm2s_status);
//        print_status_inline(mm2s_status, "");
//
//        printf("| S2MM: 0x%08X ", s2mm_status);
//        print_status_inline(s2mm_status, "");
//        printf("\n");
//
//        // Check for any errors
//        if (mm2s_status & STATUS_ERROR_MASK) {
//            printf("  >>> ERROR detected in MM2S status! <<<\n");
//            check_dma_errors(mm2s_status, "MM2S");
//            break;
//        }
//        if (s2mm_status & STATUS_ERROR_MASK) {
//            printf("  >>> ERROR detected in S2MM status! <<<\n");
//            check_dma_errors(s2mm_status, "S2MM");
//            break;
//        }
//
//        // If both are idle, we're done
//        if ((mm2s_status & STATUS_IDLE) && (s2mm_status & STATUS_IDLE)) {
//            printf("\n  >>> Both channels completed successfully! <<<\n");
//            break;
//        }
//
//        usleep(50000); // 50ms between polls
//    }
//}
//
//void print_status_inline(unsigned int status, const char *prefix)
//{
//    printf("%s", prefix);
//
//    if (status & STATUS_HALTED) printf("HALT ");
//    if (status & STATUS_IDLE) printf("IDLE ");
//    if (status & STATUS_IOC_IRQ) printf("IOC ");
//    if (status & STATUS_DELAY_IRQ) printf("DELAY ");
//    if (status & STATUS_ERR_IRQ) printf("ERR_IRQ ");
//    if (status & STATUS_DMA_INTERNAL_ERR) printf("INT_ERR ");
//    if (status & STATUS_DMA_SLAVE_ERR) printf("SLV_ERR ");
//    if (status & STATUS_DMA_DECODE_ERR) printf("DEC_ERR ");
//
//    // If no flags, indicate running
//    if (status == 0x00000000) {
//        printf("RUNNING ");
//    }
//}
//
//// ============================================================
//// Error Checking Function
//// ============================================================
//
//int check_dma_errors(unsigned int status, const char *channel)
//{
//    int error_found = 0;
//
//    if (status & STATUS_DMA_INTERNAL_ERR) {
//        printf("ERROR [%s]: DMA internal error detected!\n", channel);
//        error_found = 1;
//    }
//
//    if (status & STATUS_DMA_SLAVE_ERR) {
//        printf("ERROR [%s]: DMA slave error detected!\n", channel);
//        error_found = 1;
//    }
//
//    if (status & STATUS_DMA_DECODE_ERR) {
//        printf("ERROR [%s]: DMA decode error detected!\n", channel);
//        error_found = 1;
//    }
//
//    if (status & STATUS_SG_INTERNAL_ERR) {
//        printf("ERROR [%s]: SG internal error detected!\n", channel);
//        error_found = 1;
//    }
//
//    if (status & STATUS_SG_SLAVE_ERR) {
//        printf("ERROR [%s]: SG slave error detected!\n", channel);
//        error_found = 1;
//    }
//
//    if (status & STATUS_SG_DECODE_ERR) {
//        printf("ERROR [%s]: SG decode error detected!\n", channel);
//        error_found = 1;
//    }
//
//    if (status & STATUS_ERR_IRQ) {
//        printf("ERROR [%s]: Error interrupt detected!\n", channel);
//        error_found = 1;
//    }
//
//    return error_found ? -1 : 0;
//}
//
//// ============================================================
//// DMA Status Display Functions
//// ============================================================
//
//void dma_mm2s_status(unsigned int *virtual_addr)
//{
//    unsigned int status = dma_read(virtual_addr, MM2S_DMASR);
//
//    printf("MM2S Status (0x%08X @ offset 0x%02X):\n", status, MM2S_DMASR);
//
//    if (status & STATUS_HALTED) {
//        printf("  - Halted\n");
//    } else {
//        printf("  - Running\n");
//    }
//
//    if (status & STATUS_IDLE) {
//        printf("  - Idle\n");
//    }
//
//    if (status & STATUS_SG_INCLDED) {
//        printf("  - Scatter/Gather enabled\n");
//    }
//
//    if (status & STATUS_IOC_IRQ) {
//        printf("  - IOC interrupt occurred\n");
//    }
//
//    if (status & STATUS_DELAY_IRQ) {
//        printf("  - Delay interrupt occurred\n");
//    }
//
//    // Error flags
//    if (status & STATUS_DMA_INTERNAL_ERR) {
//        printf("  - ERROR: DMA internal error\n");
//    }
//    if (status & STATUS_DMA_SLAVE_ERR) {
//        printf("  - ERROR: DMA slave error\n");
//    }
//    if (status & STATUS_DMA_DECODE_ERR) {
//        printf("  - ERROR: DMA decode error\n");
//    }
//    if (status & STATUS_ERR_IRQ) {
//        printf("  - ERROR: Error interrupt\n");
//    }
//}
//
//void dma_s2mm_status(unsigned int *virtual_addr)
//{
//    unsigned int status = dma_read(virtual_addr, S2MM_DMASR);
//
//    printf("S2MM Status (0x%08X @ offset 0x%02X):\n", status, S2MM_DMASR);
//
//    if (status & STATUS_HALTED) {
//        printf("  - Halted\n");
//    } else {
//        printf("  - Running\n");
//    }
//
//    if (status & STATUS_IDLE) {
//        printf("  - Idle\n");
//    }
//
//    if (status & STATUS_SG_INCLDED) {
//        printf("  - Scatter/Gather enabled\n");
//    }
//
//    if (status & STATUS_IOC_IRQ) {
//        printf("  - IOC interrupt occurred\n");
//    }
//
//    if (status & STATUS_DELAY_IRQ) {
//        printf("  - Delay interrupt occurred\n");
//    }
//
//    // Error flags
//    if (status & STATUS_DMA_INTERNAL_ERR) {
//        printf("  - ERROR: DMA internal error\n");
//    }
//    if (status & STATUS_DMA_SLAVE_ERR) {
//        printf("  - ERROR: DMA slave error\n");
//    }
//    if (status & STATUS_DMA_DECODE_ERR) {
//        printf("  - ERROR: DMA decode error\n");
//    }
//    if (status & STATUS_ERR_IRQ) {
//        printf("  - ERROR: Error interrupt\n");
//    }
//}
//
//// ============================================================
//// Utility Functions
//// ============================================================
//
//void load_test_data(void *address, int byte_length)
//{
//    int *addr = (int *)address;
//    int num_words = byte_length / 4;
//
//    for (int i = 0; i < num_words; i++) {
//        addr[i] = 0x12345678 + i;
//    }
//}
//
//void print_memory(void *address, int byte_count)
//{
//    unsigned char *data_ptr = (unsigned char *)address;
//
//    for (int i = 0; i < byte_count; i++) {
//        printf("%02X", data_ptr[i]);
//
//        // Print space every 4 bytes
//        if ((i + 1) % 4 == 0) {
//            printf(" ");
//        }
//
//        // Print newline every 16 bytes
//        if ((i + 1) % 16 == 0) {
//            printf("\n");
//        }
//    }
//
//    if (byte_count % 16 != 0) {
//        printf("\n");
//    }
//}

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>

/* Register Offsets */
#define MM2S_DMACR      0x00
#define MM2S_DMASR      0x04
#define MM2S_SA         0x18
#define MM2S_SA_MSB     0x1C
#define MM2S_LENGTH     0x28

#define S2MM_DMACR      0x30
#define S2MM_DMASR      0x34
#define S2MM_DA         0x48
#define S2MM_DA_MSB     0x4C
#define S2MM_LENGTH     0x58

/* Addresses & Config */
#define DMA_BASE        0xA0000000
#define SRC_ADDR        0x30000000  // High memory (avoiding 0x0)
#define DST_ADDR        0x31000000
#define SRC_LEN         150         // MM2S bytes
#define DST_LEN         10          // S2MM bytes (after decimation)

/* Helper Functions */
void reg_write(unsigned int *base, int offset, unsigned int val) { base[offset/4] = val; }
unsigned int reg_read(unsigned int *base, int offset) { return base[offset/4]; }

int main() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open"); return -1; }

    /* Map DMA and Buffers */
    unsigned int *dma = mmap(NULL, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, DMA_BASE);
    unsigned int *src = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SRC_ADDR);
    unsigned int *dst = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, DST_ADDR);

    /* Initialize Data */
    for(int i=0; i<SRC_LEN/4; i++) src[i] = 0x78563412 + i;
    memset(dst, 0, DST_LEN);

    /* Reset DMA */
    reg_write(dma, MM2S_DMACR, 0x4);
    reg_write(dma, S2MM_DMACR, 0x4);
    while (reg_read(dma, MM2S_DMACR) & 0x4); // Wait for reset

    /* STEP 1: Configure & Start S2MM (Receiver) FIRST */
    printf("Starting S2MM (Receiver)...\n");
    reg_write(dma, S2MM_DA, (unsigned int)(DST_ADDR & 0xFFFFFFFF));
    reg_write(dma, S2MM_DA_MSB, (unsigned int)(DST_ADDR >> 32)); // 40-bit support
    reg_write(dma, S2MM_DMACR, 0x0001); // Run S2MM
    reg_write(dma, S2MM_LENGTH, DST_LEN); // Writing length triggers transfer

    /* STEP 2: Configure & Start MM2S (Sender) SECOND */
    printf("Starting MM2S (Sender)...\n");
    reg_write(dma, MM2S_SA, (unsigned int)(SRC_ADDR & 0xFFFFFFFF));
    reg_write(dma, MM2S_SA_MSB, (unsigned int)(SRC_ADDR >> 32));
    reg_write(dma, MM2S_DMACR, 0x0001); // Run MM2S
    reg_write(dma, MM2S_LENGTH, SRC_LEN);

    /* STEP 3: Poll S2MM Status for IDLE or Error */
    printf("Waiting for completion...\n");
    int timeout = 1000000;
    while (timeout--) {
        unsigned int status = reg_read(dma, S2MM_DMASR);
        if (status & 0x2) { printf("Transfer Successful!\n"); break; } // Idle bit
        if (status & 0x70) { printf("DMA Error: 0x%08X\n", status); break; } // Error bits
    }
    if (timeout <= 0) printf("Timeout: TLAST likely missing from IP core.\n");

    /* Print result */
    printf("Result (first 10 bytes): ");
    unsigned char *ptr = (unsigned char *)dst;
    for(int i=0; i<10; i++) printf("%02X ", ptr[i]);
    printf("\n");

    return 0;
}


