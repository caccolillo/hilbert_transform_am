#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <stdint.h>

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

/* File paths */
#define INPUT_FILE      "input_data.txt"
#define OUTPUT_FILE     "output_data.txt"

/* Helper Functions */
void reg_write(unsigned int *base, int offset, unsigned int val) { 
    base[offset/4] = val; 
}

unsigned int reg_read(unsigned int *base, int offset) { 
    return base[offset/4]; 
}

/* Read all 16-bit signed samples from text file */
int read_input_file(const char *filename, int16_t **samples_out) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("Failed to open input file");
        return -1;
    }
    
    // Allocate initial buffer
    int capacity = 1000;
    int16_t *samples = malloc(capacity * sizeof(int16_t));
    if (!samples) {
        perror("malloc failed");
        fclose(fp);
        return -1;
    }
    
    int count = 0;
    int16_t sample;
    
    // Read all samples from file
    while (fscanf(fp, "%hd", &sample) == 1) {
        if (count >= capacity) {
            // Expand buffer if needed
            capacity *= 2;
            int16_t *new_samples = realloc(samples, capacity * sizeof(int16_t));
            if (!new_samples) {
                perror("realloc failed");
                free(samples);
                fclose(fp);
                return -1;
            }
            samples = new_samples;
        }
        samples[count++] = sample;
    }
    
    fclose(fp);
    *samples_out = samples;
    printf("Read %d samples from %s\n", count, filename);
    return count;
}

/* Copy samples to DMA buffer, packing two 16-bit samples per 32-bit word */
void prepare_dma_buffer(unsigned int *buffer, int16_t *samples, int num_samples, 
                        int start_idx, int buffer_size_bytes) {
    int num_words = buffer_size_bytes / 4;  // Number of 32-bit words
    
    // Clear entire buffer first
    memset(buffer, 0, buffer_size_bytes);
    
    // Pack two 16-bit samples into each 32-bit word
    for (int i = 0; i < num_words; i++) {
        int sample_idx0 = start_idx + (i * 2);      // First sample (lower 16 bits)
        int sample_idx1 = start_idx + (i * 2) + 1;  // Second sample (upper 16 bits)
        
        uint16_t sample0 = (sample_idx0 < num_samples) ? (uint16_t)samples[sample_idx0] : 0;
        uint16_t sample1 = (sample_idx1 < num_samples) ? (uint16_t)samples[sample_idx1] : 0;
        
        // Pack: [sample1 (upper 16 bits) | sample0 (lower 16 bits)]
        buffer[i] = ((uint32_t)sample1 << 16) | (uint32_t)sample0;
    }
}

/* Unpack 16-bit samples from output buffer and write to text file */
void write_output_file(const char *filename, unsigned char *data, int length, int append) {
    FILE *fp = fopen(filename, append ? "a" : "w");
    if (!fp) {
        perror("Failed to open output file");
        return;
    }
    
    int16_t *samples = (int16_t *)data;
    int num_samples = length / 2;  // Each sample is 2 bytes
    
    for (int i = 0; i < num_samples; i++) {
        fprintf(fp, "%d\n", samples[i]);
    }
    
    fclose(fp);
}

int main() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { 
        perror("open"); 
        return -1; 
    }
    
    /* Map DMA and Buffers */
    unsigned int *dma = mmap(NULL, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, DMA_BASE);
    unsigned int *src = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SRC_ADDR);
    unsigned int *dst = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, DST_ADDR);
    
    if (dma == MAP_FAILED || src == MAP_FAILED || dst == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return -1;
    }
    
    /* Read all input samples from file */
    int16_t *input_samples = NULL;
    int total_samples = read_input_file(INPUT_FILE, &input_samples);
    if (total_samples < 0) {
        munmap(dma, 0x10000);
        munmap(src, 0x1000);
        munmap(dst, 0x1000);
        close(fd);
        return -1;
    }
    
    /* Calculate number of iterations needed */
    int samples_per_iteration = SRC_LEN / 2;  // 150 bytes = 75 samples
    int num_iterations = (total_samples + samples_per_iteration - 1) / samples_per_iteration;
    
    printf("Total samples: %d\n", total_samples);
    printf("Samples per iteration: %d\n", samples_per_iteration);
    printf("Number of iterations: %d\n", num_iterations);
    
    /* Clear output file */
    FILE *fp = fopen(OUTPUT_FILE, "w");
    if (fp) fclose(fp);
    
    for(int j = 0; j < num_iterations; j++){
        /* Prepare DMA source buffer with samples (pad with zeros if needed) */
        int start_idx = j * samples_per_iteration;
        prepare_dma_buffer(src, input_samples, total_samples, start_idx, SRC_LEN);
        
        /* Initialize destination buffer */
        memset(dst, 0, DST_LEN);
        
        /* Reset DMA */
        reg_write(dma, MM2S_DMACR, 0x4);
        reg_write(dma, S2MM_DMACR, 0x4);
        while (reg_read(dma, MM2S_DMACR) & 0x4); // Wait for reset
        
        /* STEP 1: Configure & Start S2MM (Receiver) FIRST */
        printf("Iteration %d/%d: Starting S2MM (Receiver)...\n", j+1, num_iterations);
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
            if (status & 0x2) { 
                printf("Transfer Successful!\n"); 
                break; 
            } // Idle bit
            if (status & 0x70) { 
                printf("DMA Error: 0x%08X\n", status); 
                break; 
            } // Error bits
        }
        
        if (timeout <= 0) 
            printf("Timeout: TLAST likely missing from IP core.\n");
        
        /* Print result */
        printf("Result (first %d samples): ", DST_LEN/2);
        int16_t *samples = (int16_t *)dst;
        for(int i = 0; i < DST_LEN/2; i++) printf("%d ", samples[i]);
        printf("\n\n");
        
        /* Write output to file (append mode) */
        write_output_file(OUTPUT_FILE, (unsigned char *)dst, DST_LEN, (j > 0));
    }
    
    printf("Output saved to %s\n", OUTPUT_FILE);
    
    /* Free allocated memory */
    free(input_samples);
    
    /* Unmap memory buffers and close /dev/mem file */
    munmap(dma, 0x10000);
    munmap(src, 0x1000);
    munmap(dst, 0x1000);
    close(fd);
    
    return 0;
}
