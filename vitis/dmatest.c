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

    for(int j=0; j<100; j++){
		/* Initialize Data */
		for(int i=0; i<SRC_LEN/4; i++) src[i] = 0x78563412 + i + j;
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

    }
    /* Unmap memory buffers and close /dev/mem file */
    munmap(dma, 0x10000);
    munmap(src, 0x1000);
    munmap(dst, 0x1000);
    close(fd);

    return 0;
}
