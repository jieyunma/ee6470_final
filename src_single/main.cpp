#include <string.h>
#include "stdio.h"
#include "filter_def.h"
#include "spikes.h"

#define SPIKE_ADDR    0x00000000
#define APPROX_ADDR   0x00000004
#define DETAIL_ADDR   0x00000008

// Sobel Filter ACC
static char* const DWT1_BASE_ADDR = reinterpret_cast<char* const>(0x73000000);
static char* const DWT2_BASE_ADDR = reinterpret_cast<char* const>(0x73100000);

// DMA 
static volatile uint32_t * const DMA_SRC_ADDR  = (uint32_t * const)0x70000000;
static volatile uint32_t * const DMA_DST_ADDR  = (uint32_t * const)0x70000004;
static volatile uint32_t * const DMA_LEN_ADDR  = (uint32_t * const)0x70000008;
static volatile uint32_t * const DMA_OP_ADDR   = (uint32_t * const)0x7000000C;
static volatile uint32_t * const DMA_STAT_ADDR = (uint32_t * const)0x70000010;
static const uint32_t DMA_OP_MEMCPY = 1;

bool _is_using_dma = false;

void write_data_to_ACC(char* ADDR, int len, int value){
    word data;
    unsigned char buffer[4];

    data.uint = value;
    buffer[0] = data.uc[0];
    buffer[1] = data.uc[1];
    buffer[2] = data.uc[2];
    buffer[3] = data.uc[3];

    if(_is_using_dma){  
      *(DMA_SRC_ADDR) = (uint32_t)(buffer);
      *(DMA_DST_ADDR) = (uint32_t)(ADDR);
      *(DMA_LEN_ADDR) = len;
      *(DMA_OP_ADDR)  = DMA_OP_MEMCPY;

    }else{
      memcpy(ADDR, buffer, sizeof(unsigned char)*len);
    }
}
int read_data_from_ACC(char* ADDR, int len){
	
    unsigned char buffer[4];
  	word data;
    if(_is_using_dma){
        *(DMA_SRC_ADDR) = (uint32_t)(ADDR);
        *(DMA_DST_ADDR) = (uint32_t)(buffer);
        *(DMA_LEN_ADDR) = len;
        *(DMA_OP_ADDR)  = DMA_OP_MEMCPY;
    }else{
        memcpy(buffer, ADDR, sizeof(unsigned char)*len);
    }
	
    data.uc[0] = buffer[0];
    data.uc[1] = buffer[1];
    data.uc[2] = buffer[2];
    data.uc[3] = buffer[3];

	return data.uint;
}

int dwth1_result[50];
int dwtl1_result[50];
int dwth2_result[25];
int dwtl2_result[25];

void do_dwt_1() {

  unsigned char  buffer[4] = {0};

  int i;
  for (i = 0; i < 100; i = i + 2) {    
// sem_wait(&print_lock);
// printf("-------------DWT1 %d----------------\n", i);
// sem_post(&print_lock);    
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);
      write_data_to_ACC(DWT1_BASE_ADDR + SPIKE_ADDR, 4, spike[i]);
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 finish send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);      
      write_data_to_ACC(DWT1_BASE_ADDR + SPIKE_ADDR, 4, spike[i+1]);
      if (i >= 6) {
        dwth1_result[i] = read_data_from_ACC(DWT1_BASE_ADDR + APPROX_ADDR, 4);
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 finish1 read %d\n", hart_id, dwth1_result[i]);
// sem_post(&print_lock);      
        dwtl1_result[i] = read_data_from_ACC(DWT1_BASE_ADDR + DETAIL_ADDR, 4);
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 finish2 read %d\n", hart_id, dwth1_result[i]);
// sem_post(&print_lock);   
// printf("high[%d] = %d, low[%d] = %d\n", i, dwth1_result[i], i, dwtl1_result[i]);
      }     
  }
}

void do_dwt_2() {

  unsigned char  buffer[4] = {0};

  int i;
  for (i = 0; i < 46; i = i + 2) {
// sem_wait(&print_lock);
// printf("core[%d]: dwt2 send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);      
      write_data_to_ACC(DWT2_BASE_ADDR + SPIKE_ADDR, 4, dwtl1_result[i]);
      write_data_to_ACC(DWT2_BASE_ADDR + SPIKE_ADDR, 4, dwtl1_result[i+1]);
// sem_wait(&print_lock);
// printf("core[%d]: dwt2 finish send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);       
      if (i >= 6) {
        dwth2_result[i] = read_data_from_ACC(DWT2_BASE_ADDR + APPROX_ADDR, 4);
        dwtl2_result[i] = read_data_from_ACC(DWT2_BASE_ADDR + DETAIL_ADDR, 4);
printf("high[%d] = %d, low[%d] = %d\n", i, dwth2_result[i], i, dwtl2_result[i]);
      }     
  }
}

int main() {

  printf("do_dwt1 start\n");
  do_dwt_1();

  printf("do_dwt2 start\n");
  do_dwt_2();

  printf("finish\n");
  return 0;
}
