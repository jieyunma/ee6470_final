#include <string.h>
#include "stdio.h"
#include "filter_def.h"
#include "spikes.h"

#define SPIKE_ADDR    0x00000000
#define APPROX_ADDR   0x00000004
#define DETAIL_ADDR   0x00000008

// Sobel Filter ACC
static char* const DWT1_CORE0_BASE_ADDR = reinterpret_cast<char* const>(0x73000000);
static char* const DWT1_CORE1_BASE_ADDR = reinterpret_cast<char* const>(0x73100000);
static char* const DWT2_CORE0_BASE_ADDR = reinterpret_cast<char* const>(0x73200000);
static char* const DWT2_CORE1_BASE_ADDR = reinterpret_cast<char* const>(0x73300000);

// DMA 
static volatile uint32_t * const DMA_SRC_ADDR  = (uint32_t * const)0x70000000;
static volatile uint32_t * const DMA_DST_ADDR  = (uint32_t * const)0x70000004;
static volatile uint32_t * const DMA_LEN_ADDR  = (uint32_t * const)0x70000008;
static volatile uint32_t * const DMA_OP_ADDR   = (uint32_t * const)0x7000000C;
static volatile uint32_t * const DMA_STAT_ADDR = (uint32_t * const)0x70000010;
static const uint32_t DMA_OP_MEMCPY = 1;

bool _is_using_dma = true;
uint32_t dma_lock; 
uint32_t print_lock; 

int sem_init (uint32_t *__sem, uint32_t count) __THROW{
  *__sem=count;
  return 0;
}
int sem_wait (uint32_t *__sem) __THROW{
  uint32_t value, success; //RV32A
  __asm__ __volatile__("\
L%=:\n\t\
     lr.w %[value],(%[__sem])            # load reserved\n\t\
     beqz %[value],L%=                   # if zero, try again\n\t\
     addi %[value],%[value],-1           # value --\n\t\
     sc.w %[success],%[value],(%[__sem]) # store conditionally\n\t\
     bnez %[success], L%=                # if the store failed, try again\n\t\
"
    : [value] "=r"(value), [success]"=r"(success)
    : [__sem] "r"(__sem)
    : "memory");
  return 0;
}

int sem_post (uint32_t *__sem) __THROW{
  uint32_t value, success; //RV32A
  __asm__ __volatile__("\
L%=:\n\t\
     lr.w %[value],(%[__sem])            # load reserved\n\t\
     addi %[value],%[value], 1           # value ++\n\t\
     sc.w %[success],%[value],(%[__sem]) # store conditionally\n\t\
     bnez %[success], L%=                # if the store failed, try again\n\t\
"
    : [value] "=r"(value), [success]"=r"(success)
    : [__sem] "r"(__sem)
    : "memory");
  return 0;
}

void write_data_to_ACC(char* ADDR, int len, int value){
    word data;
    unsigned char buffer[4];

    data.uint = value;
    buffer[0] = data.uc[0];
    buffer[1] = data.uc[1];
    buffer[2] = data.uc[2];
    buffer[3] = data.uc[3];

    if(_is_using_dma){  
	    sem_wait(&dma_lock);
        *(DMA_SRC_ADDR) = (uint32_t)(buffer);
        *(DMA_DST_ADDR) = (uint32_t)(ADDR);
        *(DMA_LEN_ADDR) = len;
        *(DMA_OP_ADDR)  = DMA_OP_MEMCPY;
	    sem_post(&dma_lock);

    }else{
        memcpy(ADDR, buffer, sizeof(unsigned char)*len);
    }
}
int read_data_from_ACC(char* ADDR, int len){
	
    unsigned char buffer[4];
  	word data;
    if(_is_using_dma){
	    sem_wait(&dma_lock);
        *(DMA_SRC_ADDR) = (uint32_t)(ADDR);
        *(DMA_DST_ADDR) = (uint32_t)(buffer);
        *(DMA_LEN_ADDR) = len;
        *(DMA_OP_ADDR)  = DMA_OP_MEMCPY;
	    sem_post(&dma_lock);
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

void do_dwt_1(unsigned hart_id) {

  unsigned char  buffer[4] = {0};
  int start = (hart_id == 0 ? 0 : 50);
  int end   = (hart_id == 0 ? 50: 100);

  int i;
  for (i = start; i < end; i = i + 2) {    
// sem_wait(&print_lock);
// printf("-------------DWT1 %d----------------\n", i);
// sem_post(&print_lock);    
    if (hart_id == 0) {
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);
      write_data_to_ACC(DWT1_CORE0_BASE_ADDR + SPIKE_ADDR, 4, spike[i]);
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 finish send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);      
      write_data_to_ACC(DWT1_CORE0_BASE_ADDR + SPIKE_ADDR, 4, spike[i+1]);
      if (i >= start + 6) {
        dwth1_result[i] = read_data_from_ACC(DWT1_CORE0_BASE_ADDR + APPROX_ADDR, 4);
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 finish1 read %d\n", hart_id, dwth1_result[i]);
// sem_post(&print_lock);      
        dwtl1_result[i] = read_data_from_ACC(DWT1_CORE0_BASE_ADDR + DETAIL_ADDR, 4);
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 finish2 read %d\n", hart_id, dwth1_result[i]);
// sem_post(&print_lock);   
      }     
    }
    else {
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);      
      write_data_to_ACC(DWT1_CORE1_BASE_ADDR + SPIKE_ADDR, 4, spike[i]);
      write_data_to_ACC(DWT1_CORE1_BASE_ADDR + SPIKE_ADDR, 4, spike[i+1]);
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 finish send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);       
      if (i >= start + 6) {
// sem_wait(&print_lock);
// printf("core[%d]: dwt1 read %d\n", hart_id, dwth1_result[i]);
// sem_post(&print_lock);        
        dwth1_result[i] = read_data_from_ACC(DWT1_CORE1_BASE_ADDR + APPROX_ADDR, 4);
sem_wait(&print_lock);
printf("core[%d]: dwt1 finish read %d\n", hart_id, dwth1_result[i]);
sem_post(&print_lock);        
        dwtl1_result[i] = read_data_from_ACC(DWT1_CORE1_BASE_ADDR + DETAIL_ADDR, 4);
      }     
    }
  }
// sem_wait(&print_lock);
// printf("------------ FINISH DWT1 ----------------\n", i);
// sem_post(&print_lock);  
}

void do_dwt_2(unsigned hart_id) {

  unsigned char  buffer[4] = {0};
  int start = (hart_id == 0 ? 0 : 23);
  int end   = (hart_id == 0 ? 23: 46);

  int i;
  for (i = start; i < end; i = i + 2) {
    if (hart_id == 0) {
// sem_wait(&print_lock);
// printf("core[%d]: dwt2 send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);      
      write_data_to_ACC(DWT2_CORE0_BASE_ADDR + SPIKE_ADDR, 4, dwtl1_result[i]);
      write_data_to_ACC(DWT2_CORE1_BASE_ADDR + SPIKE_ADDR, 4, dwtl1_result[i+1]);
// sem_wait(&print_lock);
// printf("core[%d]: dwt2 finish send %d, %d to dma\n", hart_id, spike[i], spike[i+1]);
// sem_post(&print_lock);       
      if (i >= start + 6) {
        dwth2_result[i] = read_data_from_ACC(DWT2_CORE0_BASE_ADDR + APPROX_ADDR, 4);
        dwtl2_result[i] = read_data_from_ACC(DWT2_CORE0_BASE_ADDR + DETAIL_ADDR, 4);
// sem_wait(&print_lock);
// printf("high[%d] = %d, low[%d] = %d\n", i, dwth2_result[i], i, dwtl2_result[i]);
// sem_post(&print_lock);        
      }     
    }
    else {
      write_data_to_ACC(DWT2_CORE0_BASE_ADDR + SPIKE_ADDR, 4, dwtl1_result[i]);
      write_data_to_ACC(DWT2_CORE1_BASE_ADDR + SPIKE_ADDR, 4, dwtl1_result[i+1]);
      if (i >= start + 6) {
        dwth2_result[i] = read_data_from_ACC(DWT2_CORE1_BASE_ADDR + APPROX_ADDR, 4);
        dwtl2_result[i] = read_data_from_ACC(DWT2_CORE1_BASE_ADDR + DETAIL_ADDR, 4);
// sem_wait(&print_lock);
// printf("high[%d] = %d, low[%d] = %d\n", i, dwth2_result[i], i, dwtl2_result[i]);
// sem_post(&print_lock);      
      }
    }
  }
}

int main(unsigned hart_id) {

  if(hart_id == 0){
      sem_init(&dma_lock, 1);
      sem_init(&print_lock, 1);
  }

  sem_wait(&print_lock);
  printf("Core[%d] do_dwt1 start\n", (hart_id == 0 ? 0 : 1));
  sem_post(&print_lock);
  do_dwt_1(hart_id);

  sem_wait(&print_lock);
  printf("Core[%d] do_dwt2 start\n", (hart_id == 0 ? 0 : 1));
  sem_post(&print_lock);
  do_dwt_2(hart_id);

  sem_wait(&print_lock);
  printf("Core[%d] finish\n", (hart_id == 0 ? 0 : 1));
  sem_post(&print_lock);
  return 0;
}
