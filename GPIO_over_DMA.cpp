#include "memory_table.hpp"
#include <signal.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <assert.h>
#include <queue>

using namespace std;

// 8 GPIOs to use for driving servos
static uint8_t gpio_list[] = {
  4,    // P1-7
  17,    // P1-11
  18,    // P1-12
  21,    // P1-13
  22,    // P1-15
  23,    // P1-16
  24,    // P1-18
  25,    // P1-22
};

//#define DBG             1

#define RT_PRIORITY

#define NUM_GPIOS       (sizeof(gpio_list)/sizeof(gpio_list[0]))
#ifndef PAGE_SIZE
#define PAGE_SIZE            4096
#endif

// Memory Addresses
#define DMA_BASE        0x20007000
#define DMA15_BASE      0x20E05000
#define DMA_LEN         0x1000
#define PWM_BASE        0x2020C000
#define PWM_LEN         0x28
#define CLK_BASE        0x20101000
#define CLK_LEN         0xA8
#define GPIO_BASE       0x20200000
#define GPIO_LEN        0x100
#define PCM_BASE        0x20203000
#define PCM_LEN         0x24
#define TIMER_BASE      0x7E003004

#define DMA_SRC_INC     (1<<8)
#define DMA_DEST_INC    (1<<4)
#define DMA_NO_WIDE_BURSTS  (1<<26)
#define DMA_WAIT_RESP   (1<<3)
#define DMA_D_DREQ      (1<<6)
#define DMA_PER_MAP(x)  ((x)<<16)
#define DMA_END         (1<<1)
#define DMA_RESET       (1<<31)
#define DMA_INT         (1<<2)

#define DMA_CS          (0x00/4)
#define DMA_CONBLK_AD   (0x04/4)
#define DMA_DEBUG       (0x20/4)

// GPIO Memory Addresses
#define GPIO_LEV0_ADDR  0x7E200034 
#define GPIO_FSEL0      (0x00/4)
#define GPIO_SET0       (0x1c/4)
#define GPIO_CLR0       (0x28/4)
#define GPIO_LEV0       (0x34/4)
#define GPIO_PULLEN     (0x94/4)
#define GPIO_PULLCLK    (0x98/4)

// GPIO Modes (IN=0, OUT=1)
#define GPIO_MODE_IN    0
#define GPIO_MODE_OUT   1

// PWM Memory Addresses
#define PWM_CTL         (0x00/4)
#define PWM_DMAC        (0x08/4)
#define PWM_RNG1        (0x10/4)
#define PWM_FIFO        (0x18/4)

#define PWMCLK_CNTL     40
#define PWMCLK_DIV      41

#define PWMCTL_MODE1    (1<<1)
#define PWMCTL_PWEN1    (1<<0)
#define PWMCTL_CLRF     (1<<6)
#define PWMCTL_USEF1    (1<<5)

#define PWMDMAC_ENAB    (1<<31)
#define PWMDMAC_THRSHLD ((15<<8) | (15<<0))

#define PCM_CS_A        (0x00/4)
#define PCM_FIFO_A      (0x04/4)
#define PCM_MODE_A      (0x08/4)
#define PCM_RXC_A       (0x0c/4)
#define PCM_TXC_A       (0x10/4)
#define PCM_DREQ_A      (0x14/4)
#define PCM_INTEN_A     (0x18/4)
#define PCM_INT_STC_A   (0x1c/4)
#define PCM_GRAY        (0x20/4)

#define PCMCLK_CNTL     38
#define PCMCLK_DIV      39

#define DELAY_VIA_PWM   0
#define DELAY_VIA_PCM   1

typedef struct {
  uint32_t info, src, dst, length,
    stride, next, pad[2];
} dma_cb_t;

unsigned int ppmSyncLength     = 4000;   // Length of PPM sync pause
unsigned int ppmChannelsNumber = 8;      // Number of channels packed in PPM

uint32_t buffer_length = 5; //buffer length divided by 16 kilobytes
uint32_t proc_delay = 10000;
uint32_t proc_priority = 99;
uint32_t sample_freq = 1000; // could be 1, 2, 5, 10, 25, 50
uint32_t DMA_channel = 5;


static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;
static volatile uint32_t* destination;
static volatile memory_table_t* trans_page;
static volatile memory_table_t* con_blocks;
uint32_t channels[8];
uint32_t counter2 = 0;
pthread_t _signal_handler;
pthread_t _output_thread, _time_thread;
queue<uint64_t> output_queue;
queue<uint64_t> time_queue;
uint64_t output_buffer[40000];

static void
gpio_set_mode(uint32_t pin, uint32_t mode)
{
  uint32_t fsel = gpio_reg[GPIO_FSEL0 + pin/10];

  fsel &= ~(7 << ((pin % 10) * 3));
  fsel |= mode << ((pin % 10) * 3);
  gpio_reg[GPIO_FSEL0 + pin/10] = fsel;
}

// Sets the gpio to input (level=1) or output (level=0)
static void
gpio_set(int pin, int level)
{
  if (level)
    gpio_reg[GPIO_SET0] = 1 << pin;
  else
    gpio_reg[GPIO_CLR0] = 1 << pin;
}

// Very short delay
static void
udelay(int us)
{
  struct timespec ts = { 0, us * 1000 };

  nanosleep(&ts, NULL);
}

// More memory mapping
static void *
map_peripheral(uint32_t base, uint32_t len)
{
  int fd = open("/dev/mem", O_RDWR);
  void * vaddr;

  if (fd < 0)
    printf("rpio-pwm: Failed to open /dev/mem: %m\n");
  vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
  if (vaddr == MAP_FAILED)
    printf("rpio-pwm: Failed to map peripheral at 0x%08x: %m\n", base);
  close(fd);

  return vaddr;
}



//Method to init DMA control block
static void init_dma_cb(dma_cb_t** cbp, uint32_t mode, uint32_t source, uint32_t dest, uint32_t length, uint32_t stride, uint32_t next_cb)
{
  (*cbp)->info = mode;
  (*cbp)->src = source;
  (*cbp)->dst = dest;
  (*cbp)->length = length;
  (*cbp)->next = next_cb;
  (*cbp)->stride = stride;
}

static void
init_ctrl_data(volatile memory_table_t* mem_table, volatile memory_table_t* con_blocks)
{
  uint32_t phys_fifo_addr, i;
  uint32_t dest = 0;
  dma_cb_t* cbp = 0;
  dma_cb_t* cbp_curr;
  //Set fifo addr (for delay)
  phys_fifo_addr = (PCM_BASE | 0x7e000000) + 0x04;
  
  //Init dma control blocks. For 960 i it is created 1024 control blocks (it is 
  for (i = 0; i < 3840 * buffer_length; i++) 
    {
      //Transfer timer every 30th sample
      if(i % 30 == 0){
	cbp_curr = (dma_cb_t*)mt_get_virt_from_pointer(con_blocks, (uint32_t) cbp);

	init_dma_cb(&cbp_curr, DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_INC | DMA_SRC_INC, TIMER_BASE, (uint32_t) mt_get_phys_from_pointer(mem_table, dest), 8, 0, (uint32_t) mt_get_phys_from_pointer(con_blocks, (uint32_t)(cbp + 1)));
	dest += 8;
	cbp++;
      } 
      // Transfer GPIO
      cbp_curr = (dma_cb_t*)mt_get_virt_from_pointer(con_blocks, (uint32_t) cbp);
      init_dma_cb(&cbp_curr, DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP, GPIO_LEV0_ADDR, (uint32_t) mt_get_phys_from_pointer(mem_table, dest), 4, 0, (uint32_t)  mt_get_phys_from_pointer(con_blocks, (uint32_t)(cbp + 1)));
      dest += 4;
      cbp++;
      // Delay
      cbp_curr = (dma_cb_t*)mt_get_virt_from_pointer(con_blocks, (uint32_t) cbp);
      init_dma_cb(&cbp_curr, DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2), TIMER_BASE, phys_fifo_addr, 4, 0, (uint32_t)  mt_get_phys_from_pointer(con_blocks, (uint32_t)(cbp + 1)));
      cbp++;
    }

  cbp--;
  ((dma_cb_t*)mt_get_virt_from_pointer(con_blocks, (uint32_t)cbp))->next = (uint32_t) mt_get_phys_addr(con_blocks, con_blocks->virt_pages[0]);
  //  printf("virt_con_next %#x, phys_con_next %#x\n", cbp_curr + 1, cbp_curr->next);
}

// Initialize PWM (or PCM) and DMA
static void
init_hardware(uint32_t physCb)
{
  // Initialise PCM
  pcm_reg[PCM_CS_A] = 1;                // Disable Rx+Tx, Enable PCM block
  udelay(100);
  clk_reg[PCMCLK_CNTL] = 0x5A000006;        // Source=PLLD (500MHz)
  udelay(100);
  clk_reg[PCMCLK_DIV] = 0x5A000000 | ((50000/sample_freq)<<12);    // Set pcm div to 500, giving 1MHz
  udelay(100);
  clk_reg[PCMCLK_CNTL] = 0x5A000016;        // Source=PLLD and enable
  udelay(100);
  pcm_reg[PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
  udelay(100);
  pcm_reg[PCM_MODE_A] = (10 - 1) << 10;
  udelay(100);
  pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;        // Clear FIFOs
  udelay(100);
  pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;        // DMA Req when one slot is free?
  udelay(100);
  pcm_reg[PCM_CS_A] |= 1<<9;            // Enable DMA
  udelay(100);
  pcm_reg[PCM_CS_A] |= 1<<2;            // Enable Tx
  udelay(100);

  // Initialise the DMA

  dma_reg[DMA_CS | DMA_channel << 8] = DMA_RESET;
  udelay(10);
  dma_reg[DMA_CS | DMA_channel << 8] = DMA_INT | DMA_END;
  dma_reg[DMA_CONBLK_AD | DMA_channel << 8] = physCb;
  dma_reg[DMA_DEBUG | DMA_channel << 8] = 7; // clear debug error flags
  dma_reg[DMA_CS | DMA_channel << 8] = 0x10880001;    // go, mid priority, wait for outstanding writes
}

uint32_t bytes_available(void* read_addr, void* write_addr, uint32_t buff_size)
{
  if( write_addr > read_addr ) return ((uint32_t) write_addr - (uint32_t) read_addr);
  else return buff_size - ((uint32_t) read_addr - (uint32_t) write_addr);
}


void init_buffer()
{
  trans_page = mt_init(buffer_length * 4); 
  con_blocks = mt_init(buffer_length * 61);
}

void stop_dma_and_exit()
{
  dma_reg[DMA_CS | DMA_channel << 8] = 0;    // stop dma 
  exit(1);
}

void set_sigaction()
{
  for (int i = 0; i < 64; i++) 
    { //catch all signals (like ctrl+c, ctrl+z, ...) to ensure DMA is disabled
      struct sigaction sa;
      memset(&sa, 0, sizeof(sa));
      sa.sa_handler = stop_dma_and_exit;
      sigaction(i, &sa, NULL);
    }
}


void* signal_processing(void* arg)
{
  uint32_t curr_pointer = 0, prev_tick = 0, first_change = 1, curr_channel = 0;
  memset(output_buffer, 0, 40000*8);
  int z = 0;
  int i;
  
  /*  struct timeval curr_freq_tick;
  struct timeval prev_freq_tick;
  if(gettimeofday(&prev_freq_tick, NULL) != 0) {printf("Error with getting time\n");}*/

  uint32_t curr_signal = 0, last_signal = 1488;
  uint64_t curr_tick, delta_time = 0;
  //sighandler

  z = 0;

  for(;;){
    int j;
    void* x;

    /*    if(gettimeofday(&curr_freq_tick, NULL) != 0) {printf("Error with getting time\n");}
    time_queue.push((uint64_t)(curr_freq_tick.tv_sec * 1000000 + curr_freq_tick.tv_usec - prev_freq_tick.tv_sec * 1000000 - prev_freq_tick.tv_usec));
    prev_freq_tick = curr_freq_tick;*/

    
    dma_cb_t* ad = (dma_cb_t*) mt_get_virt_addr(con_blocks, (void*) dma_reg[DMA_CONBLK_AD | DMA_channel << 8]);
    for(j = 1; j >= -1; j--){
      x = mt_get_virt_addr(trans_page, (void*) (ad + j)->dst);
      if(x != NULL) {
	break;}
    }
    uint32_t counter = (bytes_available(curr_pointer, mt_get_pointer_from_virt(trans_page, (void*)x), trans_page->page_count * PAGE_SIZE) & 0xFFFFFF80) >> 2;
    for(;counter > 10;counter--){
      if (curr_pointer %  (32*4) == 0){
	curr_tick = *((uint64_t*) mt_get_virt_from_pointer(trans_page, curr_pointer));
	curr_pointer+=8;
	counter-=2;
      }
      curr_signal = *((uint32_t*) mt_get_virt_from_pointer(trans_page, curr_pointer)) & 0x10 ? 1 : 0;
      if(last_signal == 1488) {last_signal = curr_signal; prev_tick = curr_tick;}
      if(curr_signal != last_signal)
	  {
	    delta_time = curr_tick - prev_tick;
	    prev_tick = curr_tick;
	    output_queue.push(delta_time);
	    output_buffer[delta_time]++;
	  }
      last_signal = curr_signal;
      curr_pointer+=4;
      if(curr_pointer >= trans_page->page_count*PAGE_SIZE)
	{
	  curr_pointer = 0;
	}
      curr_tick+=1000/sample_freq;
    }
    udelay(proc_delay);
  }
}

void* output_thread(void* arg)
{
  int i, j;
  int curr_count;
  while(1)
    {
      while(!output_queue.empty())
	{
	  uint64_t current;
	  current = output_queue.front();
	  output_queue.pop();
	  i++;
	  if(i == 1000)
	    {
	      for(j = 0; j < 40000; j++){
		if(output_buffer[j] != 0)
		  printf("value %u count %u\n", j, output_buffer[j]);
		i = 0;
	      }
	      
	    }

	}
      udelay(200000);
    }
}

void* time_thread(void* arg)
{
  int i, j;
  int curr_count;
  while(1)
    {
      while(!time_queue.empty())
	{
	  uint64_t current;
	  current = time_queue.front();
	  time_queue.pop();
	  printf("%llu\n", current);
	  
	}
      udelay(200000);
    }
}


int
main(int argc, char **argv)
{
  mlockall(MCL_CURRENT|MCL_FUTURE);
  int i, opt=0;
  int fr = 1;
  
  while ( (opt = getopt(argc,argv,"l:d:p:f:c:")) != -1){
    switch (opt){
    case 'l': 
      if((atoi(optarg) > 0) && (atoi(optarg) < 1000)) 
	buffer_length = atoi(optarg);
      else
	printf("Bad buffer length option. Using default value (5) .");
      break;
    case 'd': 
      proc_delay = atoi(optarg);
      break;
    case 'p':
      if((atoi(optarg) > 0) && (atoi(optarg) < 99)) 
	proc_priority = atoi(optarg);
      else
	printf("Bad priority value. Should be between 0 and 99. Using default value (99).");
      break;
    case 'f':
      fr = atoi(optarg);
      if((1000 % fr) != 0)
	printf("Bad frequency. Should divide 1000. Using default value (1000 KHz).");
      else 
	sample_freq = fr;
      break;
    case 'c':
      if((atoi(optarg) >=0) && (atoi(optarg) < 15))
	DMA_channel = atoi(optarg);
      else
	printf("Bad channel num. Should be between 0 and 14. Using default channel(5).");
      //      if(DMA_channel == 15
    };
  };
  
  dma_reg = map_peripheral(DMA_BASE, DMA_LEN);    
  pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
  pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
  clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
  gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);
  for (i = 0; i < NUM_GPIOS; i++) {
    gpio_set(gpio_list[i], 0);
    gpio_set_mode(gpio_list[i], GPIO_MODE_IN);
  }

  set_sigaction();
  init_buffer(); 
  init_ctrl_data(trans_page, con_blocks);
  init_hardware((uint32_t) *con_blocks->phys_pages | 0x40000000);

  udelay(300000);

  pthread_attr_t thread_attr, thread_attr2;
  struct sched_param param, param2;
  memset(&param, 0, sizeof(param));
  
  param.sched_priority = 12;
  sched_setscheduler(0, SCHED_FIFO, &param);
  
  pthread_attr_init(&thread_attr);
  param.sched_priority = proc_priority;
  (void)pthread_attr_setschedparam(&thread_attr, &param);
  pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);
  
  pthread_create(&_signal_handler, &thread_attr, &signal_processing , NULL);
  
  memset(&param2, 0, sizeof(param2));
  
  pthread_attr_init(&thread_attr2);
  param2.sched_priority = 1;
  (void)pthread_attr_setschedparam(&thread_attr2, &param2);
  pthread_attr_setschedpolicy(&thread_attr2, SCHED_FIFO);
  
  pthread_create(&_output_thread, &thread_attr2, &output_thread , NULL);

  memset(&param2, 0, sizeof(param2));
  
  pthread_attr_init(&thread_attr2);
  param2.sched_priority = 1;
  (void)pthread_attr_setschedparam(&thread_attr2, &param2);
  pthread_attr_setschedpolicy(&thread_attr2, SCHED_FIFO);
  
  //pthread_create(&_time_thread, &thread_attr2, &time_thread , NULL);

  while(1) sleep(100000);
  return 0;
}
