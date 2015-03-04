#include <pthread.h>
#include <stdio.h>
#include "PCA9685.h"
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

#define RT_PRIORITY

#define NUM_GPIOS       (sizeof(gpio_list)/sizeof(gpio_list[0]))

#define PAGE_SIZE            4096

// Memory Addresses
#define DMA_BASE        0x20007000
#define DMA_LEN         0x24
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

typedef struct {
  void **virt_pages;
  void **phys_pages;
  uint32_t page_count;
} memory_table_t;

unsigned int ppmSyncLength     = 4000;   // Length of PPM sync pause
unsigned int ppmChannelsNumber = 8;      // Number of channels packed in PPM


static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;
static volatile uint32_t* destination;
static volatile memory_table_t* trans_page;
static volatile memory_table_t* con_blocks;
pthread_t _signal_handler;


static int delay_hw = DELAY_VIA_PWM;

// Sets a GPIO to either GPIO_MODE_IN(=0) or GPIO_MODE_OUT(=1)

static memory_table_t* mt_init(uint32_t page_count)
{
  int i, fdMem, file;

  if ((fdMem = open("/dev/mem", O_RDWR | O_SYNC) ) < 0)
    {
      fprintf(stderr,
	      "\n" \
	      "+---------------------------------------------------------+\n" \
	      "|Sorry, you don't have permission to run this program.    |\n" \
	      "|Try running as root, e.g. precede the command with sudo. |\n" \
	      "+---------------------------------------------------------+\n\n");
      exit(-1);
    }

  if ((file = open("/proc/self/pagemap", 'r') ) < 0)
    {
      fprintf(stderr,
	      "\n" \
	      "+---------------------------------------------------------+\n" \
	      "|Sorry, you don't have permission to run this program.    |\n" \
	      "|Try running as root, e.g. precede the command with sudo. |\n" \
	      "+---------------------------------------------------------+\n\n");
      exit(-1);
    }

  memory_table_t* memory_table = malloc(sizeof(memory_table_t));
  memory_table->virt_pages = malloc(page_count * sizeof(void*));
  memory_table->phys_pages = malloc(page_count * sizeof(void*));
  memory_table->page_count = page_count;
                                     
  //Magic to determine the physical address for this page:                                                                                     
  uint64_t pageInfo;

  void* shlyapa = mmap(0, page_count*PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
  for(i = 0; i < page_count; i++)
    {
      memory_table->virt_pages[i]  =  mmap(0, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
    }
  lseek(file, ((uint32_t)shlyapa)/PAGE_SIZE*8, SEEK_SET);
  for(i = 0; i < page_count; i++)
    {
      read(file, &pageInfo, 8); 
      memory_table->phys_pages[i] = (void*)(uint32_t)(pageInfo*PAGE_SIZE);
    }


  for(i = 0; i < page_count; i++)
    {
      munmap(memory_table->virt_pages[i], PAGE_SIZE);
      memory_table->virt_pages[i]  =  mmap(memory_table->virt_pages[i], PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED|MAP_NORESERVE|MAP_LOCKED,fdMem, (void*)((uint32_t)memory_table->phys_pages[i] | 0x40000000));
    }
  close(file);
  close(fdMem);
  return memory_table;

}

static void* mt_get_phys_addr(volatile memory_table_t* memory_table, void* virt_addr)
{
  int i;
  for(i = 0; i < memory_table->page_count; i++)
    {
      if ((uint32_t) memory_table->virt_pages[i] == (((uint32_t) virt_addr) & 0xFFFFF000)){
	return (void*) (((uint32_t) memory_table->phys_pages[i] + ((uint32_t) virt_addr & 0xFFF)) | 0x40000000);
      }
    }
  return NULL;
}

static void* mt_get_virt_addr(volatile memory_table_t* memory_table, void* phys_addr)
{
  int i;
  phys_addr = (void*)((uint32_t) phys_addr & 0xbfffffff);
  for(i = 0; i < memory_table->page_count; i++)
    {
      if ((uint32_t) memory_table->phys_pages[i] == (((uint32_t) phys_addr) & 0xFFFFF000)){
	return (void*) ((uint32_t) memory_table->virt_pages[i] + ((uint32_t) phys_addr & 0xFFF));
      }
    }
  return NULL;
}

// This function returns virtual address with help of pointer, which is offset from the beginning of the buffer.
static void* mt_get_virt_from_pointer(volatile memory_table_t* mt, uint32_t pointer)
{
  if(pointer >= PAGE_SIZE * mt->page_count) return NULL;
  return mt->virt_pages[(uint32_t) pointer / 4096] + pointer % 4096;
}

static void* mt_get_phys_from_pointer(volatile memory_table_t* mt, uint32_t pointer)
{
  if(pointer >= PAGE_SIZE * mt->page_count) return NULL;
  return mt->phys_pages[(uint32_t) pointer / 4096] + pointer % 4096;
}

static int32_t mt_get_pointer_from_virt(volatile memory_table_t* mt, void* virt_addr)
{
  int i;
  //phys_addr = (void*)((uint32_t) phys_addr & 0xbfffffff);
  for(i = 0; i < mt->page_count; i++)
    {
      if ((uint32_t) mt->virt_pages[i] == (((uint32_t) virt_addr) & 0xFFFFF000))
	{
	  return (i*PAGE_SIZE + ((uint32_t) virt_addr & 0xFFF));
	}
    }
  return -1;
}



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
  for (i = 0; i < 19200; i++) 
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
  clk_reg[PCMCLK_DIV] = 0x5A000000 | (50<<12);    // Set pcm div to 50, giving 10MHz
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
  dma_reg[DMA_CS] = DMA_RESET;
  udelay(10);
  dma_reg[DMA_CS] = DMA_INT | DMA_END;
  dma_reg[DMA_CONBLK_AD] = physCb;
  dma_reg[DMA_DEBUG] = 7; // clear debug error flags
  dma_reg[DMA_CS] = 0x10880001;    // go, mid priority, wait for outstanding writes
  //printf("dma reg %p\n", dma_reg[DMA_CONBLK_AD]);
}

uint32_t bytes_available(void* read_addr, void* write_addr, uint32_t buff_size)
{
  if( write_addr > read_addr ) return ((uint32_t) write_addr - (uint32_t) read_addr);
  else return buff_size - ((uint32_t) read_addr - (uint32_t) write_addr);
}


void init_buffer()
{
  trans_page = mt_init(20); 
  con_blocks = mt_init(305);
}


void* signal_processing(void* arg)
{
  uint32_t curr_pointer = 0, prev_tick = 0, first_change = 1;
  uint64_t* time1 = malloc(2000*sizeof(uint64_t));
  uint32_t* time_p = malloc(2000*sizeof(uint64_t));
  void** xx = malloc(2000*sizeof(void*));
  uint32_t*  coun = malloc(2000*sizeof(uint32_t));
  uint32_t* my_buffer = malloc(50*PAGE_SIZE);
  int z = 0;
  int i;

  uint32_t curr_signal = 0, last_signal = 0, counter2 = 0;
  uint64_t curr_tick;
  //sighandler

  z = 0;

  for(;;){
    int j;
    void* x;

	
    dma_cb_t* ad = (dma_cb_t*) mt_get_virt_addr(con_blocks, (void*) dma_reg[DMA_CONBLK_AD]);
    for(j = 1; j >= -1; j--){
      x = mt_get_virt_addr(trans_page, (void*) (ad + j)->dst);
      if(x != NULL) {
	break;}
    }

    uint32_t counter = (bytes_available(curr_pointer, mt_get_pointer_from_virt(trans_page, (void*)x), trans_page->page_count * PAGE_SIZE) & 0xFFFFFF80) >> 2;
    for(;counter > 10;counter--){
      if (curr_pointer %  (32*4) == 0){
	curr_tick = *((uint64_t*) mt_get_virt_from_pointer(trans_page, curr_pointer));
	/*	time_p[z] = curr_pointer;
	time1[z] = curr_tick;
	coun[z] = counter;
	xx[z] = mt_get_pointer_from_virt(trans_page, (void*)x);
	z++;
	if(z == 2000 ) 
	  {
	    z = 0;
	    assert(time_p != NULL);

	    for (i = 0; i < 2000; i++)
	      {
		printf("time %llu pointer %p counter %u x %p\n", time_p[i], time1[i], coun[i], xx[i]);
	      }
	    printf("pointer %p\n", time_p);
	    }*/
	curr_pointer+=8;
	counter-=2;
      }
      curr_signal = *((uint32_t*) mt_get_virt_from_pointer(trans_page, curr_pointer)) & 0x10 ? 1 : 0;
      if(curr_signal != last_signal){
	//	printf("SIG CHANGED AT %llu\n", curr_tick);
	if(!first_change)
	  {
	    //printf
	    time1[counter2] = curr_tick - prev_tick;
	    counter2++;
	    prev_tick = curr_tick;
	  }
	else
	  {
	    first_change = 0;
	    prev_tick = curr_tick;
	  }
	  
	last_signal = curr_signal;
      }
      else last_signal = curr_signal;
      curr_pointer+=4;
      if(curr_pointer >= trans_page->page_count*PAGE_SIZE)
	{
	  //printf("%x\n", counter);
	  curr_pointer = 0;
	  //	      counter2 = 0;
	}
      curr_tick++;
    }
    usleep(5000);
    if(counter2 >= 200)
      {
	for(i = 0; i < counter2; i++)
	  {
	    printf("length %llu\n", time1[i]);
	  }
	counter2 = 0;
      }
  }
}

int
main(int argc, char **argv)
{
  mlockall(MCL_CURRENT|MCL_FUTURE);
  int i;

  dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
  pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
  pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
  clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
  gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);
  for (i = 0; i < NUM_GPIOS; i++) {
    gpio_set(gpio_list[i], 0);
    gpio_set_mode(gpio_list[i], GPIO_MODE_IN);
  }

  init_buffer(); 
  init_ctrl_data(trans_page, con_blocks);
  init_hardware((uint32_t) *con_blocks->phys_pages | 0x40000000);

  udelay(300000);

  pthread_attr_t thread_attr;
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  
  pthread_attr_init(&thread_attr);
  param.sched_priority = 99;
  (void)pthread_attr_setschedparam(&thread_attr, &param);
  pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);
  
  pthread_create(&_signal_handler, &thread_attr, &signal_processing , NULL);
  while(1);
  return 0;
}
