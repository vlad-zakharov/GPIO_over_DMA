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
  uint32_t curr_counter; //max_counter == page_count * PAGE_SIZE / 8
} memory_table_t;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;
static volatile uint32_t* destination;
static volatile memory_table_t* trans_page;// = mt_init(5); 
static volatile memory_table_t* con_blocks;


static int delay_hw = DELAY_VIA_PWM;

// Sets a GPIO to either GPIO_MODE_IN(=0) or GPIO_MODE_OUT(=1)

static memory_table_t* mt_init(uint32_t page_count, int file, int fdMem)
{
  int i;

  /*  if ((fdMem = open("/dev/mem", O_RDWR | O_SYNC) ) < 0)
    {
      fprintf(stderr,
	            "\n" \
	            "+---------------------------------------------------------+\n" \
	            "|Sorry, you don't have permission to run this program.    |\n" \
	            "|Try running as root, e.g. precede the command with sudo. |\n" \
	      "+---------------------------------------------------------+\n\n");
      exit(-1);
      }*/
 

  memory_table_t* memory_table = malloc(sizeof(memory_table_t));
  memory_table->virt_pages = malloc(page_count * sizeof(void*));
  memory_table->phys_pages = malloc(page_count * sizeof(void*));
  memory_table->page_count = page_count;
                                     
  //Magic to determine the physical address for this page:                                                                                     
  uint64_t pageInfo;
  //  int file = open("/proc/self/pagemap", 'r');////
  void* shlyapa = mmap(0, page_count*PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
  for(i = 0; i < page_count; i++)
    {
      memory_table->virt_pages[i]  =  mmap(0, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
      //printf("virt pages %p\n", memory_table->virt_pages[i]);
      //memory_table->virt_pages[i] = valloc(PAGE_SIZE);
      //mlock(memory_table->virt_pages[i], PAGE_SIZE);

      //memset(memory_table->virt_pages[i], 0, PAGE_SIZE); //zero-fill the page for convenience                                   
      }
  lseek(file, ((uint32_t)shlyapa)/PAGE_SIZE*8, SEEK_SET);
  for(i = 0; i < page_count; i++)
    {
      //memory_table->virt_pages[i]  =  mmap(0, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
      //      memory_table->virt_pages[i] = valloc(PAGE_SIZE);
      //mlock(memory_table->virt_pages[i], PAGE_SIZE);


      read(file, &pageInfo, 8); 
      memory_table->phys_pages[i] = (void*)(uint32_t)(pageInfo*PAGE_SIZE);
      //munmap(memory_table->virt_pages[i], PAGE_SIZE);
      //     memset(memory_table->virt_pages[i], 0, PAGE_SIZE); //zero-fill the page for convenience                                   
    }


  for(i = 0; i < page_count; i++)
    {
      munmap(memory_table->virt_pages[i], PAGE_SIZE);
      memory_table->virt_pages[i]  =  mmap(memory_table->virt_pages[i], PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED|MAP_NORESERVE|MAP_LOCKED,fdMem, (void*)((uint32_t)memory_table->phys_pages[i] | 0x40000000));
      //mlock(memory_table->virt_pages[i], PAGE_SIZE);
      //       memory_table->start_virt_address = (void*)( (uint32_t)memory_table->start_virt_address + PAGE_SIZE);
    }
  //  memory_table->start_virt_address = (void*)( (uint32_t)memory_table->start_virt_address - page_count*PAGE_SIZE);
  //close(file);
  printf("asdfasdfasdf\n");
  //  ((int*)memory_table->start_virt_address)[0] = 1;
  memory_table->curr_counter = 0;
  //close(fdMem);
  for(i = 0; i < page_count; i++)
    {
      printf("virt %p phys %p\n", memory_table->virt_pages[i], memory_table->phys_pages[i]);
    }
  return memory_table;

}

static void* mt_get_phys_addr(volatile memory_table_t* memory_table, void* virt_addr)
{
  //Check does the address belong to this table
  /*if(((uint32_t)virt_addr < (uint32_t)memory_table->start_virt_address) || ((uint32_t)virt_addr > (uint32_t)memory_table->start_virt_address + PAGE_SIZE * memory_table->page_count)) return NULL;
  
    return (void*) (((uint32_t)memory_table->phys_pages[((uint32_t) virt_addr - (uint32_t) memory_table->start_virt_address)/PAGE_SIZE] + (uint32_t) virt_addr % PAGE_SIZE) | 0x40000000)*/
  int i;
  //  phys_addr = (void*)((uint32_t) phys_addr & 0xbfffffff);
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
  //  printf("phys addr %p\n", phys_addr);
  int i;
  phys_addr = (void*)((uint32_t) phys_addr & 0xbfffffff);
  for(i = 0; i < memory_table->page_count; i++)
    {
      //  printf("mem table ph addr %p page count %u\n", memory_table->phys_pages[i], memory_table->page_count);
      if ((uint32_t) memory_table->phys_pages[i] == (((uint32_t) phys_addr) & 0xFFFFF000)){
	return (void*) ((uint32_t) memory_table->virt_pages[i] + ((uint32_t) phys_addr & 0xFFF));
      }
    }
  return NULL;
}

static void* mt_next(volatile memory_table_t* memory_table, uint32_t count)
{

  if (memory_table->curr_counter < (memory_table->page_count * PAGE_SIZE) / 4)
    {
      memory_table->curr_counter+=(count/4);
      return memory_table->virt_pages[(memory_table->curr_counter - count/4) / 1024] + (((memory_table->curr_counter - count/4) % 1024)*4);

    }
  else return NULL;
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


//static void* mt_type_inc(volatile memory_table_t*, void* pointer

static void mt_restart(volatile memory_table_t* memory_table)
{
  memory_table->curr_counter = 0;
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

// 
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


/*
static uint32_t* get_buf_addr(uint32_t *address, void* buf_adress, uint32_t buf_page_count)
{
  return (uint32_t) address >= (uint32_t) buf_address + PAGE_SIZE * buf_page_count) ? (uint32_t*) buf_adress + (uint32_t) adress & 0x1111111000 : adress;
  }*/

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
 
  if (delay_hw == DELAY_VIA_PWM)
    phys_fifo_addr = (PWM_BASE | 0x7e000000) + 0x18;
  else
    phys_fifo_addr = (PCM_BASE | 0x7e000000) + 0x04;
  
  //  cbp = mt_next(con_blocks, sizeof(dma_cb_t));
  //  dest = mt_next(mem_table, sizeof(uint64_t

  //Init dma control blocks. For 960 i it is created 1024 control blocks (it is 
  for (i = 0; i < 19200; i++) 
    {
      //printf("i %d\n", i);
      //Transfer timer every 25th sample
      if(i % 30 == 0){
	//dest = mt_next(mem_table, sizeof(uint64_t));
	//	printf("dest dest %p and phys dest %p\n", dest, mt_get_phys_addr(mem_table, dest));
	//if(dest == NULL)
	// {
	//   mt_restart(mem_table);
	//   dest = mt_next(mem_table, sizeof(uint64_t));
	//}
	cbp_curr = (dma_cb_t*)mt_get_virt_from_pointer(con_blocks, (uint32_t) cbp);

	init_dma_cb(&cbp_curr, DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_INC | DMA_SRC_INC, TIMER_BASE, (uint32_t) mt_get_phys_from_pointer(mem_table, dest), 8, 0, (uint32_t) mt_get_phys_from_pointer(con_blocks, (uint32_t)(cbp + 1)));
	dest += 8;
	cbp++;
      } 
      // Transfer GPIO
      cbp_curr = (dma_cb_t*)mt_get_virt_from_pointer(con_blocks, (uint32_t) cbp);
      init_dma_cb(&cbp_curr, DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP, GPIO_LEV0_ADDR, (uint32_t) mt_get_phys_from_pointer(mem_table, dest), 4, 0, (uint32_t)  mt_get_phys_from_pointer(con_blocks, (uint32_t)(cbp + 1)));
      //      printf("virt_con_next %#x, phys_con_next %#x, virt_dest %#x, phys_dest %#x\n", cbp + 1, cbp->next, dest, cbp->dst);
      dest += 4;
      cbp++;
      // Delay
      cbp_curr = (dma_cb_t*)mt_get_virt_from_pointer(con_blocks, (uint32_t) cbp);
      if (delay_hw == DELAY_VIA_PWM)
	init_dma_cb(&cbp_curr, DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5), TIMER_BASE, phys_fifo_addr, 4, 0, (uint32_t)  mt_get_phys_from_pointer(con_blocks, (uint32_t)(cbp + 1)));
      else
	init_dma_cb(&cbp_curr, DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2), TIMER_BASE, phys_fifo_addr, 4, 0, (uint32_t)  mt_get_phys_from_pointer(con_blocks, (uint32_t)(cbp + 1)));
      cbp++;
    }

  cbp--;
  ((dma_cb_t*)mt_get_virt_from_pointer(con_blocks, (uint32_t)cbp))->next = (uint32_t) mt_get_phys_addr(con_blocks, con_blocks->virt_pages[0]);
}

// Initialize PWM (or PCM) and DMA
static void
init_hardware(uint32_t physCb)
{
  if (delay_hw == DELAY_VIA_PWM) {
    // Initialise PWM
    pwm_reg[PWM_CTL] = 0;
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000006;        // Source=PLLD (500MHz)
    udelay(10);
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (5000<<12);    // set pwm div to 50, giving 10MHz
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000016;        // Source=PLLD and enable
    udelay(10);
    pwm_reg[PWM_RNG1] = 100;
    udelay(10);
    pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_CLRF;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
  } else {
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
  }
  if (delay_hw == DELAY_VIA_PCM) {
    pcm_reg[PCM_CS_A] |= 1<<2;            // Enable Tx
    udelay(100);
  }
  // Initialise the DMA
  dma_reg[DMA_CS] = DMA_RESET;
  udelay(10);
  dma_reg[DMA_CS] = DMA_INT | DMA_END;
  dma_reg[DMA_CONBLK_AD] = physCb;
  dma_reg[DMA_DEBUG] = 7; // clear debug error flags
  dma_reg[DMA_CS] = 0x10880001;    // go, mid priority, wait for outstanding writes
  printf("dma reg %p\n", dma_reg[DMA_CONBLK_AD]);
}

// Endless loop to read the FIFO DEVFILE and set the servos according

uint32_t bytes_available(void* read_addr, void* write_addr, uint32_t buff_size)
{
  //  printf("read_addr %p write_addr %p\n", read_addr, write_addr);
  if( write_addr > read_addr ) return ((uint32_t) write_addr - (uint32_t) read_addr);
  else return buff_size - ((uint32_t) read_addr - (uint32_t) write_addr);
}


void init_buffer()
{
  int i, fdMem;

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

  int file = open("/proc/self/pagemap", 'r');////

  trans_page = mt_init(20, file, fdMem); 
  close(file);
  close(fdMem);

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
    file = open("/proc/self/pagemap", 'r');
  con_blocks = mt_init(305, file, fdMem);
  close(file);
  close(fdMem);
}

int
main(int argc, char **argv)
{
  //  sleep(2)
  mlockall(MCL_CURRENT|MCL_FUTURE);
  //sleep(60);
  int i;
  //  memory_table_t* memory_table = memory_table_init(10);
  destination = (uint32_t*) malloc(PAGE_SIZE);
  // Very crude...
  if (argc == 2 && !strcmp(argv[1], "--pcm"))
    delay_hw = DELAY_VIA_PCM;
  dma_reg = map_peripheral(DMA_BASE, DMA_LEN);

  pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
  pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
  clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
  gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);
  for (i = 0; i < NUM_GPIOS; i++) {
    gpio_set(gpio_list[i], 0);
    gpio_set_mode(gpio_list[i], GPIO_MODE_IN);
  }



  uint64_t* time1 = malloc(1000*sizeof(uint64_t));
  uint64_t* time_p = malloc(1000*sizeof(uint64_t));
  void** xx = malloc(1000*sizeof(void*));
  uint32_t*  coun = malloc(1000*sizeof(uint32_t));
  uint32_t* my_buffer = malloc(50*PAGE_SIZE);
  int z = 0;

  //  volatile memory_table_t* trans_page = mt_init(5); 
  //  volatile memory_table_t* con_blocks = mt_init(305);
  //  printf("con_blocks count %u\n", con_blocks->page_count);
  init_buffer(); 

  //  printf("%p\n", *con_blocks->phys_pages);//HERE IS NILL

  //  printf("new method tests. Pointer is %d, start adress is %p and %p, res is %p and %p\n", 4224, trans_page->virt_pages[0], trans_page->virt_pages[1], mt_get_virt_from_pointer(trans_page, 128), mt_get_virt_from_pointer(trans_page, 4224));

  uint32_t curr_pointer = 0;

  init_ctrl_data(trans_page, con_blocks);

  init_hardware((uint32_t) *con_blocks->phys_pages | 0x40000000);

  //  for(z = 0; z < 500; z++) printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);
  /* printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);
  printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);
  printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);
  printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);
  printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);
  printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);
  printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);*/

  udelay(300000);
  //  sleep(1);
  printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);

  uint32_t curr_signal = 0, last_signal = 0;
  uint64_t curr_time;
  //sighandler

  /*dma_reg[DMA_CS] &= 0xFFFFFFFE;    // stop
  for(i = 0; i < 5 * 1024; i++){
    if(i%32 == 0)
      printf("time:%llu virt_addr:p\n", *((uint64_t*) trans_page->start_virt_address + i/2), (uint64_t*) trans_page->start_virt_address + i/2);
      }
      exit(0);*/


  z = 0;

  for(;;){
    int j;
    void* x;
    //uint32_t curr_signal, last_signal;
    //uint64_t curr_time;
    //    printf("ad%p\n", dma_reg[0]);
    //    printf("dma conblk %p\n", dma_reg[DMA_CONBLK_AD]);
    dma_cb_t* ad = (dma_cb_t*) mt_get_virt_addr(con_blocks, (void*) dma_reg[DMA_CONBLK_AD]);
    //printf("ad %p\n", ad);
    for(j = 1; j >= -1; j--){
      //x = mt_get_virt_addr(con_blocks, (void*) dma_reg[DMA_CONBLK_AD]);
      //          printf("dst %p\n", (void*) (ad + j)->dst);
      x = mt_get_virt_addr(trans_page, (void*) (ad + j)->dst);//What i
      //      if(x == ((PCM_BASE | 0x7e000000) + 0x04))
      if(x != NULL) {
	break;}
    }
    //printf("x %p\n", x);
    //    if(x == NULL) printf("null\n");
    //printf("x %p, curr_p %p ", x, curr_pointer);
    //accessing memory
    //cycle, begin - curr_pointer, end - write_mem 


    // THE PROBLEM IS THAT WE CAN JUMP OVER X, AND IT IS BAD. 1 SOLUTION - COMPARE NOT ONLY WITH CURR_POINTER BUT ALSO WITH CURR_POINTER -1 -2
    //THE SECOND WAY IS TO CHANGE THIS ALGORYTHM. THE NEW ONE IS 1 BYTE 1 ITERATION. Detect: printf x at the end at curr_pointer at the end.
    //Maybe 
    //If less than 3 bytes stop!!!!! If %32 then read ho?
    //dma_reg[DMA_CS] &= 0xFFFFFFFE;
    //uint32_t b_a = bytes_available(curr_pointer, x, trans_page->page_count * PAGE_SIZE);
    //может и не совпадать
    uint32_t counter;// = (bytes_available(mt_get_virt_from_pointer(trans_page, curr_pointer), x, trans_page->page_count * PAGE_SIZE) & 0xFFFFFF80) >> 2;
    //printf("%x\n", counter);
    //printf("counter %#X\n", counter);      //main cycle
    for(counter = 2000;counter > 10;counter--){//change this rule.
      //printf("%p\n", curr_pointer);
      if (curr_pointer %  (32*4) == 0){
	//THIS IS TIME
	//curr_time = *(curr_pointer);
	
	//memcpy(&curr_time, curr_pointer, 8);
	curr_time = *((uint64_t*) mt_get_virt_from_pointer(trans_page, curr_pointer));
	//printf("time time %u pointer %p\n", curr_time, curr_pointer);
	//	curr_time = *(uint64_t*) curr_pointer;
	

	/*if((z > 0) &&(curr_time < time1[z-1])) {
	    dma_reg[DMA_CS] &= 0xFFFFFFFE;    // stop
	      ad = (dma_cb_t*) mt_get_virt_addr(con_blocks, (void*) dma_reg[DMA_CONBLK_AD]);
	        for(j = 0; j >= -1; j--){
		    //x = mt_get_virt_addr(con_blocks, (void*) dma_reg[DMA_CONBLK_AD]);
		        x = (void*) mt_get_virt_addr(trans_page, (void*) (ad + j)->dst);//What i
			//      if(x == ((PCM_BASE | 0x7e000000) + 0x04))
			if(x != NULL) {
	break;}
      }
		printf("time %u curr_pointer %p x %p \n", curr_time, curr_pointer, x);
		usleep(1000);
		for(i = 0; i < 5 * 1024; i++){
	if(i%32 == 0)
	  printf("time:%llu virt_addr:%p\n", *((uint64_t*) trans_page->start_virt_address + i/2), (uint64_t*) trans_page->start_virt_address + i/2);
      }
		exit(0);
      }*/
	    
	//memcpy(my_buffer, trans_page->start_virt_address, 5*PAGE_SIZE);
	time_p[z] = curr_pointer;
	time1[z] = curr_time;
	//time1[z] = curr_time;
	//xx[z] = x;
	//coun[z] = counter;
	//memcpy~~
	//coun[z] = counter;
	z++;
	/*for(i = 0; i < 5 * 1024; i++)
	    if(i % 32 == 0){
	        printf("%d my_beff\n", *(my_buffer + i));
		 }
		 //my_buffer += 5*1024;
		 if(z == 10) exit(0);*/
	//printf("time %x at adress %p when x is %p\n", curr_time, curr_pointer, x);
	/*dma_reg[DMA_CS] &= 0xFFFFFFFE;    // stop    
	  printf("time %llu at adress %p when x is %p\n", *((uint64_t*)&curr_time), curr_pointer, x);
	  for(i = 0; i < 5 * 1024; i++){
	    if(i%32 == 0)
	        printf("time:%llu virt_addr:%p\n", *((uint64_t*) trans_page->start_virt_address + i/2), (uint64_t*) trans_page->start_virt_address + i/2);
		}
		exit(0);*/
	if(z == 1000 ) 
	  {
	    z = 0;
	    for (i = 0; i < 1000; i++) printf("time %llu pointer %p\n", time1[i], time_p[i]);
	}
	curr_pointer+=8;
	counter-=2;
      }
      //printf("c %#x l %#x\n", curr_signal, last_signal);
      curr_signal = *((uint32_t*) mt_get_virt_from_pointer(trans_page, curr_pointer)) & 0x10 ? 1 : 0;
      if(curr_signal != last_signal){
	//printf("%d %u\n", curr_signal, curr_time/* + ((((uint32_t) curr_pointer) - (uint32_t) trans_page->start_virt_address) %  (32*4))/4*/);
	last_signal = curr_signal;
      }
      else last_signal = curr_signal;
      curr_pointer+=4;
      if(curr_pointer >= trans_page->page_count*PAGE_SIZE) curr_pointer = 0;
      //      printf("x: %p p: %p\n", x, curr_pointer);
    }
    //    dma_reg[DMA_CS] |= 0x1; Переход в цикле! Переход в цикле! Переход в ЦИКЛЕЕЕЕЕ
    usleep(1000);
  }
  /*void* x;
  for(i = 0; i < 3; i++){
    x = mt_get_virt_addr(trans_page, (void*) (((dma_cb_t*) mt_get_virt_addr(con_blocks, (void*) dma_reg[DMA_CONBLK_AD])) + i)->dst);
    if(x != NULL) {
    break;}
  }
  if(x == NULL) printf ("@#$@#$@#$@#$\n");
  printf("dma dst: %p -> %d\n", x, *((int*) x));*/
  return 0;
}
