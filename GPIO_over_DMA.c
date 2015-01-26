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

#define DEVFILE         "/dev/rpio-pwm"
#define NUM_GPIOS       (sizeof(gpio_list)/sizeof(gpio_list[0]))

// PERIOD_TIME_US is the pulse cycle time (period) per servo, in microseconds.
// Typically servos expect it to be 20,000us (20ms). If you are using
// 8 channels (gpios), this results in a 2.5ms timeslot per gpio channel. A
// servo output is set high at the start of its 2.5ms timeslot, and set low
// after the appropriate delay.
#define PERIOD_TIME_US       20000

// PULSE_WIDTH_INCR_US is the pulse width increment granularity, again in microseconds.
// Setting it too low will likely cause problems as the DMA controller will use too much
//memory bandwidth. 10us is a good value, though you might be ok setting it as low as 2us.
#define PULSE_WIDTH_INCR_US  10

// Timeslot per channel (delay between setting pulse information)
// With this delay it will arrive at the same channel after PERIOD_TIME.
#define CHANNEL_TIME_US      (PERIOD_TIME_US/NUM_GPIOS)

// CHANNEL_SAMPLES is the maximum number of PULSE_WIDTH_INCR_US that fit into one gpio
// channels timeslot. (eg. 250 for a 2500us timeslot with 10us PULSE_WIDTH_INCREMENT)
#define CHANNEL_SAMPLES      (CHANNEL_TIME_US/PULSE_WIDTH_INCR_US)

// Min and max channel width settings (used only for controlling user input)
#define CHANNEL_WIDTH_MIN    0
#define CHANNEL_WIDTH_MAX    (CHANNEL_SAMPLES - 1)

// Various
#define NUM_SAMPLES          (PERIOD_TIME_US/PULSE_WIDTH_INCR_US)
#define NUM_CBS              (NUM_SAMPLES*2)

#define PAGE_SIZE            4096
#define PAGE_SHIFT           12
#define NUM_PAGES            ((NUM_CBS * 32 + NUM_SAMPLES * 4 + \
			       PAGE_SIZE - 1) >> PAGE_SHIFT)

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
  void *start_virt_address;
  void **phys_pages;
  uint32_t page_count;
} memory_table_t;

typedef struct {
  void *virtPage, *physPage;
} page_table_t;

struct ctl {
  uint32_t sample[NUM_SAMPLES];
  dma_cb_t cb[NUM_CBS];
};

typedef struct {
  uint8_t *virtaddr;
  uint32_t physaddr;
} page_map_t;

page_map_t *page_map;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;
uint32_t* destination;
static page_table_t *page_tab;// = malloc(30*sizeof(page_table));

static int delay_hw = DELAY_VIA_PWM;

// Sets a GPIO to either GPIO_MODE_IN(=0) or GPIO_MODE_OUT(=1)

void* getPhys(void* virtAddr);

static memory_table_t* mt_init(uint32_t page_count)
{
  int i;
  memory_table_t* memory_table = malloc(sizeof(memory_table_t));
  memory_table->start_virt_address = valloc(page_count * PAGE_SIZE);
  memory_table->phys_pages = malloc(page_count * sizeof(void*));

  ((int*)memory_table->start_virt_address)[0] = 1;
  mlock(memory_table->start_virt_address, page_count * PAGE_SIZE);
  memset(memory_table->start_virt_address, 0, page_count * PAGE_SIZE); //zero-fill the page for convenience                                                                        
  //Magic to determine the physical address for this page:                                                                                     
  uint64_t pageInfo;
  int file = open("/proc/self/pagemap", 'r');////
  lseek(file, ((uint32_t)memory_table->start_virt_address)/PAGE_SIZE*8, SEEK_SET);
  for(i = 0; i < page_count; i++)
    {
       read(file, &pageInfo, 8); 
       memory_table->phys_pages[i] = (void*)(uint32_t)(pageInfo*PAGE_SIZE);
       printf("makeVirtPhysPage virtual to phys: %p -> %p\n", memory_table->start_virt_address + PAGE_SIZE * i, memory_table->phys_pages[i]);
    }
  close(file);
  return memory_table;
}

static void* mt_get_phys_addr(memory_table_t* memory_table, void* virt_addr)
{
  //Check does the address belong to this table
  if((virt_addr < memory_table->start_virt_address) || (virt_addr > memory_table->start_virt_address + PAGE_SIZE * memory_table->page_count / sizeof(void*))) return NULL;
  
  return (void*) ((uint32_t)memory_table->phys_pages[((uint32_t) virt_addr - (uint32_t) memory_table->start_virt_address)/PAGE_SIZE] + (uint32_t) virt_addr % PAGE_SIZE);
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

static void
init_ctrl_data(uint32_t dest, memory_table_t* con_blocks)
{
  uint32_t phys_fifo_addr;
  uint32_t phys_gpclr0 = 0x7e200000 + 0x28;
  int servo, i, j = 0;
  dma_cb_t* cbp = (dma_cb_t*) con_blocks->start_virt_address;
  // uint32_t cb_start = (uint32_t) cbp;
  if (delay_hw == DELAY_VIA_PWM)
    phys_fifo_addr = (PWM_BASE | 0x7e000000) + 0x18;
  else
    phys_fifo_addr = (PCM_BASE | 0x7e000000) + 0x04;

  if (delay_hw == DELAY_VIA_PWM)
    cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
  else
    cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
  cbp->src = (uint32_t)0x7E003004;    // Any data will do
  cbp->dst = phys_fifo_addr;
  if (delay_hw == DELAY_VIA_PWM)
    cbp->length = 64;
  else
    cbp->length = 512;
  cbp->stride = 0;
  cbp->next = mt_get_phys_addr(con_blocks, cbp + 1);//cb + sizeof(dma_cb_t);
  cbp++;

  for (i = 0; i < 1024; i++) {
    cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
    cbp->src = (uint32_t)0x7E003004;//mem_virt_to_phys(ctl->sample + i);
    cbp->dst = dest;
    cbp->length = 4;
    cbp->stride = 0;
    //    if(cbp + 1 >= cb_start + PAGE_SIZE){cbp->next =page_tab[++j].physPage;  cbp = page_tab[j].virtPage; cb_start = (uint32_t) cbp;}
    cbp->next = mt_get_phys_addr(con_blocks, cbp + 1); 
    cbp++;//cb + sizeof(dma_cb_t);
    dest+=4;
    // Delay
    if (delay_hw == DELAY_VIA_PWM)
      cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
    else
      cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
    cbp->src = (uint32_t)0x7E003004;    // Any data will do
    cbp->dst = phys_fifo_addr;
    cbp->length = 4;
    cbp->stride = 0;
    cbp->next = mt_get_phys_addr(con_blocks, cbp + 1); 
    //    if(cbp + 1 >= cb_start + PAGE_SIZE){cbp-> next =page_tab[++j].physPage;  cbp = page_tab[j].virtPage; cb_start = (uint32_t) cbp;}
    //  cbp->next = getPhys(cbp + 1); cbp++;}//cb + sizeof(dma_cb_t);
    cbp++;
  }
  cbp--;
  cbp->next = 0; //mem_virt_to_phys(ctl->cb);
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
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (50<<12);    // set pwm div to 50, giving 10MHz
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000016;        // Source=PLLD and enable
    udelay(10);
    pwm_reg[PWM_RNG1] = PULSE_WIDTH_INCR_US * 10;
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
    pcm_reg[PCM_MODE_A] = (PULSE_WIDTH_INCR_US * 10 - 1) << 10;
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
}

// Endless loop to read the FIFO DEVFILE and set the servos according

void makeVirtPhysPage(void** virtAddr, void** physAddr) {
  *virtAddr = valloc(PAGE_SIZE); //allocate one page of RAM
  //force page into RAM and then lock it there:                                                                                                
  ((int*)*virtAddr)[0] = 1;
  mlock(*virtAddr, PAGE_SIZE);
  memset(*virtAddr, 0, PAGE_SIZE); //zero-fill the page for convenience                                                                        

  //Magic to determine the physical address for this page:                                                                                     
  uint64_t pageInfo;
  int file = open("/proc/self/pagemap", 'r');
  lseek(file, ((uint32_t)*virtAddr)/PAGE_SIZE*8, SEEK_SET);
  read(file, &pageInfo, 8);

  *physAddr = (void*)(uint32_t)(pageInfo*PAGE_SIZE);
  printf("makeVirtPhysPage virtual to phys: %p -> %p\n", *virtAddr, *physAddr);
}


void makeVirtPhysPages(uint32_t numPages)
{
  uint32_t i = 0;
  for(i = 0; i < numPages; i++)
    {
       makeVirtPhysPage(&page_tab[i].virtPage, &page_tab[i].physPage);
    }
}

void* getPhys(void* virtAddr)
{
  void* physAddr = (void*) -1;
  uint32_t i = 0;
  for(i = 0; i < 16; i++)
    {
      if(((int)virtAddr)/PAGE_SIZE == ((int)page_tab[i].virtPage)/PAGE_SIZE) physAddr = (void*) ((int)page_tab[i].physPage + ((int)virtAddr)%PAGE_SIZE);
    }
  return physAddr;
}


int
main(int argc, char **argv)
{
  int i;
  //  memory_table_t* memory_table = memory_table_init(10);
  destination = (uint32_t*) malloc(PAGE_SIZE);
  page_tab = malloc(30*sizeof(page_table_t));
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

  void *virtAd, *physAd;
  makeVirtPhysPage(&virtAd, &physAd);
  //  memory_table_t* transfer_page = mt_init(1);
  memory_table_t* con_blocks = mt_init(16);
  uint32_t dest = (uint32_t) physAd;
  destination = (uint32_t*) virtAd;
  init_ctrl_data(dest, con_blocks);
  udelay(100);
  init_hardware((uint32_t) *con_blocks->phys_pages);

  for(i = 0; i < 500; i++) printf("%u\n", *(destination + i));
  udelay(100);
  return 0;
}
