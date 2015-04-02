#include "memory_table.h"

using namespace std;

memory_table_t* mt_init(uint32_t page_count)
{
  int i, fdMem, file;
  uint64_t pageInfo;
  void* offset;

  memory_table_t* memory_table = (memory_table_t*)malloc(sizeof(memory_table_t));
  memory_table->virt_pages = (void**)malloc(page_count * sizeof(void*));
  memory_table->phys_pages = (void**)malloc(page_count * sizeof(void*));
  memory_table->page_count = page_count;


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

                                     
  //Magic to determine the physical address for this page:
  offset = mmap(0, page_count*PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
  lseek(file, ((uint32_t)offset)/PAGE_SIZE*8, SEEK_SET);
  for(i = 0; i < page_count; i++)
    {
      memory_table->virt_pages[i]  =  mmap(0, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,-1,0);
      read(file, &pageInfo, 8); 
      memory_table->phys_pages[i] = (void*)(uint32_t)(pageInfo*PAGE_SIZE);
    }


  for(i = 0; i < page_count; i++)
    {
      munmap(memory_table->virt_pages[i], PAGE_SIZE);
      memory_table->virt_pages[i]  = mmap(memory_table->virt_pages[i], PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED|MAP_NORESERVE|MAP_LOCKED,fdMem, ((uint32_t)memory_table->phys_pages[i] | 0xC0000000));
      printf("phys page %p virt page %p\n", memory_table->virt_pages[i], memory_table->phys_pages[i]);
    }
  close(file);
  close(fdMem);
  return memory_table;
}

void* mt_get_phys_addr(volatile memory_table_t* memory_table, void* virt_addr)
{
  int i;
  for(i = 0; i < memory_table->page_count; i++)
    {
      if ((uint32_t) memory_table->virt_pages[i] == (((uint32_t) virt_addr) & 0xFFFFF000)){
	return (void*) (((uint32_t) memory_table->phys_pages[i] + ((uint32_t) virt_addr & 0xFFF)) | 0xC0000000);
      }
    }
  return NULL;
}

void* mt_get_virt_addr(volatile memory_table_t* memory_table, void* phys_addr)
{
  int i;
  phys_addr = (void*)((uint32_t) phys_addr & 0x3fffffff);
  for(i = 0; i < memory_table->page_count; i++)
    {
      if ((uint32_t) memory_table->phys_pages[i] == (((uint32_t) phys_addr) & 0xFFFFF000)){
	return (void*) ((uint32_t) memory_table->virt_pages[i] + ((uint32_t) phys_addr & 0xFFF));
      }
    }
  return NULL;
}

// This function returns virtual address with help of pointer, which is offset from the beginning of the buffer.
void* mt_get_virt_from_pointer(volatile memory_table_t* mt, uint32_t pointer)
{
  if(pointer >= PAGE_SIZE * mt->page_count) return NULL;
  return (uint8_t*)mt->virt_pages[(uint32_t) pointer / 4096] + pointer % 4096;
}

void* mt_get_phys_from_pointer(volatile memory_table_t* mt, uint32_t pointer)
{
  if(pointer >= PAGE_SIZE * mt->page_count) return NULL;
  return (uint8_t*)mt->phys_pages[(uint32_t) pointer / 4096] + pointer % 4096;
}

uint32_t mt_get_pointer_from_virt(volatile memory_table_t* mt, void* virt_addr)
{
  int i;
  for(i = 0; i < mt->page_count; i++)
    {
      if ((uint32_t) mt->virt_pages[i] == (((uint32_t) virt_addr) & 0xFFFFF000))
	{
	  return (i*PAGE_SIZE + ((uint32_t) virt_addr & 0xFFF));
	}
    }
  return -1;
}
