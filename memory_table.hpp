#ifndef MEMORY_TABLE_H
#define MEMORY_TABLE_H

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <assert.h>

#define DBG             1
#define PAGE_SIZE            4096

int errno;

typedef struct {
  void **virt_pages;
  void **phys_pages;
  uint32_t page_count;
} memory_table_t;

memory_table_t* mt_init(uint32_t page_count);

void* mt_get_phys_addr(volatile memory_table_t* memory_table, void* virt_addr);

void* mt_get_virt_addr(volatile memory_table_t* memory_table, void* phys_addr);

void* mt_get_virt_from_pointer(volatile memory_table_t* mt, uint32_t pointer);

void* mt_get_phys_from_pointer(volatile memory_table_t* mt, uint32_t pointer);

int32_t mt_get_pointer_from_virt(volatile memory_table_t* mt, void* virt_addr);
#endif
