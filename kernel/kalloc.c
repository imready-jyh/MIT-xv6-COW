// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

void freerange(void *pa_start, void *pa_end);

int getrefcnt(void *);
void increfcnt(void *);
int decrefcnt(void *);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;

struct {
  struct spinlock lock;
  int count[PHYSTOP / PGSIZE];  // Reference count for each physical page
} refcnt;

void
kinit()
{
  initlock(&kmem.lock, "kmem");
  initlock(&refcnt.lock, "refcnt");
  freerange(end, (void*)PHYSTOP);
}

// Get reference count for a physical address
int
getrefcnt(void *pa)
{
  return refcnt.count[(uint64)pa / PGSIZE];
}

// Increment reference count
void
increfcnt(void *pa)
{
  if((uint64)pa % PGSIZE != 0 || (uint64)pa >= PHYSTOP)
    panic("increfcnt");
  
  acquire(&refcnt.lock);
  refcnt.count[(uint64)pa / PGSIZE]++;
  release(&refcnt.lock);
}

// Decrement reference count and return new count
int
decrefcnt(void *pa)
{
  if((uint64)pa % PGSIZE != 0 || (uint64)pa >= PHYSTOP)
    panic("decrefcnt");
  
  acquire(&refcnt.lock);
  int cnt = --refcnt.count[(uint64)pa / PGSIZE];
  release(&refcnt.lock);
  return cnt;
}

void
freerange(void *pa_start, void *pa_end)
{
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start);
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE)
    kfree(p);
}

// Free the page of physical memory pointed at by pa,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(void *pa)
{
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  // Only free if reference count reaches 0
  if(decrefcnt(pa) > 0)
    return;

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  r = (struct run*)pa;

  acquire(&kmem.lock);
  r->next = kmem.freelist;
  kmem.freelist = r;
  release(&kmem.lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void *
kalloc(void)
{
  struct run *r;

  acquire(&kmem.lock);
  r = kmem.freelist;
  if(r)
    kmem.freelist = r->next;
  release(&kmem.lock);

  if(r) {
    memset((char*)r, 5, PGSIZE); // fill with junk
    refcnt.count[(uint64)r / PGSIZE] = 1;
  }
  return (void*)r;
}
