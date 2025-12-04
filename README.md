# MIT-xv6-COW

# xv6 Copy-on-Write Fork Implementation Guide

## Overview
This guide implements Copy-on-Write (COW) fork for xv6, which defers memory copying until pages are actually written to, improving performance and memory efficiency.

## Step 1: Add COW Flag Definition

**File: `kernel/riscv.h`**

Add this definition with the other PTE flags (around line 344):
```c
#define PTE_COW (1L << 8)  // Copy-on-write flag (using RSW bit)
```

## Step 2: Add Reference Counting Infrastructure

**File: `kernel/kalloc.c`**

Add reference counting array and lock at the top (after the existing structures):
```c
struct {
  struct spinlock lock;
  int count[PHYSTOP / PGSIZE];  // Reference count for each physical page
} refcnt;
```

Modify `kinit()` to initialize the reference count lock:
```c
void
kinit()
{
  initlock(&kmem.lock, "kmem");
  initlock(&refcnt.lock, "refcnt");  // Add this line
  freerange(end, (void*)PHYSTOP);
}
```

Add helper functions for reference counting (add these before `kfree`):
```c
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
```

Modify `kalloc()` to set initial reference count to 1:
```c
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
    refcnt.count[(uint64)r / PGSIZE] = 1;  // Add this line
  }
  return (void*)r;
}
```

Modify `kfree()` to only free when reference count reaches 0:
```c
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
```

Add function declarations at the top of the file:
```c
int getrefcnt(void *);
void increfcnt(void *);
int decrefcnt(void *);
```

## Step 3: Modify uvmcopy for COW

**File: `kernel/vm.c`**

Replace the entire `uvmcopy()` function:
```c
int
uvmcopy(pagetable_t old, pagetable_t new, uint64 sz)
{
  pte_t *pte;
  uint64 pa, i;
  uint flags;

  for(i = 0; i < sz; i += PGSIZE){
    if((pte = walk(old, i, 0)) == 0)
      panic("uvmcopy: pte should exist");
    if((*pte & PTE_V) == 0)
      panic("uvmcopy: page not present");
    
    pa = PTE2PA(*pte);
    flags = PTE_FLAGS(*pte);
    
    // If page is writable, mark as COW and remove write permission
    if(flags & PTE_W) {
      flags = (flags & ~PTE_W) | PTE_COW;
      *pte = PA2PTE(pa) | flags;
    }
    
    // Map the same physical page in child with same flags
    if(mappages(new, i, PGSIZE, pa, flags) != 0){
      goto err;
    }
    
    // Increment reference count since page is now shared
    increfcnt((void*)pa);
  }
  return 0;

 err:
  uvmunmap(new, 0, i / PGSIZE, 1);
  return -1;
}
```

## Step 4: Implement COW Page Fault Handler

**File: `kernel/trap.c`**

Add this helper function before `usertrap()`:
```c
// Handle copy-on-write page fault
int
cowhandler(pagetable_t pagetable, uint64 va)
{
  if(va >= MAXVA)
    return -1;
  
  pte_t *pte = walk(pagetable, va, 0);
  if(pte == 0)
    return -1;
  if((*pte & PTE_V) == 0)
    return -1;
  if((*pte & PTE_U) == 0)
    return -1;
  
  // Check if it's a COW page
  if((*pte & PTE_COW) == 0)
    return -1;
  
  uint64 pa = PTE2PA(*pte);
  uint flags = PTE_FLAGS(*pte);
  
  // Allocate new page
  char *mem = kalloc();
  if(mem == 0)
    return -1;
  
  // Copy old page to new page
  memmove(mem, (char*)pa, PGSIZE);
  
  // Update PTE: remove COW flag, add write permission
  flags = (flags & ~PTE_COW) | PTE_W;
  *pte = PA2PTE((uint64)mem) | flags;
  
  // Decrement reference count of old page
  kfree((void*)pa);
  
  return 0;
}
```

Modify `usertrap()` to handle COW page faults. Find the section that handles exceptions and add COW handling for store page faults:
```c
void
usertrap(void)
{
  int which_dev = 0;

  if((r_sstatus() & SSTATUS_SPP) != 0)
    panic("usertrap: not from user mode");

  // send interrupts and exceptions to kerneltrap(),
  // since we're now in the kernel.
  w_stvec((uint64)kernelvec);

  struct proc *p = myproc();
  
  // save user program counter.
  p->trapframe->epc = r_sepc();
  
  if(r_scause() == 8){
    // system call
    if(killed(p))
      exit(-1);
    p->trapframe->epc += 4;
    intr_on();
    syscall();
  } else if((which_dev = devintr()) != 0){
    // ok
  } else if(r_scause() == 15) {  // Store page fault
    uint64 va = r_stval();
    // Try COW handler first
    if(cowhandler(p->pagetable, va) < 0) {
      // If COW handler fails, try vmfault for lazy allocation
      uint64 fault_result = vmfault(p->pagetable, va, 1);
      if(fault_result == 0) {
        // Both COW and vmfault failed - invalid access
        printf("usertrap(): unexpected scause 0x%lx pid=%d\n", r_scause(), p->pid);
        printf("            sepc=0x%lx stval=0x%lx\n", r_sepc(), r_stval());
        setkilled(p);
      }
    }
  } else {
    printf("usertrap(): unexpected scause 0x%lx pid=%d\n", r_scause(), p->pid);
    printf("            sepc=0x%lx stval=0x%lx\n", r_sepc(), r_stval());
    setkilled(p);
  }

  if(killed(p))
    kexit(-1);
  
  // give up the CPU if this is a timer interrupt.
  if(which_dev == 2)
    yield();
  
  prepare_return();
  // the user page table to switch to, for trampoline.S
  uint64 satp = MAKE_SATP(p->pagetable);
  // return to trampoline.S; satp value in a0.
  return satp;
}
```

## Step 5: Modify copyout for COW

**File: `kernel/vm.c`**

Modify the `copyout()` function to handle COW pages. Your version already has `vmfault()`, so integrate COW handling:

```c
int
copyout(pagetable_t pagetable, uint64 dstva, char *src, uint64 len)
{
  uint64 n, va0, pa0;
  pte_t *pte;
  
  while(len > 0){
    va0 = PGROUNDDOWN(dstva);
    if(va0 >= MAXVA)
      return -1;
  
    pa0 = walkaddr(pagetable, va0);
    if(pa0 == 0) {
      if((pa0 = vmfault(pagetable, va0, 0)) == 0) {
        return -1;
      }
    }
    
    pte = walk(pagetable, va0, 0);
    
    // Handle COW page before checking PTE_W
    if(*pte & PTE_COW) {
      if(cowhandler(pagetable, va0) < 0)
        return -1;
      pte = walk(pagetable, va0, 0);
      pa0 = PTE2PA(*pte);  // Update pa0 after COW handling
    }
    
    // forbid copyout over read-only user text pages.
    if((*pte & PTE_W) == 0)
      return -1;
      
    n = PGSIZE - (dstva - va0);
    if(n > len)
      n = len;
    memmove((void *)(pa0 + (dstva - va0)), src, n);
    
    len -= n;
    src += n;
    dstva = va0 + PGSIZE;
  }
  return 0;
}
```

**Note**: This version preserves your existing `vmfault()` call and adds COW handling before the write permission check.

## Step 6: Add Function Declarations

**File: `kernel/defs.h`**

Add these declarations in the appropriate sections:

Under the `// kalloc.c` section:
```c
void            increfcnt(void *);
int             decrefcnt(void *);
int             getrefcnt(void *);
```

Under the `// trap.c` section:
```c
int             cowhandler(pagetable_t, uint64);
```

## Testing

Compile and test your implementation:
```bash
$ make qemu
$ cowtest
simple: ok
simple: ok
three: ok
three: ok
three: ok
file: ok
forkfork: ok
ALL COW TESTS PASSED

$ usertests -q
...
ALL TESTS PASSED
```

## Key Concepts

1. **Reference Counting**: Each physical page tracks how many page tables reference it
2. **COW Flag**: Uses RSW bit in PTE to mark copy-on-write pages
3. **Lazy Copying**: Physical pages are only copied when written to
4. **Page Fault Handling**: Store page faults on COW pages trigger the copy operation
5. **Memory Efficiency**: Shared read-only pages remain shared until modified

## Common Pitfalls

- Forgetting to increment reference count in `uvmcopy`
- Not handling COW in `copyout` (kernel writes to user space)
- Race conditions in reference counting (use locks!)
- Not checking if page was originally writable before making it COW

完整代码在cow分支
