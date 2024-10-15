#include "kernel.hh"
#include "k-apic.hh"
#include "k-vmiter.hh"
#include "obj/k-firstprocess.h"
#include <atomic>

// kernel.cc
//
//    This is the kernel.


// INITIAL PHYSICAL MEMORY LAYOUT
//
//  +-------------- Base Memory --------------+
//  v                                         v
// +-----+--------------------+----------------+--------------------+---------/
// |     | Kernel      Kernel |       :    I/O | App 1        App 1 | App 2
// |     | Code + Data  Stack |  ...  : Memory | Code + Data  Stack | Code ...
// +-----+--------------------+----------------+--------------------+---------/
// 0  0x40000              0x80000 0xA0000 0x100000             0x140000
//                                             ^
//                                             | \___ PROC_SIZE ___/
//                                      PROC_START_ADDR

#define PROC_SIZE 0x40000       // initial state only

proc ptable[PID_MAX];           // array of process descriptors
                                // Note that `ptable[0]` is never used.
proc* current;                  // pointer to currently executing proc

#define HZ 100                  // timer interrupt frequency (interrupts/sec)
static std::atomic<unsigned long> ticks; // # timer interrupts so far


// Memory state - see `kernel.hh`
physpageinfo physpages[NPAGES];


[[noreturn]] void schedule();
[[noreturn]] void run(proc* p);
void exception(regstate* regs);
uintptr_t syscall(regstate* regs);
void memshow();


// kernel_start(command)
//    Initialize the hardware and processes and start running. The `command`
//    string is an optional string passed from the boot loader.

static void process_setup(pid_t pid, const char* program_name);

void kernel_start(const char* command) {
    // initialize hardware
    init_hardware();
    log_printf("Starting WeensyOS\n");

    ticks = 1;
    init_timer(HZ);

    // clear screen
    console_clear();

    // (re-)initialize kernel page table
    for (uintptr_t addr = 0; addr < MEMSIZE_PHYSICAL; addr += PAGESIZE) {
        int perm = PTE_P | PTE_W | PTE_U;
        //int perm; 
        if (addr == 0) {
            // nullptr is inaccessible even to the kernel
            perm = 0;
        }
        // for each kernel address, process cannot touch it - first bit of perm should be 1 
        // want to turn PTE on (1) for processes, off (0) for kernel 
        if (addr > 0 && addr < PROC_START_ADDR  && addr != CONSOLE_ADDR){
            perm = PTE_P | PTE_W;
        }

        // install identity mapping
        int r = vmiter(kernel_pagetable, addr).try_map(addr, perm);
        assert(r == 0); // mappings during kernel_start MUST NOT fail
                        // (Note that later mappings might fail!!)
    }


    // set up process descriptors
    for (pid_t i = 0; i < PID_MAX; i++) {
        ptable[i].pid = i;
        ptable[i].state = P_FREE;
    }
    if (!command) {
        command = WEENSYOS_FIRST_PROCESS;
    }
    if (!program_image(command).empty()) {
        process_setup(1, command);
    } else {
        process_setup(1, "allocator");
        process_setup(2, "allocator2");
        process_setup(3, "allocator3");
        process_setup(4, "allocator4");
    }

    // switch to first process using run()
    run(&ptable[1]);
}


// kalloc(sz)
//    Kernel physical memory allocator. Allocates at least `sz` contiguous bytes
//    and returns a pointer to the allocated memory, or `nullptr` on failure.
//    The returned pointer’s address is a valid physical address, but since the
//    WeensyOS kernel uses an identity mapping for virtual memory, it is also a
//    valid virtual address that the kernel can access or modify.
//
//    The allocator selects from physical pages that can be allocated for
//    process use (so not reserved pages or kernel data), and from physical
//    pages that are currently unused (`physpages[N].refcount == 0`).
//
//    On WeensyOS, `kalloc` is a page-based allocator: if `sz > PAGESIZE`
//    the allocation fails; if `sz < PAGESIZE` it allocates a whole page
//    anyway.
//
//    The returned memory is initially filled with 0xCC, which corresponds to
//    the `int3` instruction. Executing that instruction will cause a `PANIC:
//    Unhandled exception 3!` This may help you debug.

void* kalloc(size_t sz) {
    if (sz > PAGESIZE) {
        return nullptr;
    }

    
    int pageno = 0;
    int page_increment = 3;
    // In the handout code, `kalloc` returns the first free page.
    // Alternate search strategies can be faster and/or expose bugs elsewhere.
    // This initialization returns a random free page:
    // int pageno = rand(0, NPAGES - 1);
    // This initialization remembers the most-recently-allocated page and
    // starts the search from there:
    //     static int pageno = 0;
    // In Step 3, you must change the allocation to use non-sequential pages.
    // The easiest way to do this is to set page_increment to 3, but you can
    // also set `pageno` randomly.
    //int pageno = last_allocated_pageno; 
    for (int tries = 0; tries != NPAGES; ++tries) { // loop through all pages to find a free one 
        uintptr_t pa = pageno * PAGESIZE;
        if (allocatable_physical_address(pa) && physpages[pageno].refcount == 0) { // if its free
            ++physpages[pageno].refcount; //increment to the next refcount
            memset((void*) pa, 0xCC, PAGESIZE);
            
            return (void*) pa;
        }
        pageno = (pageno + page_increment) % NPAGES;
    }

    return nullptr;
}


// kfree(kptr)
//    Free `kptr`, which must have been previously returned by `kalloc`.
//    If `kptr == nullptr` does nothing.

void kfree(void* kptr) {
    if (kptr == nullptr) {
        return;
    }
    uintptr_t pa = (uintptr_t) kptr;
    
    assert(pa % PAGESIZE == 0);
    int pageno = pa / PAGESIZE;
    if (allocatable_physical_address(pa) && physpages[pageno].refcount == 1) {
        physpages[pageno].refcount = 0;
        memset(kptr, 0, PAGESIZE);
    }
}


//process_setup(pid, program_name)
//    Load application program `program_name` as process number `pid`.
//    This loads the application's code and data into memory, sets its
//    %rip and %rsp, gives it a stack page, and marks it as runnable.



void process_setup(pid_t pid, const char* program_name) {
    init_process(&ptable[pid], 0);


    // intitialize an empty page table
    x86_64_pagetable* pt = (x86_64_pagetable*) kalloc_pagetable(); 
    ptable[pid].pagetable = pt;


    vmiter kernel_it = vmiter(kernel_pagetable, 0x1000); //maps permissions in kernel pagetable to virtual address = physc addr 
    vmiter new_it = vmiter(pt, 0x1000); 
    

    while (kernel_it.va() < PROC_START_ADDR){

        //int perm = PTE_P | PTE_W; 
        int r = new_it.try_map(kernel_it.pa(), kernel_it.perm());
        assert(r == 0);  

        kernel_it += PAGESIZE; 
        new_it += PAGESIZE; 
        
    }
    

    // obtain reference to program image
    // (The program image models the process executable.)
    program_image pgm(program_name);

    int perm; 

    // allocate and map process memory as specified in program image
    for (auto seg = pgm.begin(); seg != pgm.end(); ++seg) {

        //perm = PTE_P | PTE_W | PTE_U; 
        perm = PTE_P | PTE_U; // everyone gets P and U permissions 

        if (seg.writable()){
            perm |=PTE_W;   //writeable pages get an extra write permission 
        }
       
        for (uintptr_t a = round_down(seg.va(), PAGESIZE); a < seg.va() + seg.size(); a += PAGESIZE) {
            uintptr_t pa = (uintptr_t) kalloc(PAGESIZE); 
             

        
            // `a` is the process virtual address for the next code/data page
            // (The handout code requires that the corresponding physical
            // address is currently free.)
            
            // assert(physpages[a / PAGESIZE].refcount == 0);
            // ++physpages[a / PAGESIZE].refcount;

            // map the permissions to a given process 
           
            kernel_it = kernel_it.find(a); 
            new_it += kernel_it.va() - new_it.va(); 
            
            //log_printf("%p", new_it.va()); 

            //int r = new_it.try_map(pa, PTE_P | PTE_W | PTE_U); 
            int r = new_it.try_map(pa, perm); 
            assert(r == 0); 

        }
    }
    
    // copy instructions and data from program image into process memory
    //total size of seg is 2500
    
    for (auto seg = pgm.begin(); seg != pgm.end(); ++seg) {
        //log_printf("size of the entire data segment is %x\n", seg.size());


        // vmiter vir_itr = vmiter(pt, 0x1000); 
        // vir_itr = vir_itr.find(seg.va()); //find virtual address - .pa() will pull its physical address


        // GOAL: loop through all of the pages that make up a segment and copy them 1 by 1 

        for (uintptr_t a = round_down(seg.va(), PAGESIZE); a < seg.va() + seg.size(); a += PAGESIZE){
            unsigned long position = a - seg.va(); 
            memset(vmiter(ptable[pid].pagetable, a).kptr(), 0, min(seg.size() - position, PAGESIZE)); 
            memcpy(vmiter(ptable[pid].pagetable, a).kptr(), seg.data() + position, min(seg.data_size() - position, PAGESIZE));
        }

    }

    // mark entry point
    ptable[pid].regs.reg_rip = pgm.entry();

    // allocate and map stack segment
    // Compute process virtual address for stack page
    // uintptr_t stack_addr = PROC_START_ADDR + PROC_SIZE * pid - PAGESIZE; 
    uintptr_t stack_addr = MEMSIZE_VIRTUAL - PAGESIZE; //style points
    
    
    uintptr_t pa = (uintptr_t) kalloc(PAGESIZE); 
    // The handout code requires that the corresponding physical address
    // is currently free.

    kernel_it = kernel_it.find(stack_addr); 
    new_it += kernel_it.va() - new_it.va();

    int r = new_it.try_map(pa, PTE_P | PTE_W | PTE_U);
    assert(r == 0); 

    
    ptable[pid].regs.reg_rsp = stack_addr + PAGESIZE;
    
    // mark process as runnable
    ptable[pid].state = P_RUNNABLE;  
    check_pagetable(ptable[pid].pagetable); 
}



void free_everything(x86_64_pagetable* pt){
    for (vmiter it(pt, 0); it.va() < MEMSIZE_VIRTUAL; it += PAGESIZE){
        if (it.user() && it.va() != CONSOLE_ADDR){ // if there's actually a physical page mapped here - and its NOT the console address

            int pageno = it.pa() / PAGESIZE; 
            physpages[pageno].refcount --; 
            if (physpages[pageno].refcount == 0){
                kfree(it.kptr());
            }
        }
    }
    for (ptiter it(pt); !it.done(); it.next()){
        kfree(it.kptr()); 
    }
    //free the pagetable
    kfree(pt); 
}


pid_t fork(){

   pid_t pid = - 1; 
   for (int i = 1; i < PID_MAX; i++){
        if (ptable[i].state == P_FREE){
            pid = i; 
            break; 
        }
   }
   if (pid < 0){
    return pid; 
   }
   
   x86_64_pagetable *pt = kalloc_pagetable(); 

   if (pt == nullptr){
    return -1; 
   }

   ptable[pid].pagetable = pt; 
     
//look for a slot in the ptable[] array 

for (uintptr_t addr = 0; addr < MEMSIZE_VIRTUAL; addr += PAGESIZE){
    vmiter parent = vmiter(ptable[current->pid].pagetable,addr);
    vmiter child = vmiter(ptable[pid].pagetable,addr);
       
        if (parent.present() && parent.writable() && parent.user() && addr != CONSOLE_ADDR) {
            void* process_addr = kalloc(PAGESIZE);

            if (process_addr == nullptr) {
                free_everything(ptable[pid].pagetable);
                return -1;
            }
           
            int r = child.try_map(process_addr, parent.perm());
           
            if (r != 0){
                free_everything(ptable[pid].pagetable);
                kfree(process_addr); 
                return -1;  
            }
            //copies over the data
            memcpy(child.kptr(), parent.kptr(), PAGESIZE);


        } else if (parent.present() && parent.user() && addr != CONSOLE_ADDR) {
            int r = child.try_map(parent.pa(), parent.perm());

            int pageno = (uintptr_t) parent.pa() / PAGESIZE;
            ++physpages[pageno].refcount;
            
            if (r != 0){
                free_everything(ptable[pid].pagetable); 
                return -1; 
            }
            
        } else if (parent.present()){
            int r = child.try_map(parent.pa(), parent.perm()); 

            if (r != 0){
                free_everything(ptable[pid].pagetable);
                return -1;  
            }
        } 
            
    }

    
    ptable[pid].regs = current->regs;
    ptable[pid].regs.reg_rax = 0;
    ptable[pid].state = P_RUNNABLE;
    return pid;
}



void sys_exit(){
    free_everything(ptable[current->pid].pagetable); 
    current->state = P_FREE; 
}


// exception(regs)
//    Exception handler (for interrupts, traps, and faults).
//
//    The register values from exception time are stored in `regs`.
//    The processor responds to an exception by saving application state on
//    the kernel's stack, then jumping to kernel assembly code (in
//    k-exception.S). That code saves more registers on the kernel's stack,
//    then calls exception().
//
//    Note that hardware interrupts are disabled when the kernel is running.

void exception(regstate* regs) {
    // Copy the saved registers into the `current` process descriptor.
    current->regs = *regs;
    regs = &current->regs;

    // It can be useful to log events using `log_printf`.
    // Events logged this way are stored in the host's `log.txt` file.
    /* log_printf("proc %d: exception %d at rip %p\n",
                current->pid, regs->reg_intno, regs->reg_rip); */

    // Show the current cursor location and memory state
    // (unless this is a kernel fault).
    console_show_cursor(cursorpos);
    if (regs->reg_intno != INT_PF || (regs->reg_errcode & PTE_U)) {
        memshow();
    }

    // If Control-C was typed, exit the virtual machine.
    check_keyboard();


    // Actually handle the exception.
    switch (regs->reg_intno) {

    case INT_IRQ + IRQ_TIMER:
        ++ticks;
        lapicstate::get().ack();
        schedule();
        break;                  /* will not be reached */

    case INT_PF: {
        // Analyze faulting address and access type.
        uintptr_t addr = rdcr2();
        const char* operation = regs->reg_errcode & PTE_W
                ? "write" : "read";
        const char* problem = regs->reg_errcode & PTE_P
                ? "protection problem" : "missing page";

        if (!(regs->reg_errcode & PTE_U)) {
            proc_panic(current, "Kernel page fault on %p (%s %s, rip=%p)!\n",
                       addr, operation, problem, regs->reg_rip);
        }
        error_printf(CPOS(24, 0), COLOR_ERROR,
                     "PAGE FAULT on %p (pid %d, %s %s, rip=%p)!\n",
                     addr, current->pid, operation, problem, regs->reg_rip);
        log_print_backtrace(current);
        current->state = P_FAULTED;
        break;
    }

    default:
        proc_panic(current, "Unhandled exception %d (rip=%p)!\n",
                   regs->reg_intno, regs->reg_rip);

    }


    // Return to the current process (or run something else).
    if (current->state == P_RUNNABLE) {
        run(current);
    } else {
        schedule();
    }
}


int syscall_page_alloc(uintptr_t addr);


// syscall(regs)
//    Handle a system call initiated by a `syscall` instruction.
//    The process’s register values at system call time are accessible in
//    `regs`.
//
//    If this function returns with value `V`, then the user process will
//    resume with `V` stored in `%rax` (so the system call effectively
//    returns `V`). Alternately, the kernel can exit this function by
//    calling `schedule()`, perhaps after storing the eventual system call
//    return value in `current->regs.reg_rax`.
//
//    It is only valid to return from this function if
//    `current->state == P_RUNNABLE`.
//
//    Note that hardware interrupts are disabled when the kernel is running.

uintptr_t syscall(regstate* regs) {
    // Copy the saved registers into the `current` process descriptor.
    current->regs = *regs;
    regs = &current->regs;

    // It can be useful to log events using `log_printf`.
    // Events logged this way are stored in the host's `log.txt` file.
    /* log_printf("proc %d: syscall %d at rip %p\n",
                  current->pid, regs->reg_rax, regs->reg_rip); */

    // Show the current cursor location and memory state.
    console_show_cursor(cursorpos);
    memshow();

    // If Control-C was typed, exit the virtual machine.
    check_keyboard();


    // Actually handle the exception.
    switch (regs->reg_rax) {

    case SYSCALL_PANIC:
        user_panic(current);
        break; // will not be reached

    case SYSCALL_GETPID:
        return current->pid;

    case SYSCALL_YIELD:
        current->regs.reg_rax = 0;
        schedule();             // does not return

    case SYSCALL_PAGE_ALLOC:
        return syscall_page_alloc(current->regs.reg_rdi);

    case SYSCALL_FORK: 
        return fork();

    //make a new case for exiting 
    case SYSCALL_EXIT: 
        sys_exit(); 
        schedule();  
    
    
        
    

    default:
        proc_panic(current, "Unhandled system call %ld (pid=%d, rip=%p)!\n",
                   regs->reg_rax, current->pid, regs->reg_rip);

    }

    panic("Should not get here!\n");
}


// syscall_page_alloc(addr)
//    Handles the SYSCALL_PAGE_ALLOC system call. This function
//    should implement the specification for `sys_page_alloc`
//    in `u-lib.hh` (but in the handout code, it does not).

int syscall_page_alloc(uintptr_t addr) {
    uintptr_t pa = (uintptr_t) kalloc(PAGESIZE); 
    // assert(physpages[addr / PAGESIZE].refcount == 0);
    // ++physpages[addr / PAGESIZE].refcount;

    if (!pa){
        return -1; 
    }

    memset((void*) pa, 0, PAGESIZE);

   

    pid_t pid = current->pid;
    x86_64_pagetable* pt = ptable[pid].pagetable;

    //vmiter kernel_it = vmiter(kernel_pagetable, addr); 
    vmiter proc_it = vmiter(pt, addr); 

    int r = proc_it.try_map(pa, PTE_P | PTE_W | PTE_U);
    assert(r==0); 

    
    return 0;
}


// schedule
//    Pick the next process to run and then run it.
//    If there are no runnable processes, spins forever.

void schedule() {
    pid_t pid = current->pid;
    for (unsigned spins = 1; true; ++spins) {
        pid = (pid + 1) % PID_MAX;
        if (ptable[pid].state == P_RUNNABLE) {
            run(&ptable[pid]);
        }

        // If Control-C was typed, exit the virtual machine.
        check_keyboard();

        // If spinning forever, show the memviewer.
        if (spins % (1 << 12) == 0) {
            memshow();
        }
    }
}


// run(p)
//    Run process `p`. This involves setting `current = p` and calling
//    `exception_return` to restore its page table and registers.

void run(proc* p) {
    assert(p->state == P_RUNNABLE);
    current = p;

    // Check the process's current registers.
    check_process_registers(p);

    // Check the process's current pagetable.
    check_pagetable(p->pagetable);

    // This function is defined in k-exception.S. It restores the process's
    // registers then jumps back to user mode.
    exception_return(p);

    // should never get here
    while (true) {
    }
}


// memshow()
//    Draw a picture of memory (physical and virtual) on the CGA console.
//    Switches to a new process's virtual memory map every 0.25 sec.
//    Uses `console_memviewer()`, a function defined in `k-memviewer.cc`.

void memshow() {
    static unsigned last_ticks = 0;
    static int showing = 0;

    // switch to a new process every 0.25 sec
    if (last_ticks == 0 || ticks - last_ticks >= HZ / 2) {
        last_ticks = ticks;
        showing = (showing + 1) % PID_MAX;
    }

    proc* p = nullptr;
    for (int search = 0; !p && search < PID_MAX; ++search) {
        if (ptable[showing].state != P_FREE
            && ptable[showing].pagetable) {
            p = &ptable[showing];
        } else {
            showing = (showing + 1) % PID_MAX;
        }
    }

    console_memviewer(p);
    if (!p) {
        console_printf(CPOS(10, 26), 0x0F00, "   VIRTUAL ADDRESS SPACE\n"
            "                          [All processes have exited]\n"
            "\n\n\n\n\n\n\n\n\n\n\n");
    }
}