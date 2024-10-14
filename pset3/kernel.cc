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
        if (addr > 0 && addr < 0x100000 && addr != 0xB8000){
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

    for (int tries = 0; tries != NPAGES; ++tries) {
        uintptr_t pa = pageno * PAGESIZE;
        if (allocatable_physical_address(pa)
            && physpages[pageno].refcount == 0) {
            ++physpages[pageno].refcount;
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
    (void) kptr;

    if (!kptr){
        return; // do nothing if this is a nullptr
    }

    uintptr_t pa = (uintptr_t)kptr; //copy this from kalloc and use kptr
    int pageno = pa / PAGESIZE;

    assert(pa % PAGESIZE == 0); //make sure this is aligned
    assert(physpages[pageno].refcount > 0); //make sure this was previously given by kalloc
    
    //if this is zero, decrememt and clear the page 
    if (--physpages[pageno].refcount == 0){
        memset((void*)pa, 0, PAGESIZE); //sets everything in the page to 0 --leaks?? 
        // worried this might build up to a bunch of zeros 
        
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

    // allocate and map process memory as specified in program image
    for (auto seg = pgm.begin(); seg != pgm.end(); ++seg) {
        for (uintptr_t a = round_down(seg.va(), PAGESIZE);
            a < seg.va() + seg.size();
            a += PAGESIZE) {
            // `a` is the process virtual address for the next code/data page
            // (The handout code requires that the corresponding physical
            // address is currently free.)
            
            // assert(physpages[a / PAGESIZE].refcount == 0);
            // ++physpages[a / PAGESIZE].refcount;

            // map the permissions to a given process 
            uintptr_t pa = (uintptr_t) kalloc(PAGESIZE); 
             
            kernel_it = kernel_it.find(a); 
            new_it += kernel_it.va() - new_it.va(); 
            
            //log_printf("%p", new_it.va()); 

            int r = new_it.try_map(pa, PTE_P | PTE_W | PTE_U); 
            assert(r == 0); 

        }
    }
    
    // copy instructions and data from program image into process memory
    for (auto seg = pgm.begin(); seg != pgm.end(); ++seg) {

        vmiter vir_itr = vmiter(pt, 0x1000); 
        vir_itr = vir_itr.find(seg.va()); //find virtual address - .pa() will pull its physical address
        
        // to handle disjoint segments in physical memory, memcpy 1 by 1 
        
        size_t processed = 0; // number of segments that i've already copied over / processed
        //something's wrong here. should not be subtracting pagesize in second argument 
        //log_printf("size of segment data is %x\n", seg.data_size()); 

        for (uintptr_t a = round_down(seg.va(), PAGESIZE); a < seg.va() + seg.size(); a += PAGESIZE){
            
            size_t copysize = PAGESIZE;
           
            // if we hit the segment where its not the full pagesize 
            if (seg.va() + seg.size() - a < PAGESIZE){
                //log_printf("made it to end of the segments"); 
                copysize = seg.va() + seg.size() - a;  //theres less than a full page left to copy  
                //log_printf("copied size for smaller segment: %x", copysize);
            }

            
            //otherwise, we have a full page, so copy segment data to virtual addr 
            memcpy((void*) vir_itr.pa(), seg.data() + processed, copysize);
            //log_printf("found the %x'th full page\n", processed);
            
            processed += copysize; 
            //log_printf("process after copysize: %x\n", processed);

            //log_printf(" virtual address: %x, segment size: %x, iteration: %x, physical address: %x \n", seg.va(), seg.size(), a, vir_itr.pa()); 

            vir_itr += PAGESIZE; //move iterator to next page 
            //log_printf("address of vir_itr in virtual memory is %x\n", vir_itr.va());

        }
        if (processed < seg.size()){
            size_t remaining = seg.size() - processed; 
            memset((void*)(vir_itr.pa() - PAGESIZE + processed), 0, remaining); 
        }
    
    }
    

    // mark entry point
    ptable[pid].regs.reg_rip = pgm.entry();

    // allocate and map stack segment
    // Compute process virtual address for stack page
    // uintptr_t stack_addr = PROC_START_ADDR + PROC_SIZE * pid - PAGESIZE; 
    uintptr_t stack_addr = 0x300000 - PAGESIZE; 
    
    
    uintptr_t pa = (uintptr_t) kalloc(PAGESIZE); 
    // The handout code requires that the corresponding physical address
    // is currently free.
    // assert(physpages[stack_addr / PAGESIZE].refcount == 0);
    // ++physpages[stack_addr / PAGESIZE].refcount;

    kernel_it = kernel_it.find(stack_addr); 
    new_it += kernel_it.va() - new_it.va();

    int r = new_it.try_map(pa, PTE_P | PTE_W | PTE_U);
    assert(r == 0); 

    
    ptable[pid].regs.reg_rsp = stack_addr + PAGESIZE;
    
    // mark process as runnable
    ptable[pid].state = P_RUNNABLE;  
}



// copy the page tables, with the address logic 
int copy_page_table(x86_64_pagetable* parent_pt, x86_64_pagetable* child_pt, pid_t parent_pid){

    for (uintptr_t addr = 0; addr < MEMSIZE_VIRTUAL; addr += PAGESIZE){
        vmiter parent_itr(parent_pt, addr); 
        vmiter child_itr(child_pt, addr); 

        if (parent_itr.present()){
            uintptr_t pa = parent_itr.pa(); //grab physical addresses
            int perm = parent_itr.perm(); //grab permissions

            if (addr >= PROC_START_ADDR && perm & PTE_W){ //we live in allowed virtual memory now, with write permissions
                pa = (uintptr_t) kalloc(PAGESIZE); 
                
                if (!pa){
                    return 0; //failed to allocate enough space for the physical addresses - out of memory! 
                }
                // copy data over into pa, and this is what gets fed into the mapping
                memcpy((void*)pa, (void*)parent_itr.pa(), PAGESIZE); //copy over all parent info into new allocated pagetable 
                
            }
            // otherwise, we're below PROC_START_ADDR, and we can just straight up copy the physical pages in without changing pa 
            child_itr.map(pa, perm); 
        }

    }
    return 1; //successful copy! 
}

void free_everything(x86_64_pagetable* pt){

    for (vmiter it(pt, 0); it.va() < MEMSIZE_VIRTUAL; it += PAGESIZE){
        if (it.present() && it.va() != 0xB8000){ // if there's actually a physical page mapped here - and its NOT the console address

            log_printf("freeing virtual address 0x%X, physical 0x%X\n", it.va(), it.pa());
            kfree((void*)it.pa()); 
        }
    }

    for (ptiter it(pt); !it.done(); it.next()){
        kfree((void*)it.kptr()); 
    }

    //free the pagetable
    kfree(pt); 

}



int fork(pid_t parent_pid){

    log_printf("starting fork\n"); 
    if(parent_pid < 0 || parent_pid >= PID_MAX || ptable[parent_pid].pid == -1){
        return -1; 
    }

     
    //look for a slot in the ptable[] array 
   
    for (int i = 1; i < PID_MAX; ++i){
        if (ptable[i].state == P_FREE){ // there's a free slot here! 

            //create a new, initially empty pagetable for the child 
            x86_64_pagetable* child_pt = (x86_64_pagetable*) kalloc_pagetable();

            if (!child_pt){
                return -1; 
            }
            
            //log_printf("made it here"); 

            //initialize the child's process entry 
            ptable[i].pagetable = child_pt; //pointer to the child's new pagetable 
            ptable[i].pid = i; //set it to the freed id 
            ptable[i].state = P_RUNNABLE; //set state to runnable 
            ptable[i].regs = ptable[parent_pid].regs; //copy over the same registers from the parent
            ptable[i].regs.reg_rax = 0; //set rax to 0 

            //copy parent's pt to child 
            if (!copy_page_table(ptable[parent_pid].pagetable, child_pt, parent_pid)){ //if the copying fails 
                log_printf("entered fail condition\n"); 
                free_everything(child_pt); //free everything
                return -1; //fail condition
            }

            return i; //returns child PID

            }
        }
    return -1; 
}


void exit(int pid){
    // if the process we're interested in freeing isn't already free, call kfree
    if (ptable[pid].state != P_FREE){
        
        x86_64_pagetable* pt = ptable[pid].pagetable;

        // iterate through all the virtual addresses in a process we wanna free's pagetable 
        for (vmiter it(pt, 0); it.va() < MEMSIZE_VIRTUAL; it += PAGESIZE){
            if (it.present() && it.va() != CONSOLE_ADDR){
                kfree((void*)it.pa()); // release the corresponding physical memory
            }
            
        } 
        
        // free the page table itself (could have multiple levels)
        // PLEASE tell me i don't have to go through every page table level 


        //mark process as free
        ptable[pid].state = P_FREE; 
        ptable[pid].pagetable = nullptr; //set page table pointer to a nullptr 
        memset(&ptable[pid].regs, 0, sizeof(ptable[pid].regs)); //clear registers

    }

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
        return fork(current->pid);

    //make a new case for exiting 
    case SYSCALL_EXIT: 
        exit(current->pid);
        
        
        
        

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
