use crate::config::plat::{BOOT_STACK_SIZE, PHYS_VIRT_OFFSET};
use axplat::mem::{Aligned4K, pa};

#[unsafe(link_section = ".bss.stack")]
static mut BOOT_STACK: [u8; BOOT_STACK_SIZE] = [0; BOOT_STACK_SIZE];

#[unsafe(link_section = ".data")]
static mut BOOT_PT_SV39: Aligned4K<[u64; 512]> = Aligned4K::new([0; 512]);

#[allow(clippy::identity_op)] // (0x0 << 10) here makes sense because it's an address
unsafe fn init_boot_page_table() {
    unsafe {
        // 0x0000_0000..0x4000_0000, VRWX_GAD, 1G block
        BOOT_PT_SV39[0] = (0x0 << 10) | 0xef;
        // 0x4000_0000..0x8000_0000, VRWX_GAD, 1G block
        BOOT_PT_SV39[1] = (0x40000 << 10) | 0xef;
        // 0xffff_ffc0_0000_0000..0xffff_ffc0_4000_0000, VRWX_GAD, 1G block
        BOOT_PT_SV39[0x100] = (0x0 << 10) | 0xef;
        // 0xffff_ffc0_4000_0000..0xffff_ffc0_4000_0000, VRWX_GAD, 1G block
        BOOT_PT_SV39[0x101] = (0x40000 << 10) | 0xef;
    }
}

unsafe fn init_mmu() {
    unsafe {
        axcpu::asm::write_kernel_page_table(pa!(&raw const BOOT_PT_SV39 as usize));
        axcpu::asm::flush_tlb(None);
    }
}

#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".text.boot")]
unsafe extern "C" fn _start() -> ! {
    core::arch::naked_asm!("
        .ascii  \"MZ\"
        j       {entry}

        .balign 8
        .dword  0x200000            // Image load offset, little endian
        .dword  _ekernel - _skernel // Effective Image size, little endian
        .dword  0                   // Kernel flags, little endian
        .word   0x2                 // Version of this header
        .word   0                   // Reserved
        .dword  0                   // Reserved
        .ascii  \"RISCV\"           // Magic number, little endian

        .balign 8
        .ascii  \"RSC\\x05\"        // Magic number 2, little endian
        .word   0                   // Reserved for PE COFF offset
        ",
        entry = sym _start_primary,
    )
}

/// The earliest entry point for the primary CPU.
#[unsafe(naked)]
#[unsafe(link_section = ".text.boot")]
unsafe extern "C" fn _start_primary() -> ! {
    // PC = 0x8020_0000
    // a0 = hartid
    // a1 = dtb
    core::arch::naked_asm!("
        mv      s0, a0                  // save hartid
        mv      s1, a1                  // save DTB pointer
        la      sp, {boot_stack}
        li      t0, {boot_stack_size}
        add     sp, sp, t0              // setup boot stack

        call    {init_boot_page_table}
        call    {init_mmu}              // setup boot page table and enabel MMU

        li      s2, {phys_virt_offset}  // fix up virtual high address
        add     sp, sp, s2

        mv      a0, s0
        addi    a0, a0, -1
        mv      a1, s1
        la      a2, {entry}
        add     a2, a2, s2
        jalr    a2                      // call_main(cpu_id, dtb)
        j       .",
        phys_virt_offset = const PHYS_VIRT_OFFSET,
        boot_stack_size = const BOOT_STACK_SIZE,
        boot_stack = sym BOOT_STACK,
        init_boot_page_table = sym init_boot_page_table,
        init_mmu = sym init_mmu,
        entry = sym axplat::call_main,
    )
}

/// The earliest entry point for secondary CPUs.
#[cfg(feature = "smp")]
#[unsafe(naked)]
#[unsafe(link_section = ".text.boot")]
pub(crate) unsafe extern "C" fn _start_secondary() -> ! {
    // a0 = hartid
    // a1 = SP
    core::arch::naked_asm!("
        mv      s0, a0                  // save hartid
        mv      sp, a1                  // set SP

        call    {init_mmu}              // setup boot page table and enabel MMU

        li      s1, {phys_virt_offset}  // fix up virtual high address
        add     a1, a1, s1
        add     sp, sp, s1

        mv      a0, s0
        addi    a0, a0, -1
        la      a1, {entry}
        add     a1, a1, s1
        jalr    a1                      // call_secondary_main(cpu_id)
        j       .",
        phys_virt_offset = const PHYS_VIRT_OFFSET,
        init_mmu = sym init_mmu,
        entry = sym axplat::call_secondary_main,
    )
}
