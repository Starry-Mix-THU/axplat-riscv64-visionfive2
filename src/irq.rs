//! TODO: PLIC

use crate::config::{devices::PLIC_BASE, plat::CPU_NUM};
use axplat::{
    irq::{HandlerTable, IrqHandler, IrqIf},
    mem::phys_to_virt,
};
use core::sync::atomic::{AtomicPtr, Ordering};
use lazyinit::LazyInit;
use plic::{Mode, PLIC};
use riscv::register::sie;

/// `Interrupt` bit in `scause`
pub(super) const INTC_IRQ_BASE: usize = 1 << (usize::BITS - 1);

/// Supervisor software interrupt in `scause`
#[allow(unused)]
pub(super) const S_SOFT: usize = INTC_IRQ_BASE + 1;

/// Supervisor timer interrupt in `scause`
pub(super) const S_TIMER: usize = INTC_IRQ_BASE + 5;

/// Supervisor external interrupt in `scause`
pub(super) const S_EXT: usize = INTC_IRQ_BASE + 9;

static TIMER_HANDLER: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

/// The maximum number of IRQs.
pub const MAX_IRQ_COUNT: usize = 1024;

static IRQ_HANDLER_TABLE: HandlerTable<MAX_IRQ_COUNT> = HandlerTable::new();

static PLIC: LazyInit<PLIC<CPU_NUM>> = LazyInit::new();

fn init_plic() -> PLIC<CPU_NUM> {
    let plic = PLIC::new(phys_to_virt(PLIC_BASE.into()).as_usize(), [2; CPU_NUM]);
    for hart in 0..(CPU_NUM as u32) {
        plic.set_threshold(hart, Mode::Supervisor, 0);
    }
    plic
}

macro_rules! with_cause {
    ($cause: expr, @S_TIMER => $timer_op: expr, @S_EXT => $ext_op: expr, @EX_IRQ => $plic_op: expr $(,)?) => {
        match $cause {
            S_TIMER => $timer_op,
            S_EXT => $ext_op,
            other => {
                if other & INTC_IRQ_BASE == 0 {
                    // Device-side interrupts read from PLIC
                    $plic_op
                } else {
                    // Other CPU-side interrupts
                    panic!("Unknown IRQ cause: {}", other);
                }
            }
        }
    };
}

pub(super) fn init_percpu() {
    // enable soft interrupts, timer interrupts, and external interrupts
    unsafe {
        sie::set_ssoft();
        sie::set_stimer();
        sie::set_sext();
    }
}

struct IrqIfImpl;

#[impl_plat_interface]
impl IrqIf for IrqIfImpl {
    /// Enables or disables the given IRQ.
    fn set_enable(irq: usize, enabled: bool) {
        with_cause!(
            irq,
            @S_TIMER => {
                unsafe {
                    if enabled {
                        sie::set_stimer();
                    } else {
                        sie::clear_stimer();
                    }
                }
            },
            @S_EXT => {
                warn!("External IRQ should be got from PLIC, not scause");
            },
            @EX_IRQ => {
                PLIC.call_once(init_plic);
                let plic = &PLIC;
                if enabled {
                    plic.set_priority(irq as _, 6);
                    for hart in 0..(CPU_NUM as u32) {
                        plic.enable(hart, Mode::Supervisor, irq as _);
                    }
                } else {
                    for hart in 0..(CPU_NUM as u32) {
                        plic.disable(hart, Mode::Supervisor, irq as _);
                    }
                }
            }
        );
    }

    /// Registers an IRQ handler for the given IRQ.
    ///
    /// It also enables the IRQ if the registration succeeds. It returns `false` if
    /// the registration failed.
    ///
    /// The `irq` parameter has the following semantics
    /// 1. If its highest bit is 1, it means it is an interrupt on the CPU side. Its
    /// value comes from `scause`, where [`S_SOFT`] represents software interrupt
    /// and [`S_TIMER`] represents timer interrupt. If its value is [`S_EXT`], it
    /// means it is an external interrupt, and the real IRQ number needs to
    /// be obtained from PLIC.
    /// 2. If its highest bit is 0, it means it is an interrupt on the device side,
    /// and its value is equal to the IRQ number provided by PLIC.
    fn register(irq: usize, handler: IrqHandler) -> bool {
        with_cause!(
            irq,
            @S_TIMER => TIMER_HANDLER.compare_exchange(core::ptr::null_mut(), handler as *mut _, Ordering::AcqRel, Ordering::Acquire).is_ok(),
            @S_EXT => {
                warn!("External IRQ should be got from PLIC, not scause");
                false
            },
            @EX_IRQ => {
                if IRQ_HANDLER_TABLE.register_handler(irq, handler) {
                    Self::set_enable(irq, true);
                    true
                } else {
                    false
                }
            }
        )
    }

    /// Unregisters the IRQ handler for the given IRQ.
    ///
    /// It also disables the IRQ if the unregistration succeeds. It returns the
    /// existing handler if it is registered, `None` otherwise.
    fn unregister(irq: usize) -> Option<IrqHandler> {
        with_cause!(
            irq,
            @S_TIMER => {
                let handler = TIMER_HANDLER.swap(core::ptr::null_mut(), Ordering::AcqRel);
                if !handler.is_null() {
                    Some(unsafe { core::mem::transmute::<*mut (), IrqHandler>(handler) })
                } else {
                    None
                }
            },
            @S_EXT => {
                warn!("External IRQ should be got from PLIC, not scause");
                None
            },
            @EX_IRQ => IRQ_HANDLER_TABLE.unregister_handler(irq)
        )
    }

    /// Handles the IRQ.
    ///
    /// It is called by the common interrupt handler. It should look up in the
    /// IRQ handler table and calls the corresponding handler. If necessary, it
    /// also acknowledges the interrupt controller after handling.
    fn handle(irq: usize) {
        with_cause!(
            irq,
            @S_TIMER => {
                trace!("IRQ: timer");
                let handler = TIMER_HANDLER.load(Ordering::Acquire);
                if !handler.is_null() {
                    // SAFETY: The handler is guaranteed to be a valid function pointer.
                    unsafe { core::mem::transmute::<*mut (), IrqHandler>(handler)(irq) };
                }
            },
            @S_EXT => {
                if !PLIC.is_inited() {
                    return;
                }
                // TODO: hart
                let irq = PLIC.claim(0, Mode::Supervisor);
                if !IRQ_HANDLER_TABLE.handle(irq as _) {
                    warn!("Unhandled IRQ {irq}");
                }
                PLIC.complete(0, Mode::Supervisor, irq);
            },
            @EX_IRQ => {
                unreachable!("Device-side IRQs should be handled by triggering the External Interrupt.");
            }
        )
    }
}
