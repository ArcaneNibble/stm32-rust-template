#![feature(alloc)]
#![feature(global_allocator)]
#![feature(core_intrinsics)]
#![feature(lang_items)]
#![feature(proc_macro)]
#![no_std]

extern crate alloc_cortex_m;
#[macro_use]
extern crate alloc;
extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate cortex_m_semihosting;
extern crate stm32f417;

use core::fmt::Write;
use core::intrinsics;

use cortex_m_semihosting::hio;
use alloc_cortex_m::CortexMHeap;
use rtfm::{app, Threshold};
use stm32f417::{GPIOD};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

extern "C" {
    static mut _sheap: u32;
    static mut _eheap: u32;
}

app! {
    device: stm32f417,

    resources: {
        static GPIOD: GPIOD;
        static SYST: cortex_m::peripheral::SYST;
        static USART2_OBJ: stm32f417::USART2;
    },

    idle: {
        resources: [GPIOD, SYST],
    },

    tasks: {
        USART2: {
            path: loopback,

            resources: [USART2_OBJ],
        }
    }
}

fn init(mut p: init::Peripherals) -> init::LateResources {
    // Initialize the allocator
    let start = unsafe { &mut _sheap as *mut u32 as usize };
    let end = unsafe { &mut _eheap as *mut u32 as usize };
    unsafe { ALLOCATOR.init(start, end - start) }

    // Test allocator
    let xs = vec![0, 1, 2];

    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Hello world!").unwrap();
    writeln!(stdout, "{:?}", xs).unwrap();

    // Set up clocks
    let rcc = p.device.RCC;

    // Turn clock-related interrupts off and clear them
    rcc.cir.reset();
    rcc.cir.modify(|_, w| w
        .cssc().clear()
        .plli2srdyc().clear()
        .pllrdyc().clear()
        .hserdyc().clear()
        .hsirdyc().clear()
        .lserdyc().clear()
        .lsirdyc().clear());

    // Turn on enternal clock
    rcc.cr.modify(|_, w| w.hseon().bit(false));
    rcc.cr.modify(|_, w| w.hsebyp().bit(false));
    rcc.cr.modify(|_, w| w.hseon().bit(true));
    while !rcc.cr.read().hserdy().bit() {}

    // Turn on PLL (turn off, configure, turn on)
    rcc.cr.modify(|_, w| w.pllon().bit(false));
    // Turn off spread-spectrum
    rcc.sscgr.reset();
    // Need to keep reserved values
    // We are configuring a 336 MHz VCO from an 8 MHz input
    // in order to output a system clock of 168 MHz and a USB/SDIO clock of
    // 48 MHz.
    rcc.pllcfgr.modify(|_, w| w
        .pllm().bits(4)             // 8 MHz to 2 MHz
        .plln().bits(168)           // 2 MHz to 336 MHz
        .pllp()._2()                // 336 MHz to 168 MHz
        .pllsrc().hse()             // Use crystal
        .pllq().bits(7));           // 336 MHz to 48 MHz
    rcc.cr.modify(|_, w| w.pllon().bit(true));
    while !rcc.cr.read().pllrdy().bit() {}

    // Enable flash caching and prefetching
    // According to the table we need 5 waitstates
    // We also need to do this _before_ switching clock sources.
    p.device.FLASH.acr.write(|w| w
        .dcen().bit(true)
        .icen().bit(true)
        .prften().bit(true)
        .latency().bits(5));

    // Set the prescalers so that AHB is 168 MHz, APB2 is 84 MHz, APB1 is 42 MHz
    rcc.cfgr.write(|w| w
        .ppre2()._2()
        .ppre1()._4()
        .hpre()._1());
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}

    // Turn off HSI and CSS, we don't need them
    rcc.cr.modify(|_, w| w.hsion().bit(false).csson().bit(false));

    // Power up the relevant peripherals
    rcc.ahb1enr.modify(|_, w| w
        .gpioaen().enabled()
        .gpioden().enabled());
    rcc.apb1enr.modify(|_, w| w
        .usart2en().enabled());

    // Ensure VTOR points to flash
    unsafe { p.core.SCB.vtor.write(0x08000000); }

    // Set up USART2 for 1M 8N1
    let usart2 = p.device.USART2;
    usart2.brr.write(|w| w.div_mantissa().bits(2).div_fraction().bits(10));
    usart2.cr1.write(|w| w.ue().bit(true).te().bit(true).re().bit(true));
    usart2.cr2.write(|w| w.stop()._1());
    usart2.cr3.write(|w| w);

    // Set up USART2 on PA2/3
    let gpioa = p.device.GPIOA;
    gpioa.afrl.modify(|_, w| w.afrl2().af7().afrl3().af7());
    gpioa.moder.modify(|_, w| w.moder2().alternate().moder3().alternate());

    // Enable the USART2 RXNE interrupt
    usart2.cr1.modify(|_, w| w.rxneie().bit(true));

    let gpiod = p.device.GPIOD;

    // Configure the pin PD15 as an output pin
    gpiod.moder.modify(|_, w| w.moder15().output());

    // Set up systick for 1 ms timeouts
    p.core.SYST.set_reload(168_000 - 1);
    p.core.SYST.clear_current();
    p.core.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    p.core.SYST.enable_counter();

    init::LateResources {
        GPIOD: gpiod,
        SYST: p.core.SYST,
        USART2_OBJ: usart2,
    }
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    // blink the LED
    let mut state = false;
    loop {
        for _ in 0..1000 {
            while !r.SYST.has_wrapped() {}
        }

        // Toggle the state
        state = !state;

        // Blink the LED
        if state {
            r.GPIOD.bsrr.write(|w| w.bs15().set());
            // r.USART2.dr.write(|w| w.dr().bits('1' as u16));
        } else {
            r.GPIOD.bsrr.write(|w| w.br15().reset());
            // r.USART2.dr.write(|w| w.dr().bits('0' as u16));
        }
    }
}

fn loopback(_t: &mut Threshold, r: USART2::Resources) {
    // Loop back incoming bytes
    let c = r.USART2_OBJ.dr.read().dr().bits();
    r.USART2_OBJ.dr.write(|w| w.dr().bits(c));
}

#[lang = "panic_fmt"]
#[no_mangle]
pub unsafe extern "C" fn rust_begin_unwind(
    args: core::fmt::Arguments,
    file: &'static str,
    line: u32,
    col: u32,
) -> ! {
    if let Ok(mut stdout) = hio::hstdout() {
        write!(stdout, "panicked at '")
            .and_then(|_| {
                stdout
                    .write_fmt(args)
                    .and_then(|_| writeln!(stdout, "', {}:{}:{}", file, line, col))
            })
            .ok();
    }

    intrinsics::abort()
}
