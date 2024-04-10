#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use arcane_node as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = nrf52840_hal::pac,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [SWI0_EGU0]
)]
mod app {
    use nrf52840_hal::{
        gpio::{p0, p1, Input, Level, Output, Pin, PullUp, PushPull},
        gpiote::Gpiote,
        pac::{pdm, PDM},
        prelude::*,
    };

    use rtic_monotonics::nrf::rtc::Rtc0 as Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        led: Pin<Output<PushPull>>,
    }

    // Local resources go here
    #[local]
    struct Local {
        gpiote: Gpiote,
        pdm: PDM,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // let p = Peripherals::take().unwrap(); // ASK: why doesn't this work?

        // set SLEEPONEXIT and SLEEPDEEP bits to enter low power sleep states
        ctx.core.SCB.set_sleeponexit();
        ctx.core.SCB.set_sleepdeep();

        // Initialize Monotonic
        let token = rtic_monotonics::create_nrf_rtc0_monotonic_token!();
        Mono::start(ctx.device.RTC0, token);

        let port0 = p0::Parts::new(ctx.device.P0);

        let port1 = p1::Parts::new(ctx.device.P1);

        // initialize LED (OFF)
        let led = port1.p1_01.into_push_pull_output(Level::Low).degrade();

        // intialize PDM CLK
        let _clk: Pin<Output<PushPull>> = port1.p1_09.into_push_pull_output(Level::Low).degrade();
        let _dat: Pin<Input<PullUp>> = port0.p0_08.into_pullup_input().degrade();

        // setup GPIOTE peripheral to trigger an interrupt on P1_02 (CLUE button A)
        // high-to-low transition. another option would be to enable SENSE in pin
        // configuration and then use GPIOTE PORT event for detection...
        let button = port1.p1_02.into_pullup_input().degrade();
        let gpiote = Gpiote::new(ctx.device.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&button)
            .hi_to_lo()
            .enable_interrupt();

        // let pdm = p.PDM; // ASK: why not??
        let pdm = ctx.device.PDM;

        // configure PDM clock
        pdm.psel.clk.write(|w| unsafe {
            w.port().bit(true);
            w.pin().bits(0x09);
            w.connect().bit(true)
        });

        // configure PDM data pin
        pdm.psel.din.write(|w| unsafe {
            w.port().bit(false);
            w.pin().bits(0x08);
            w.connect().bit(true)
        });

        // set mode to mono
        pdm.mode.write(|w| w.operation().bit(true));

        // pdm.inten
        //     .write(|w| w.started().bit(true).stopped().bit(true).end().bit(true));

        // enable PDM peripheral
        pdm.enable.write(|w| w.enable().bit(true));

        pdm.sample.ptr.write(|w| unsafe { w.bits(0x20000000) });
        pdm.sample
            .maxcnt
            .write(|w| unsafe { w.buffsize().bits(0x2) });

        pdm.tasks_start.write(|w| unsafe { w.bits(0x1) });

        defmt::info!("init done");

        pdm_read::spawn().ok();

        (Shared { led }, Local { gpiote, pdm })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Cortex-M wait for interrupt instruction (WFI) - do nothing and
            // go to sleep
            rtic::export::wfi();
        }
    }

    #[task(binds = GPIOTE, shared = [led] , local = [gpiote, state: bool = false])]
    fn gpiote_event(mut ctx: gpiote_event::Context) {
        if ctx.local.gpiote.channel0().is_event_triggered() {
            ctx.local.gpiote.channel0().reset_events();

            match *ctx.local.state {
                true => ctx.shared.led.lock(|l| l.set_low().unwrap()),
                false => ctx.shared.led.lock(|l| l.set_low().unwrap()),
            }
            *ctx.local.state = !*ctx.local.state;
        }
    }

    // #[task(binds = PDM, shared = [led], local = [pdm])]
    // fn pdm_event(ctx: pdm_event::Context) {
    //     let reg = ctx.local.pdm.sample.ptr.read();
    //     let sample = reg.bits();
    //     defmt::info!("{:?}", sample);
    // }

    #[task(priority = 1, shared = [led], local = [pdm])]
    async fn pdm_read(ctx: pdm_read::Context) {
        loop {
            // this reads the value of the pointer, not the actua
            // let started = ctx.local.pdm.events_started.read().bits();
            // if started != 0 {
            //     defmt::trace!("started {:?}", started)
            // }

            // let data = ctx.local.pdm.events_end.read().bits();
            // if data != 0 {
            //     defmt::trace!("data {:?}", data);
            // }

            let txd: [u8; 2] = [0, 0];
            let txd_ptr = txd.as_ptr() as u32;

            // Set up the DMA read.
            ctx.local
                .pdm
                .sample
                .ptr
                .write(|w| unsafe { w.sampleptr().bits(txd_ptr) });
            if 1 != 0 {
                defmt::trace!("test {:?}", txd);
            }
        }
    }
}
