#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use arcane_node as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [SWI0_EGU0]
)]
mod app {
    use arcane_node::initialize_can;
    use embedded_hal::{
        can::{Frame, Id, StandardId},
        digital::v2::OutputPin,
    };
    use mcp2515::{frame::CanFrame, MCP2515};
    use nrf52840_hal::{
        gpio::{p0, p1, Level, Output, Pin, PushPull},
        gpiote::Gpiote,
        pac::SPIM0,
        spim, Spim,
    };
    use rtic_monotonics::nrf::timer::{ExtU64, Timer0 as Mono};

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        led: Pin<Output<PushPull>>,
        can: MCP2515<Spim<SPIM0>, Pin<Output<PushPull>>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize Monotonic
        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(ctx.device.TIMER0, token);

        // initialize gpio ports
        let port0 = p0::Parts::new(ctx.device.P0);
        let port1 = p1::Parts::new(ctx.device.P1);

        // initialize LED (OFF)
        let led = port1.p1_10.into_push_pull_output(Level::Low).degrade();

        let gpiote = Gpiote::new(ctx.device.GPIOTE);

        // initialize SPI
        let spiclk = port0.p0_14.into_push_pull_output(Level::Low).degrade();
        let spimosi = port0.p0_13.into_push_pull_output(Level::Low).degrade();
        let spimiso = port0.p0_15.into_floating_input().degrade();
        let cs = port1.p1_08.into_push_pull_output(Level::Low).degrade();
        let mcp_int = port0.p0_07.into_pullup_input().degrade();

        let pins = spim::Pins {
            sck: Some(spiclk),
            miso: Some(spimiso),
            mosi: Some(spimosi),
        };

        let can = initialize_can(cs, &mcp_int, pins, ctx.device.SPIM0, &gpiote, ctx.core.SYST);

        // schedule task
        can_send_test::spawn().ok();

        (Shared {}, Local { led, can })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Cortex-M wait for interrupt instruction (WFI) - do nothing and
            // go to sleep
            rtic::export::wfi();
        }
    }

    #[task(priority = 1, shared = [], local = [led, can, state: bool = false])]
    async fn can_send_test(ctx: can_send_test::Context) {
        loop {
            let frame = match *ctx.local.state {
                true => CanFrame::new(
                    Id::Standard(StandardId::new(0b00010000001).unwrap()),
                    &[0x90, 0x3C, 0x40],
                )
                .unwrap(), // MIDI Note-On
                false => CanFrame::new(
                    Id::Standard(StandardId::new(0b00010000001).unwrap()),
                    &[0x90, 0x3C, 0x00],
                )
                .unwrap(), // MIDI Note-Off
            };

            ctx.local
                .can
                .send_message(frame)
                .unwrap_or_else(|_| defmt::error!("Something went wrong"));

            if *ctx.local.state {
                ctx.local.led.set_low().unwrap()
            } else {
                ctx.local.led.set_high().unwrap()
            };

            defmt::info!("Sent ARCANE Message");

            *ctx.local.state = !*ctx.local.state;

            Mono::delay(1000.millis()).await;

            defmt::info!("Waited for a second");
        }
    }
}
