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
    use arcane_node::mic::Microphone;
    use embedded_hal::can::{Frame, Id, StandardId};
    use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
    use nrf52840_hal::{
        gpio::{p0, p1, Floating, Input, Level, Output, Pin, PushPull},
        gpiote::Gpiote,
        pac::{PDM, SPIM0},
        prelude::*,
        spim, Delay, Spim,
    };

    use rtic_monotonics::{nrf::rtc::Rtc0 as Mono, systick::fugit::Duration};

    // Shared resources go here
    #[shared]
    struct Shared {
        led: Pin<Output<PushPull>>,
    }

    // Local resources go here
    #[local]
    struct Local {
        gpiote: Gpiote,
        can: MCP2515<Spim<SPIM0>, p1::P1_08<Output<PushPull>>>,
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

        // initialize gpio ports
        let port0 = p0::Parts::new(ctx.device.P0);
        let port1 = p1::Parts::new(ctx.device.P1);

        // initialize LED (OFF)
        let led = port1.p1_01.into_push_pull_output(Level::Low).degrade();

        // initialize SPI
        let spiclk = port0.p0_14.into_push_pull_output(Level::Low).degrade();
        let spimosi = port0.p0_13.into_push_pull_output(Level::Low).degrade();
        let spimiso = port0.p0_15.into_floating_input().degrade();

        let cs = port1.p1_08.into_push_pull_output(Level::Low);

        let pins = spim::Pins {
            sck: Some(spiclk),
            miso: Some(spimiso),
            mosi: Some(spimosi),
        };
        let spi = Spim::new(
            ctx.device.SPIM0,
            pins,
            spim::Frequency::K500,
            spim::MODE_0,
            0,
        );

        // initialize CAN

        let mut delay = Delay::new(ctx.core.SYST);

        let mut can = MCP2515::new(spi, cs);

        can.init(
            &mut delay,
            mcp2515::Settings {
                mode: OpMode::Loopback,        // Loopback for testing and example
                can_speed: CanSpeed::Kbps1000, // Many options supported.
                mcp_speed: McpSpeed::MHz16,    // Currently 16MHz and 8MHz chips are supported.
                clkout_en: false,
            },
        )
        .unwrap();

        // setup GPIOTE peripheral to trigger an interrupt on P1_02
        // high-to-low transition. another option would be to enable SENSE in pin
        // configuration and then use GPIOTE PORT event for detection...
        let button = port1.p1_02.into_pullup_input().degrade();
        let gpiote = Gpiote::new(ctx.device.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&button)
            .hi_to_lo()
            .enable_interrupt();

        (Shared { led }, Local { gpiote, can })
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

    #[task(priority = 1, shared = [], local = [can, state: bool = false])]
    async fn can_send_test(ctx: can_send_test::Context) {
        loop {
            let frame = match *ctx.local.state {
                true => CanFrame::new(
                    Id::Standard(StandardId::new(0b0001 + 0b000001).unwrap()),
                    &[0x90, 0x3C, 0x40],
                )
                .unwrap(), // MIDI Note-On
                false => CanFrame::new(
                    Id::Standard(StandardId::new(0b0001 + 0b000001).unwrap()),
                    &[0x90, 0x3C, 0x00],
                )
                .unwrap(), // MIDI Note-Off
            };
            // Send a message
            let frame = CanFrame::new(
                Id::Standard(StandardId::new(0b0000 + 0b000000).unwrap()),
                &[0x90, 0x3C, 0x40], // MIDI Note-On
            )
            .unwrap();

            ctx.local.can.send_message(frame).unwrap();

            defmt::info!("Sent Note-On");

            // Read the message back (we are in loopback mode)
            // match ctx.local.can.read_message() {
            //     Ok(frame) => {
            //         defmt::info!("Received frame {:?}", frame);
            //     }
            //     Err(Error::NoMessage) => defmt::info!("No message to read!"),
            //     Err(_) => panic!("Oh no!"),
            // }

            // Mono::delay(Duration::<u64, 1, 32768>::from_ticks(1000)).await;
        }
    }
}