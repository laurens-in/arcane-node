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
        pdm: PDM,
        can: MCP2515<Spim<SPIM0>, p1::P1_08<Output<PushPull>>>,
        data: &'static [u8; 128],
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

        // intialize PDM
        let _clk: Pin<Output<PushPull>> = port1.p1_09.into_push_pull_output(Level::Low).degrade();
        let _dat: Pin<Input<Floating>> = port0.p0_08.into_floating_input().degrade();

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

        static data: [u8; 128] = [0; 128];

        // let pdm = p.PDM; // ASK: why not??
        let pdm = ctx.device.PDM;

        let mic = Microphone::new(pdm);

        (
            Shared { led },
            Local {
                gpiote,
                pdm,
                can,
                data: &data,
            },
        )
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

    #[task(priority = 1, shared = [led], local = [pdm, data])]
    async fn pdm_read(ctx: pdm_read::Context) {
        loop {
            let started = ctx.local.pdm.events_started.read().bits();
            if started != 0 {
                defmt::trace!("started {:?}", started);
                ctx.local
                    .pdm
                    .sample
                    .ptr
                    .write(|w| unsafe { w.sampleptr().bits(ctx.local.data.as_ptr() as u32) });
            }
            let ended = ctx.local.pdm.events_end.read().bits();
            if ended != 0 {
                defmt::trace!("ended {:?}", ended);

                defmt::trace!(
                    "data {:?}",
                    // u16::from_le_bytes([ctx.local.data[0], ctx.local.data[1]])
                    ctx.local.data
                );

                // reset
                // ctx.local.pdm.events_end.write(|w| w);
            }
        }
    }

    #[task(priority = 1, shared = [], local = [can])]
    async fn can_test(ctx: can_test::Context) {
        loop {
            // Send a message
            let frame = CanFrame::new(
                Id::Standard(StandardId::new(0b0000 + 0b000000).unwrap()),
                &[0x90, 0x3C, 0x40], // MIDI Note-On
            )
            .unwrap();

            ctx.local.can.send_message(frame).unwrap();

            defmt::info!("Sent message");

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
