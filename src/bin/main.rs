#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use arcane_node as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [SWI0_EGU0]
)]
mod app {

    use arcane_node::config::Config;
    use arcane_node::mic::Microphone;
    use embedded_hal::can::{Frame, Id, Id::Standard, StandardId};
    use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
    use nrf52840_hal::{
        gpio::{p0, p1, Input, Level, Output, Pin, PullUp, PushPull},
        gpiote::Gpiote,
        pac::SPIM0,
        prelude::*,
        spim, Clocks, Delay, Spim,
    };
    use rtic_monotonics::nrf::timer::{ExtU64, Timer0 as Mono};
    use serde_json_core;

    const GAIN: i8 = -35;
    const SAMPLECOUNT: u16 = 128;

    const CONFIG_JSON: &str = include_str!("../../config.json");

    fn mean(samples: &[i16]) -> f32 {
        let mut avg: f32 = 0.0;
        for s in samples {
            avg += (*s).saturating_abs() as f32;
        }
        avg / (samples.len() as f32)
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        // led: Pin<Output<PushPull>>,
        can: MCP2515<Spim<SPIM0>, p1::P1_08<Output<PushPull>>>,
        config: Config,
    }

    // Local resources go here
    #[local]
    struct Local {
        gpiote: Gpiote,
        // can: MCP2515<Spim<SPIM0>, p1::P1_08<Output<PushPull>>>,
        led_blue: Pin<Output<PushPull>>,
        led_red: Pin<Output<PushPull>>,
        mic: Microphone,
        pdm_buffers: [[i16; SAMPLECOUNT as usize]; 2],
        buf_to_display: usize,
        buf_to_capture: usize,
        spi_int: Pin<Input<PullUp>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Parse the JSON configuration data
        let config: Config = serde_json_core::from_str(CONFIG_JSON).unwrap().0;
        defmt::info!("{:?}", config);
        // set SLEEPONEXIT and SLEEPDEEP bits to enter low power sleep states
        // ctx.core.SCB.set_sleeponexit();
        // ctx.core.SCB.set_sleepdeep();

        // Initialize Monotonic
        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(ctx.device.TIMER0, token);

        // initialize gpio ports
        let port0 = p0::Parts::new(ctx.device.P0);
        let port1 = p1::Parts::new(ctx.device.P1);

        // initialize LED (OFF)
        let led_blue = port1.p1_10.into_push_pull_output(Level::Low).degrade();
        let led_red = port1.p1_15.into_push_pull_output(Level::Low).degrade();

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
                mode: OpMode::Normal,          // Loopback for testing and example
                can_speed: CanSpeed::Kbps1000, // Many options supported.
                mcp_speed: McpSpeed::MHz16,    // Currently 16MHz and 8MHz chips are supported.
                clkout_en: false,
            },
        )
        .unwrap();

        // initiliaze MIC

        let mut mic = Microphone::new(ctx.device.PDM);

        // we need the high frequency oscillator enabled for the microphone
        let c = Clocks::new(ctx.device.CLOCK);
        c.enable_ext_hfosc();

        let pdm_buffers: [[i16; SAMPLECOUNT as usize]; 2] = [[0; SAMPLECOUNT as usize]; 2];

        let buf_to_display: usize = 0;
        let buf_to_capture: usize = 1;

        mic.enable();
        mic.enable_interrupts();
        mic.set_gain(GAIN);

        mic.set_sample_buffer(&pdm_buffers[buf_to_capture]);
        mic.start_sampling();

        // setup GPIOTE peripheral to trigger an interrupt on P1_02
        // high-to-low transition. another option would be to enable SENSE in pin
        // configuration and then use GPIOTE PORT event for detection...
        let _button = port1.p1_02.into_pullup_input().degrade();
        let spi_int = port0.p0_07.into_pullup_input().degrade();

        // read_can_message::spawn();

        let gpiote = Gpiote::new(ctx.device.GPIOTE);

        gpiote
            .channel0()
            .input_pin(&spi_int)
            .hi_to_lo()
            .enable_interrupt();

        (
            Shared {
                // led,
                can,
                config,
            },
            Local {
                gpiote,
                led_blue,
                led_red,
                mic,
                pdm_buffers,
                buf_to_display,
                buf_to_capture,
                spi_int,
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

    #[task(binds = GPIOTE, shared = [can, config] , local = [gpiote, led_red, spi_int, state: bool = false])]
    fn gpiote_event(mut ctx: gpiote_event::Context) {
        if ctx.local.gpiote.channel0().is_event_triggered() {
            ctx.local.gpiote.channel0().reset_events();
            ctx.local.gpiote.channel0().clear();

            // TODO: remove?
            match ctx.local.spi_int.is_high() {
                Ok(_) => defmt::trace!("Received frame"),
                Err(_) => defmt::trace!("Pin is low?"),
                // Err(_) => defmt::trace!("Cannot read pin state"),
            }
            defmt::info!("interrupt from spi!");

            let config = ctx.shared.config.lock(|c| c.clone());

            ctx.shared.can.lock(|c| match c.read_message() {
                Ok(frame) => match frame.id() {
                    Standard(id) if id == StandardId::new(config.id as u16).unwrap() => {
                        defmt::trace!("Message for this node: {:?}", frame)
                        // match against index of param
                    }
                    _ => defmt::trace!("Message for other node"),
                },
                Err(Error::NoMessage) => defmt::trace!("No message to read!"),
                Err(_) => defmt::trace!("Oh no!"),
            });
            match *ctx.local.state {
                true => ctx.local.led_red.set_low().unwrap(),
                false => ctx.local.led_red.set_high().unwrap(),
            }
            *ctx.local.state = !*ctx.local.state;
        }
    }

    #[task(binds = PDM, shared = [config], local = [mic, led_blue, pdm_buffers, buf_to_display, buf_to_capture])]
    fn mic_task(mut ctx: mic_task::Context) {
        let threshold = ctx.shared.config.lock(|c| c.parameters.threshold.value);
        let mean = mean(&ctx.local.pdm_buffers[*ctx.local.buf_to_display]);
        if mean > threshold {
            ctx.local.led_blue.set_high().unwrap();
            defmt::println!("mean: {}", mean);
            send_can_message::spawn()
                .unwrap_or_else(|_| defmt::error!("Cannot spawn task, already running..."));
        } else {
            ctx.local.led_blue.set_low().unwrap();
        }

        (*ctx.local.buf_to_capture, *ctx.local.buf_to_display) = if *ctx.local.buf_to_capture == 0 {
            (1, 0)
        } else {
            (0, 1)
        };

        // Wait for the current buffer capture to complete, then start on the flipped buffer
        while !ctx.local.mic.sampling_started() {}

        ctx.local.mic.clear_sampling_started();
        ctx.local
            .mic
            .set_sample_buffer(&ctx.local.pdm_buffers[*ctx.local.buf_to_capture]);
    }

    #[task(priority = 1, shared = [can, config])]
    async fn send_can_message(mut ctx: send_can_message::Context) {
        defmt::trace!("send note-on");
        let config = ctx.shared.config.lock(|c| c.clone());
        ctx.shared.can.lock(|c| {
            c.send_message(
                CanFrame::new(
                    Id::Standard(StandardId::new((0b0001 as u16) << 7 | config.id as u16).unwrap()),
                    &[0x90, config.parameters.note.value, 0x40],
                )
                .unwrap(),
            )
            .unwrap_or_else(|_| defmt::error!("Something went wrong"))
        });
        Mono::delay(config.parameters.length.value.millis()).await;
        defmt::trace!("send note-off");
        ctx.shared.can.lock(|c| {
            c.send_message(
                CanFrame::new(
                    Id::Standard(StandardId::new((0b0001 as u16) << 7 | config.id as u16).unwrap()),
                    &[0x90, config.parameters.note.value, 0x00],
                )
                .unwrap(),
            )
            .unwrap_or_else(|_| defmt::error!("Something went wrong"))
        });
        Mono::delay(config.parameters.throttle.value.millis()).await;
    }
}
