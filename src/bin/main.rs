#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use arcane_node as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [SWI0_EGU0]
)]
mod app {
    use arcane_node::mic::Microphone;
    use arcane_node::{config::Config, initialize_can};
    use arcane_node::{initialize_mic, process_cfg, SAMPLECOUNT};
    use embedded_hal::can::{Frame, Id, StandardId};
    use mcp2515::{frame::CanFrame, MCP2515};
    use nrf52840_hal::{
        gpio::{p0, p1, Input, Level, Output, Pin, PullUp, PushPull},
        gpiote::Gpiote,
        pac::SPIM0,
        prelude::*,
        spim, Spim,
    };
    use rtic_monotonics::nrf::timer::{ExtU64, Timer0 as Mono};
    use serde_json_core;

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
        can: MCP2515<Spim<SPIM0>, Pin<Output<PushPull>>>,
        config: Config,
    }

    // Local resources go here
    #[local]
    struct Local {
        gpiote: Gpiote,
        led_blue: Pin<Output<PushPull>>,
        led_red: Pin<Output<PushPull>>,
        mcp_int: Pin<Input<PullUp>>,
        mic: Microphone,
        pdm_buffers: [[i16; SAMPLECOUNT as usize]; 2],
        buf_to_display: usize,
        buf_to_capture: usize,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Parse the JSON configuration data
        let config: Config = serde_json_core::from_str(CONFIG_JSON).unwrap().0;

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
        let cs = port1.p1_08.into_push_pull_output(Level::Low).degrade();
        let mcp_int = port0.p0_07.into_pullup_input().degrade();

        let pins = spim::Pins {
            sck: Some(spiclk),
            miso: Some(spimiso),
            mosi: Some(spimosi),
        };

        let gpiote = Gpiote::new(ctx.device.GPIOTE);

        let can = initialize_can(cs, &mcp_int, pins, ctx.device.SPIM0, &gpiote, ctx.core.SYST);

        let (mic, pdm_buffers, buf_to_display, buf_to_capture) =
            initialize_mic(ctx.device.PDM, ctx.device.CLOCK, &config);

        (
            Shared { can, config },
            Local {
                gpiote,
                led_blue,
                led_red,
                mcp_int,
                mic,
                pdm_buffers,
                buf_to_display,
                buf_to_capture,
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

    #[task(binds = GPIOTE, shared = [can, config], local = [gpiote, led_red, mcp_int, state: bool = false])]
    fn gpiote_event(mut ctx: gpiote_event::Context) {
        if !ctx.local.gpiote.channel0().is_event_triggered() {
            return;
        }

        ctx.local.gpiote.channel0().reset_events();
        ctx.local.gpiote.channel0().clear();

        ctx.shared.config.lock(|config| {
            ctx.shared.can.lock(|can| {
                if let Ok(frame) = can.read_message() {
                    process_cfg(frame, config);
                } else {
                    defmt::trace!("No message to read or something went wrong!");
                }
            });
        });

        // Toggle the LED state
        if *ctx.local.state {
            ctx.local.led_red.set_low().unwrap();
        } else {
            ctx.local.led_red.set_high().unwrap();
        }
        *ctx.local.state = !*ctx.local.state;
    }

    #[task(binds = PDM, shared = [config], local = [mic, led_blue, pdm_buffers, buf_to_display, buf_to_capture])]
    fn mic_task(mut ctx: mic_task::Context) {
        let threshold = ctx.shared.config.lock(|c| c.parameters.threshold.value);
        let mean = mean(&ctx.local.pdm_buffers[*ctx.local.buf_to_display]);
        if mean > threshold {
            ctx.local.led_blue.set_high().unwrap();
            defmt::println!("mean: {}", mean);
            send_can_message::spawn()
                .unwrap_or_else(|_| defmt::trace!("Cannot spawn task, already running..."));
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
        defmt::trace!("config: {:?}", config);
        ctx.shared.can.lock(|c| {
            c.send_message(
                CanFrame::new(
                    Id::Standard(StandardId::new((0b0001 as u16) << 7 | config.id as u16).unwrap()),
                    &[
                        0x90 | (0x0F & config.parameters.channel.value),
                        config.parameters.note.value,
                        0x40,
                    ],
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
                    &[
                        0x90 | (0x0F & config.parameters.channel.value),
                        config.parameters.note.value,
                        0x00,
                    ],
                )
                .unwrap(),
            )
            .unwrap_or_else(|_| defmt::error!("Something went wrong"))
        });
        Mono::delay(config.parameters.throttle.value.millis()).await;
    }
}
