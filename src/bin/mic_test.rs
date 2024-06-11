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
    use arcane_node::{config::Config, initialize_mic};
    use nrf52840_hal::{
        gpio::{p0, p1, Level, Output, Pin, PushPull},
        gpiote::Gpiote,
        prelude::*,
        Clocks,
    };
    use rtic_monotonics::nrf::timer::{ExtU64, Timer0 as Mono};
    use serde_json_core;

    const GAIN: i8 = 0x00;
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
        led: Pin<Output<PushPull>>,
    }

    // Local resources go here
    #[local]
    struct Local {
        gpiote: Gpiote,
        mic: Microphone,
        pdm_buffers: [[i16; SAMPLECOUNT as usize]; 2],
        buf_to_display: usize,
        buf_to_capture: usize,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // Initialize Monotonic
        let token = rtic_monotonics::create_nrf_timer0_monotonic_token!();
        Mono::start(ctx.device.TIMER0, token);

        // initialize gpio port
        let port1 = p1::Parts::new(ctx.device.P1);

        // initialize LED (OFF)
        let led = port1.p1_10.into_push_pull_output(Level::Low).degrade();

        // initiliaze MIC

        let (mic, pdm_buffers, buf_to_display, buf_to_capture) =
            initialize_mic(ctx.device.PDM, ctx.device.CLOCK, &config);

        // setup GPIOTE peripheral to trigger an interrupt on P1_02
        let button = port1.p1_02.into_pullup_input().degrade();
        let gpiote = Gpiote::new(ctx.device.GPIOTE);

        gpiote
            .channel0()
            .input_pin(&button)
            .hi_to_lo()
            .enable_interrupt();

        (
            Shared { led },
            Local {
                gpiote,
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

    #[task(binds = GPIOTE, shared = [led] , local = [gpiote, state: bool = false])]
    fn gpiote_event(mut ctx: gpiote_event::Context) {
        if ctx.local.gpiote.channel0().is_event_triggered() {
            ctx.local.gpiote.channel0().reset_events();

            defmt::info!("Button pressed!");
            match *ctx.local.state {
                true => ctx.shared.led.lock(|l| l.set_low().unwrap()),
                false => ctx.shared.led.lock(|l| l.set_high().unwrap()),
            }
            *ctx.local.state = !*ctx.local.state;
        }
    }

    #[task(binds = PDM, shared = [led], local = [mic, pdm_buffers, buf_to_display, buf_to_capture])]
    fn mic_task(mut ctx: mic_task::Context) {
        defmt::info!("Starting recording task");

        let mean = mean(&ctx.local.pdm_buffers[*ctx.local.buf_to_display]);
        defmt::println!("mean: {}", mean);
        if mean > 2000.00 {
            ctx.shared.led.lock(|l| l.set_high().unwrap());
        } else {
            ctx.shared.led.lock(|l| l.set_low().unwrap());
        }

        (*ctx.local.buf_to_capture, *ctx.local.buf_to_display) = if *ctx.local.buf_to_capture == 0 {
            (1, 0)
        } else {
            (0, 1)
        };

        defmt::info!("waiting for start");
        // Wait for the current buffer capture to complete, then start on the flipped buffer
        while !ctx.local.mic.sampling_started() {}
        defmt::info!("sampling has started");

        ctx.local.mic.clear_sampling_started();
        ctx.local
            .mic
            .set_sample_buffer(&ctx.local.pdm_buffers[*ctx.local.buf_to_capture]);
    }
}
