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
    use arcane_node::{mean, SAMPLECOUNT};
    use nrf52840_hal::{
        gpio::{p1, Level, Output, Pin, PushPull},
        prelude::*,
    };
    use rtic_monotonics::nrf::timer::Timer0 as Mono;
    use serde_json_core;

    const CONFIG_JSON: &str = include_str!("../../config.json");

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        led: Pin<Output<PushPull>>,
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

        // initialize gpio port
        let port1 = p1::Parts::new(ctx.device.P1);

        // initialize LED (OFF)
        let led = port1.p1_10.into_push_pull_output(Level::Low).degrade();

        // initiliaze MIC

        let (mic, pdm_buffers, buf_to_display, buf_to_capture) =
            initialize_mic(ctx.device.PDM, ctx.device.CLOCK, &config);

        (
            Shared {},
            Local {
                led,
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

    #[task(binds = PDM, shared = [], local = [led, mic, pdm_buffers, buf_to_display, buf_to_capture])]
    fn mic_task(ctx: mic_task::Context) {
        defmt::info!("Starting recording task");

        let mean = mean(&ctx.local.pdm_buffers[*ctx.local.buf_to_display]);
        defmt::println!("mean: {}", mean);
        if mean > 2000.00 {
            ctx.local.led.set_high().unwrap();
        } else {
            ctx.local.led.set_low().unwrap();
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
