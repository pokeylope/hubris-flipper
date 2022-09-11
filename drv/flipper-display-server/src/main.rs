// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_display_api::*;
use drv_spi_api::Spi;
use drv_stm32xx_sys_api::*;
use idol_runtime::{Leased, RequestError, R, LenLimit};
use st7565r::{Mode, St7565r};
use userlib::*;

use embedded_graphics::prelude::*;

const DISPLAY_RST_N: PinSet = Port::B.pin(0);
const DISPLAY_DI: PinSet = Port::B.pin(1);

task_slot!(SPI, spi2_driver);
task_slot!(SYS, sys);

#[export_name = "main"]
fn main() -> ! {
    let sys = Sys::from(SYS.get_task_id());

    sys.gpio_configure_output(
        DISPLAY_RST_N,
        OutputType::PushPull,
        Speed::Low,
        Pull::None,
    )
    .unwrap();
    sys.gpio_configure_output(
        DISPLAY_DI,
        OutputType::PushPull,
        Speed::Low,
        Pull::None,
    )
    .unwrap();
    sys.gpio_set(DISPLAY_RST_N).unwrap();

    let spi = Spi::from(SPI.get_task_id()).device(0);
    let toggle_di = |mode| match mode {
        Mode::Command => sys.gpio_reset(DISPLAY_DI).unwrap(),
        Mode::Data => sys.gpio_set(DISPLAY_DI).unwrap(),
    };
    let mut st7565r = St7565r::new(spi, toggle_di);
    st7565r.initialize().unwrap();

    let mut incoming = [0u8; INCOMING_SIZE];
    let mut server = DisplayServer { st7565r };
    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

struct DisplayServer<F>
where
    F: Fn(Mode),
{
    st7565r: St7565r<F>,
}

impl<F> InOrderDisplayImpl for DisplayServer<F>
where
    F: Fn(Mode),
{
    fn draw_pixels(
        &mut self,
        _msg: &userlib::RecvMessage,
        pixels: LenLimit<Leased<R, [DisplayPixel]>, 128>,
    ) -> Result<(), RequestError<DisplayError>> {
        let mut buf = [DisplayPixel::default(); 128];
        if let Ok(_) = pixels.read_range(0..pixels.len(), &mut buf) {
            self.st7565r
                .display
                .draw_iter(buf[0..pixels.len()].iter().map(|p| p.into()))
                .map_err(|_| DisplayError::SomeError)?;
        }
        Ok(())
    }

    fn fill_solid(
        &mut self,
        _msg: &userlib::RecvMessage,
        area: DisplayRectangle,
        color: DisplayColorStorage,
    ) -> Result<(), idol_runtime::RequestError<DisplayError>> {
        self.st7565r
            .display
            .fill_solid(&area.into(), DisplayColorRaw::from(color).into())
            .map_err(|_| DisplayError::SomeError.into())
    }

    fn clear(
        &mut self,
        _msg: &userlib::RecvMessage,
        color: DisplayColorStorage,
    ) -> Result<(), idol_runtime::RequestError<DisplayError>> {
        self.st7565r
            .display
            .clear(DisplayColorRaw::from(color).into())
            .map_err(|_| DisplayError::SomeError.into())
    }

    fn flush(
        &mut self,
        _msg: &userlib::RecvMessage,
    ) -> Result<(), RequestError<DisplayError>> {
        self.st7565r
            .flush()
            .map_err(|_| DisplayError::SomeError.into())
    }

    fn size(
        &mut self,
        _msg: &userlib::RecvMessage,
    ) -> Result<
        DisplaySize,
        idol_runtime::RequestError<core::convert::Infallible>,
    > {
        Ok(self.st7565r.display.size().into())
    }
}

include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
