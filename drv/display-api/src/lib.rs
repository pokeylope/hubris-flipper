// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the Display driver.

#![no_std]

use derive_idol_err::IdolError;
use embedded_graphics::{pixelcolor, prelude::*, primitives::Rectangle};
use userlib::*;
use zerocopy::{AsBytes, FromBytes};

pub type DisplayColor = pixelcolor::BinaryColor;
pub type DisplayColorRaw = <DisplayColor as PixelColor>::Raw;
pub type DisplayColorStorage = <DisplayColorRaw as RawData>::Storage;

#[derive(Clone, Copy, Default, AsBytes, FromBytes)]
#[repr(packed)]
pub struct DisplayPoint {
    pub x: i32,
    pub y: i32,
}

impl From<Point> for DisplayPoint {
    fn from(p: Point) -> Self {
        DisplayPoint { x: p.x, y: p.y }
    }
}

impl From<DisplayPoint> for Point {
    fn from(p: DisplayPoint) -> Self {
        Point::new(p.x, p.y)
    }
}

#[derive(Clone, Copy, Default, AsBytes, FromBytes)]
#[repr(packed)]
pub struct DisplayPixel(DisplayPoint, DisplayColorStorage);

impl From<Pixel<DisplayColor>> for DisplayPixel {
    fn from(p: Pixel<DisplayColor>) -> Self {
        DisplayPixel(p.0.into(), p.1.into_storage())
    }
}

impl From<&DisplayPixel> for Pixel<DisplayColor> {
    fn from(p: &DisplayPixel) -> Self {
        Pixel(p.0.into(), DisplayColorRaw::from(p.1).into())
    }
}

#[derive(Clone, Copy, AsBytes, FromBytes)]
#[repr(packed)]
pub struct DisplaySize {
    pub width: u32,
    pub height: u32,
}

impl From<Size> for DisplaySize {
    fn from(size: Size) -> Self {
        DisplaySize {
            width: size.width,
            height: size.height,
        }
    }
}

impl From<DisplaySize> for Size {
    fn from(size: DisplaySize) -> Self {
        Size::new(size.width, size.height)
    }
}

#[derive(Clone, Copy, AsBytes, FromBytes)]
#[repr(packed)]
pub struct DisplayRectangle {
    pub top_left: DisplayPoint,
    pub size: DisplaySize,
}

impl From<&Rectangle> for DisplayRectangle {
    fn from(rect: &Rectangle) -> Self {
        DisplayRectangle { top_left: rect.top_left.into(), size: rect.size.into() }
    }
}

impl From<DisplayRectangle> for Rectangle {
    fn from(rect: DisplayRectangle) -> Self {
        Rectangle::new(rect.top_left.into(), rect.size.into())
    }
}

#[derive(Copy, Clone, Debug, FromPrimitive, IdolError)]
pub enum DisplayError {
    SomeError = 1,
}

pub struct RemoteDisplay {
    display: Display,
    size: Size,
}

impl RemoteDisplay {
    pub fn new(display: Display) -> Self {
        let size: Size = display.size().into();
        Self { display, size }
    }

    pub fn flush(&self) -> Result<(), DisplayError> {
        self.display.flush()
    }
}

impl DrawTarget for RemoteDisplay {
    type Color = DisplayColor;

    type Error = DisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let mut buf = [DisplayPixel::default(); 128];
        let mut count = 0;

        for pixel in pixels.into_iter() {
            buf[count] = pixel.into();
            count += 1;

            if count >= buf.len() {
                self.display.draw_pixels(&buf)?;
                count = 0;
            }
        }
        if count != 0 {
            self.display.draw_pixels(&buf[0..count])?;
        }

        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.display.fill_solid(area.into(), color.into_storage())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.display.clear(color.into_storage())
    }
}

impl OriginDimensions for RemoteDisplay {
    fn size(&self) -> Size {
        self.size
    }
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
