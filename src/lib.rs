//! This is a driver for the UBLOX NEO-M9N
//!  
//!

#![no_std]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]
// #![deny(clippy::float_arithmetic)]
#![allow(non_snake_case)]

use defmt::Format;
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

mod registers;

pub use registers::*;

/// Errors that can occur when communicating with the BMP390 barometer.
#[derive(Debug, Clone, Copy, Format)]
pub enum Error<E> {
    /// An error occurred while communicating with the BMP390 over I2C. The inner error contains the specific error.
    I2c(E),

    // /// The BMP390's chip ID did not match the expected value of `0x60`. The actual chip ID is provided.
    // WrongChip(u8),

    // /// A fatal error occurred on the BMP390. See [`ErrReg`] for more.
    // Fatal,

    // /// A command error occurred on the BMP390. See [`ErrReg`] for more.
    // Command,

    // /// A configuration error occurred on the BMP390. See [`ErrReg`] for more.
    // Configuration,
}

/// Note: [`embedded_hal_async::i2c::ErrorKind`] is an alias for [`embedded_hal::i2c::ErrorKind`], so the one impl
/// covers both.
impl From<embedded_hal_async::i2c::ErrorKind> for Error<embedded_hal_async::i2c::ErrorKind> {
    fn from(error: embedded_hal_async::i2c::ErrorKind) -> Self {
        Error::I2c(error)
    }
}

#[derive(Debug, Clone, Copy, Format)]
#[repr(u8)]
pub enum Address {
    Default = 0x42,
    Custom(u8),
}

impl From<Address> for u8 {
    /// Convert the address to a [`u8`] for I2C communication.
    fn from(address: Address) -> u8 {
        match address {
            Address::Default => 0x42,
            Address::Custom(addr) => addr & 0xFE, // Ensure the least significant bit is 0
        }
    }
}

impl Address {
    /// Create a new custom address, ensuring the least significant bit is 0.
    pub fn new_custom(addr: u8) -> Self {
        Address::Custom(addr & 0xFE)
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub struct Configuration {
    pub output_nmea: bool,
    pub output_ubx: bool,
    pub output_rtcm: bool,
}

pub struct UBLOX<I, D> {
    /// The I2C bus the barometer is connected to.
    i2c: I,

    /// The I2C address of the barometer.
    address: Address,

    /// The devices delay function.
    delay: D,
}

impl<I, E, D> UBLOX<I, D>
where
    I: I2c<Error = E>,
    D: DelayNs,
{
    pub async fn try_new(
        i2c: I,
        address: Address,
        mut delay: D,
        _config: &Configuration
    ) -> Result<Self, Error<E>> {
        
        delay.delay_ms(250).await;

        Ok(Self { i2c, address, delay })
    }
    
    /// Enable UBX-NAV-UTC messages at 1Hz
    /// 
    /// Note: Leap Seconds are broadcasted every 12.5 minutes, so the time will be off by a few seconds until then.
    /// 
    /// Note: Converting from GPS time to UTC time is not trivial, so it's best to use the GPS's UTC time rather than converting it yourself.
    pub async fn enable_ubx_time_utc(&mut self) -> Result<(), Error<E>> {

        let ubx_cfg_valset_ram: [u8; 48] = [
            0xB5, 0x62, 0x06, 0x8A, 0x28, 0x00, 0x01, 0x01,
            0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x08, 0x06,
            0x00, 0x93, 0x10, 0x01, 0x01, 0x00, 0x21, 0x30,
            0xC8, 0x00, 0x02, 0x00, 0x51, 0x10, 0x01, 0x06,
            0x00, 0x91, 0x20, 0x01, 0x02, 0x00, 0x72, 0x10,
            0x00, 0x5B, 0x00, 0x91, 0x20, 0x01, 0x85, 0x14
        ];
        
        self.i2c
            .write(self.address.into(), &ubx_cfg_valset_ram)
            .await
            .map_err(Error::I2c)?;

        // Wait for the module to process the command, this takes 100ms for any message
        // However the effect of the command is not immediate, and each message has its own delay.
        self.delay.delay_ms(100).await;

        Ok(())
    }

    pub async fn enable_ubx_nav_pvt(&mut self) -> Result<(), Error<E>> {
        // Enable UBX-NAV-PVT messages at 1Hz Hex Code generated from U-Center 2
        // Note: 1Hz is not the real output rate, but the rate that it will output from it's internal navigation filtering.
        // The documentation for the NEO M8 says that if you set it to '2 Hz' it will actually output every other navigation update.
        // So in general 1Hz is the best and only setting you should use.
        // If you want a faster output you need to increase the navigation rate which has a limit of 4Hz. (I tried this but the data was incomplete)

        // This can also be generated from the ublox library, however I'd rather use raw bytes from U-Center for now.
        let ubx_cfg_valset_ram: [u8; 43] = [
            0xB5, 0x62, 0x06, 0x8A, 0x23, 0x00,
            0x01, 0x01, 0x00, 0x00, 0x21, 0x00,
            0x11, 0x20, 0x08, 0x06, 0x00, 0x93,
            0x10, 0x01, 0x01, 0x00, 0x21, 0x30,
            0xC8, 0x00, 0x02, 0x00, 0x51, 0x10,
            0x01, 0x06, 0x00, 0x91, 0x20, 0x01,
            0x02, 0x00, 0x72, 0x10, 0x00, 0x73, 0x48
        ];
        self.i2c
            .write(self.address.into(), &ubx_cfg_valset_ram)
            .await
            .map_err(Error::I2c)?;

        // Wait for the module to process the command, this takes 100ms for any message
        // However the effect of the command is not immediate, and each message has its own delay.
        self.delay.delay_ms(100).await;

        Ok(())
    }

    // This needs to be changed to request the message type, atm it's a copy of get_data
    // pub async fn get_ubx_nav_pvt(&mut self) -> Result<Option<[u8; 92]>, Error<E>> {
    //     // Get the number of bytes available from the module
    //     let mut buffer = [0u8; 2];
    //     self.i2c
    //         .write_read(self.address.into(), &[Register::NUMBER_BYTES_READY_MSB.into()], &mut buffer)
    //         .await
    //         .map_err(Error::I2c)?;

    //     let bytes_available = ((buffer[0] as u16) << 8) | buffer[1] as u16;
    //     if bytes_available == 0 {
    //         return Ok(None);
    //     }

    //     // Read up to 128 bytes at a time
    //     let bytes_to_read = bytes_available.min(128) as usize;

    //     // PVT message is 92 bytes long
    //     let mut data = [0u8; 92];
    //     // self.i2c
    //     //     .write_read(self.address.into(), &[UBX_NAV_PVT[0]], &mut data[0..bytes_to_read as usize])
    //     //     .await
    //     //     .map_err(Error::I2c)?;
    //     self.i2c
    //         .write_read(self.address.into(), &[0xFF], &mut data[0..bytes_to_read])
    //         .await
    //         .map_err(Error::I2c)?;

    //     Ok(Some(data))
    // }

    /// Get's any queued up data from the generic register 0xFF
    /// Data streams must be enabled otherwise the module will not send any data
    /// If you get 0xFF back, then the module is not sending any data or not ready yet so ignore it.
    pub async fn get_data(&mut self) -> Result<Option<[u8; 128]>, Error<E>> {
        // Get the number of bytes available from the module
        let mut buffer = [0u8; 2];
        self.i2c
            .write_read(self.address.into(), &[Register::NUMBER_BYTES_READY_MSB.into()], &mut buffer)
            .await
            .map_err(Error::I2c)?;

        let bytes_available = ((buffer[0] as u16) << 8) | buffer[1] as u16;
        if bytes_available == 0 {
            return Ok(None);
        }

        // Step 2: Determine the number of bytes to read (max 128 bytes per transaction)
        let bytes_to_read = bytes_available.min(128) as usize;

        // 2K buffer
        let mut data = [0u8; 128];
        self.i2c.read(self.address.into(), &mut data[0..bytes_to_read as usize])
            .await
            .map_err(Error::I2c)?;

        // for byte in data {
        //     self.process(byte, incoming_ubx, requested_class, requested_id);
        // }
        Ok(Some(data))
    }

    pub async fn is_connected(&mut self) -> Result<bool, Error<E>> {
        // Send a dummy read to check if the device acknowledges
        let mut buffer = [0u8; 1];
        match self.i2c.read(self.address.into(), &mut buffer).await {
            Ok(_) => Ok(true),
            Err(_) => Ok(false),
        }
    }

    pub async fn check_if_ready(
        &mut self,
    ) -> Result<bool, Error<E>> {
        // Get the number of bytes available from the module
        let mut buffer = [0u8; 2];
        self.i2c
        .write_read(self.address.into(), &[Register::NUMBER_BYTES_READY_MSB.into()], &mut buffer)
        .await
        .map_err(Error::I2c)?;

        let bytes_available = ((buffer[0] as u16) << 8) | buffer[1] as u16;
        if bytes_available == 0 {
            return Ok(false);
        }

        let mut bytes_to_read = bytes_available;
        if bytes_to_read > 32 {
            bytes_to_read = 32;
        }

        // 2K buffer
        let mut data = [0u8; 2048];
        self.i2c.read(self.address.into(), &mut data[0..bytes_to_read as usize])
            .await
            .map_err(Error::I2c)?;

        // for byte in data {
        //     self.process(byte, incoming_ubx, requested_class, requested_id);
        // }
        defmt::info!("Data: {:?}", data);
        Ok(true)
    }
}