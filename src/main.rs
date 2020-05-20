#![no_std]
#![no_main]
#![feature(proc_macro_hygiene)]

use core::panic::PanicInfo;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::WriteIter;
use heapless::Vec;
use hifive1::hal::clock::Clocks;
use hifive1::hal::delay::Delay;
use hifive1::hal::gpio::{
    gpio0::{Pin10, Pin9},
    Floating, Input, Invert, Output, Regular,
};
use hifive1::hal::prelude::*;
use hifive1::hal::spi::{Spi, SpiX, MODE_0};
use hifive1::hal::DeviceResources;
use hifive1::{pin, sprintln};
//use libesp_err_sys::
use riscv_rt::entry;

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    sprintln!("panic: {}", info);
    loop {
        use core::sync::atomic;
        use core::sync::atomic::Ordering;
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

#[derive(Debug)]
enum EspError {
    ProtocolError,
    BufferOverflow,
    WouldBlock,
}

struct EspWiFi<SPI, PINS> {
    spi: Spi<SPI, PINS>,
    handshake: Pin10<Input<Floating>>,
    delay: FastDelay,
}

impl<SPI: SpiX, PINS> EspWiFi<SPI, PINS> {
    fn send_bytes(&mut self, bytes: &[u8]) {
        self.delay.delay_us(18u32);
        self.spi.write(bytes).unwrap();
        self.delay.delay_us(5000u32);
    }

    fn transfer(&mut self, buffer: &mut [u8]) {
        self.delay.delay_us(18u32);
        self.spi.transfer(buffer).unwrap();
        self.delay.delay_us(5000u32);
    }

    fn discard(&mut self, size: usize) {
        self.delay.delay_us(18u32);
        self.spi.write_iter((0..size).map(|_| 0x00)).unwrap();
        self.delay.delay_us(5000u32);
    }

    pub fn send(&mut self, s: &str) {
        let bytes = s.as_bytes();
        let length = bytes.len();
        assert!(length <= 125);
        let mut newbytes = [0u8; 128];
        for i in 0..length {
            newbytes[i] = bytes[i];
        }
        newbytes[length] = b'\r';
        newbytes[length + 1] = b'\n';
        sprintln!(
            "> {}",
            core::str::from_utf8(&newbytes[0..length + 2]).unwrap()
        );
        self.send_bytes(&[0x02, 0x00, 0x00, 0x00]);
        self.send_bytes(&[(bytes.len() + 2) as u8, 0x00, 0x00, 0x41]);
        self.send_bytes(&newbytes);
    }

    pub fn recv<'a>(&mut self, buffer: &'a mut [u8]) -> Result<&'a str, EspError> {
        if self.handshake.is_low().unwrap() {
            return Err(EspError::WouldBlock);
        }

        self.send_bytes(&[0x01, 0x00, 0x00, 0x00]);

        let mut request = [0u8; 4];
        self.transfer(&mut request);
        if request[3] != 0x42 {
            return Err(EspError::ProtocolError);
        }

        let n = (request[0] & 0x7F) as usize + ((request[1] as usize) << 7);
        if n > buffer.len() {
            self.discard(n);
            return Err(EspError::BufferOverflow);
        }

        self.transfer(&mut buffer[..n]);
        Ok(core::str::from_utf8(&buffer[..n]).unwrap())
    }
}

struct FastDelay {
    us_cycles: u64,
}

impl FastDelay {
    pub fn new(clocks: Clocks) -> Self {
        Self {
            us_cycles: clocks.coreclk().0 as u64 * 3 / 2_000_000,
        }
    }
}

impl DelayUs<u32> for FastDelay {
    fn delay_us(&mut self, us: u32) {
        use riscv::register::mcycle;

        let t = mcycle::read64() + self.us_cycles * (us as u64);
        while mcycle::read64() < t {}
    }
}

#[entry]
fn main() -> ! {
    let dr = DeviceResources::take().unwrap();
    let p = dr.peripherals;
    let gpio = dr.pins;

    // Configure clocks
    let clocks = hifive1::clock::configure(p.PRCI, p.AONCLK, 50.mhz().into());

    // Configure UART for stdout
    hifive1::stdout::configure(
        p.UART0,
        pin!(gpio, uart0_tx),
        pin!(gpio, uart0_rx),
        115_200.bps(),
        clocks,
    );

    // Configure SPI pins
    let mosi = pin!(gpio, spi0_mosi).into_iof0();
    let miso = pin!(gpio, spi0_miso).into_iof0();
    let sck = pin!(gpio, spi0_sck).into_iof0();
    let cs = pin!(gpio, spi0_ss2).into_iof0();

    // Configure SPI
    let pins = (mosi, miso, sck, cs);
    let spi = Spi::new(p.QSPI1, pins, MODE_0, 80000.hz(), clocks);

    let handshake = gpio.pin10.into_floating_input();
    let mut wifi = EspWiFi {
        spi,
        handshake,
        delay: FastDelay::new(clocks),
    };

    sprintln!("WiFi Test");

    Delay.delay_ms(10u32);

    let mut buffer = [0u8; 256];

    wifi.send("AT+CWMODE=1");
    Delay.delay_ms(20u32);
    sprintln!("resp: {:?}", wifi.recv(&mut buffer));

    let mut buf = [0u8; 128];
    let mut len: usize = 0;
    loop {
        let read = hifive1::stdout::read_noblock(&mut buf, &mut len);
        if read > 0 {
            if buf[len - 1] == b'\n' || buf[len - 1] == b'\r' {
                // got a line!
                len -= 1;
                if len == 0 {
                    continue;
                }
                if buf[len - 1] == b'\r' {
                    // remove any \r, a CRLF will be added in a bit
                    len -= 1;
                }
                if let Ok(command) = core::str::from_utf8(&buf[..len]) {
                    sprintln!("Command: '{}'", command);
                    if command.len() == 0 {
                        continue;
                    }
                    wifi.send(command);
                    len = 0;
                }
            }
        }
        match wifi.recv(&mut buffer) {
            Ok(x) => sprintln!("< {}", x),
            Err(EspError::WouldBlock) => (),
            Err(e) => sprintln!("Err: {:?}", e),
        }
    }
}

/* AT error codes

typedef enum {
    ESP_AT_SUB_OK                       = 0x00,              /*!< OK */
    ESP_AT_SUB_COMMON_ERROR             = 0x01,
    ESP_AT_SUB_NO_TERMINATOR            = 0x02,              /*!<  not end with "\r\n" */
    ESP_AT_SUB_NO_AT                    = 0x03,              /*!<  not found AT or at or At or aT */
    ESP_AT_SUB_PARA_LENGTH_MISMATCH     = 0x04,              /*!<  parameter length not match */
    ESP_AT_SUB_PARA_TYPE_MISMATCH       = 0x05,              /*!<  parameter length not match */
    ESP_AT_SUB_PARA_NUM_MISMATCH        = 0x06,              /*!<  parameter number not match */
    ESP_AT_SUB_PARA_INVALID             = 0x07,
    ESP_AT_SUB_PARA_PARSE_FAIL          = 0x08,              /*!<  parse parameter fail */
    ESP_AT_SUB_UNSUPPORT_CMD            = 0x09,
    ESP_AT_SUB_CMD_EXEC_FAIL            = 0x0A,
    ESP_AT_SUB_CMD_PROCESSING           = 0x0B,              /*!<  previous command is processing */
    ESP_AT_SUB_CMD_OP_ERROR             = 0x0C,
} esp_at_error_code;
*/
