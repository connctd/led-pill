#![no_main]
#![no_std]

extern crate stm32f1xx_hal as hal;
extern crate panic_semihosting;

use cortex_m_semihosting::hprintln;
use cortex_m::asm::delay;
use rtfm::app;

use hal::gpio::gpiob::*;
use hal::gpio::*;
use hal::prelude::*;
use hal::spi::Spi;
use hal::time::MegaHertz;
use ws2812::Ws2812;
use ws2812_spi as ws2812;

use stm32_usbd::{UsbBus, UsbBusType};
use usb_device::bus;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    static mut LED_COUNT: usize = 8;
    static mut LEDS: Ws2812<
        Spi<
            hal::pac::SPI1,
            (
                PB3<Alternate<PushPull>>,
                PB4<Input<Floating>>,
                PB5<Alternate<PushPull>>,
            ),
        >,
    > = ();

    static mut USB_DEV: UsbDevice<'static, UsbBusType> = ();
    static mut SERIAL: SerialPort<'static, UsbBusType> = ();

    #[init]
    fn init() -> init::LateResources {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

        // Cortex-M peripherals
        let _core: rtfm::Peripherals = core;
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        // Init USB serial
        hprintln!("Initialising USB serial device").unwrap();
        assert!(clocks.usbclk_valid());

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        *USB_BUS = Some(UsbBus::new(device.USB, (usb_dm, usb_dp)));
        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());
        let usb_dev =
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .max_power(500)
                .build();


        hprintln!("Initialising Ws2812 LEDs").unwrap();
        let mut portb = device.GPIOB.split(&mut rcc.apb2);

        // Setup SPI1
        // MOSI PB5
        // MISO PB4
        // SCK PB3
        let mosi = portb.pb5.into_alternate_push_pull(&mut portb.crl);
        let miso = portb.pb4.into_floating_input(&mut portb.crl);
        let sck = portb.pb3.into_alternate_push_pull(&mut portb.crl);
        /*let spi_mode = Mode{
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };*/
        let spi_freq: MegaHertz = 3.mhz();
        let mut spi = Spi::spi1(
            device.SPI1,
            (sck, miso, mosi),
            &mut afio.mapr,
            ws2812::MODE,
            spi_freq,
            clocks,
            &mut rcc.apb2,
        );

        let mut leds = Ws2812::new(spi);
        
        init::LateResources { LEDS: leds, USB_DEV: usb_dev, SERIAL: serial }
    }

    #[interrupt(resources = [USB_DEV, SERIAL])]
    fn USB_HP_CAN_TX() {
        usb_poll(&mut resources.USB_DEV, &mut resources.SERIAL);
    }

    #[interrupt(resources = [USB_DEV, SERIAL])]
    fn USB_LP_CAN_RX0() {
        usb_poll(&mut resources.USB_DEV, &mut resources.SERIAL);
    }
};

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }

            serial.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}
