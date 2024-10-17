//! Receive bytes over nRF24L01.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use cortex_m_rt::entry;
use heapless::{spsc::Queue, Vec};
use nrf24l01_commands::{commands, commands::Command, registers};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32u5::stm32u575::{interrupt, Interrupt, Peripherals, EXTI, GPIOA, SPI1, USART2};

const RX_ADDR: u64 = 0xA2891FFF6A;

struct SyncUnsafeCell<P>(UnsafeCell<Option<P>>);

impl<P> SyncUnsafeCell<P> {
    const fn new() -> Self {
        SyncUnsafeCell(UnsafeCell::new(None))
    }

    fn set(&self, inner: P) {
        unsafe { *self.0.get() = Some(inner) };
    }

    #[allow(clippy::mut_from_ref)]
    fn get(&self) -> &mut P {
        unsafe { &mut *self.0.get() }.as_mut().unwrap()
    }
}

// SAFETY: CPU is single-threaded. Interrupts cannot execute simultaneously and cannot
// preempt each other (all interrupts have same priority).
unsafe impl Sync for SyncUnsafeCell<GPIOA> {}
unsafe impl Sync for SyncUnsafeCell<USART2> {}
unsafe impl Sync for SyncUnsafeCell<SPI1> {}
unsafe impl Sync for SyncUnsafeCell<EXTI> {}
unsafe impl Sync for SyncUnsafeCell<Queue<u8, 64>> {}
unsafe impl Sync for SyncUnsafeCell<Queue<u16, 64>> {}
unsafe impl Sync for SyncUnsafeCell<Queue<Vec<u8, 64>, 16>> {}

static GPIOA_PERIPHERAL: SyncUnsafeCell<GPIOA> = SyncUnsafeCell::new();
static USART2_PERIPHERAL: SyncUnsafeCell<USART2> = SyncUnsafeCell::new();
static SPI1_PERIPHERAL: SyncUnsafeCell<SPI1> = SyncUnsafeCell::new();
static EXTI_PERIPHERAL: SyncUnsafeCell<EXTI> = SyncUnsafeCell::new();
/// Bytes received over SPI1
static RX_BUFFER: SyncUnsafeCell<Queue<u16, 64>> = SyncUnsafeCell::new();

fn send_command(
    command_bytes: &[u8],
    spi1: &mut SPI1,
    usart2: &mut USART2,
    rx_buffer: &mut Queue<u16, 64>,
) {
    /* SPI Transmit procedure:
       TSIZE must be written before SPE=1, p.2924: SPI must be disabled while changing TSIZE
       SPE=1
       Push bytes to TXFIFO through TXDR, p.2908: FIFOs cannot be accessed with SPE=0
       TXTF gets set after bytes are written to TXFIFO, clear it
       CSTART=1
       Poll EOT=1, CSTART is reset when EOT is set
       Read TSIZE bytes out of RXFIFO
       SPE=0
    */

    spi1.spi_cr2()
        .write(|w| unsafe { w.tsize().bits(command_bytes.len() as u16) });
    spi1.spi_cr1().modify(|_, w| w.spe().set_bit());
    spi1.spi_cr1().modify(|_, w| w.cstart().set_bit());
    for &byte in command_bytes {
        while spi1.spi_sr().read().txp().bit_is_clear() {}
        spi1.txdr8().write(|w| unsafe { w.txdr().bits(byte) });

        while spi1.spi_sr().read().rxp().bit_is_clear() {}
        let received_byte = spi1.rxdr8().read().rxdr().bits();
        let _ = rx_buffer.enqueue(received_byte as u16);
    }

    spi1.spi_ifcr().write(|w| w.txtfc().set_bit());
    // while spi1.spi_sr().read().eot().bit_is_clear() {}
    spi1.spi_cr1().modify(|_, w| w.spe().clear_bit());
    usart2.cr1_disabled().modify(|_, w| w.txfnfie().set_bit());
}

#[interrupt]
fn USART2() {
    let usart2 = USART2_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let gpioa = GPIOA_PERIPHERAL.get();
    let rx_buffer = RX_BUFFER.get();

    // Dequeue bytes off rx buffer and transmit over USART2
    if usart2.isr_disabled().read().txfnf().bit_is_set() {
        if let Some(byte) = rx_buffer.dequeue() {
            usart2.tdr().write(|w| unsafe { w.tdr().bits(byte) });
            if rx_buffer.is_empty() {
                usart2.cr1_disabled().modify(|_, w| w.txfnfie().clear_bit());
            }
        } else {
            usart2.cr1_disabled().modify(|_, w| w.txfnfie().clear_bit());
        }
    }

    // Read incoming bytes from USART2 run nRF24L01 command
    if usart2.isr_disabled().read().rxfne().bit_is_set() {
        // Read data, this clears RXNE
        let received_byte = usart2.rdr().read().rdr().bits();

        match received_byte {
            97 => {
                // a
                // NOP
                send_command(&[commands::Nop::WORD], spi1, usart2, rx_buffer);
            }
            98 => {
                // b
                // Read RF_CH
                send_command(
                    &commands::RRegister::<registers::RfCh>::bytes(),
                    spi1,
                    usart2,
                    rx_buffer,
                );
            }
            99 => {
                // c
                // Read RX Addr P0
                send_command(
                    &commands::RRegister::<registers::RxAddrP0<5>>::bytes(),
                    spi1,
                    usart2,
                    rx_buffer,
                );
            }
            100 => {
                // d
                // Read Config
                send_command(
                    &commands::RRegister::<registers::Config>::bytes(),
                    spi1,
                    usart2,
                    rx_buffer,
                );
            }
            101 => {
                // e
                // Clear RX_DR flag
                send_command(
                    &commands::WRegister(registers::Status::new().with_rx_dr(true)).bytes(),
                    spi1,
                    usart2,
                    rx_buffer,
                );
            }
            102 => {
                // f
                // Read pipe 0 payload width
                send_command(
                    &commands::RRegister::<registers::RxPwP0>::bytes(),
                    spi1,
                    usart2,
                    rx_buffer,
                );
            }
            103 => {
                // g
                // Read FifoStatus
                send_command(
                    &commands::RRegister::<registers::FifoStatus>::bytes(),
                    spi1,
                    usart2,
                    rx_buffer,
                );
            }
            104 => {
                // h
                // CE=1
                gpioa.bsrr().write(|w| w.bs0().set_bit());
            }
            105 => {
                // i
                // CE=0
                gpioa.bsrr().write(|w| w.br0().set_bit());
            }
            106 => {
                // j
                // read payload
                send_command(
                    &commands::RRxPayload::<32>::bytes(),
                    spi1,
                    usart2,
                    rx_buffer,
                );
            }
            107 => {
                // k
                // carrier detect
                send_command(
                    &commands::RRegister::<registers::Cd>::bytes(),
                    spi1,
                    usart2,
                    rx_buffer,
                );
            }
            _ => (),
        }
    }
    if usart2.isr_disabled().read().ore().bit_is_set() {
        usart2.icr().write(|w| w.orecf().set_bit());
    }
}

#[interrupt]
fn EXTI1() {
    let exti = EXTI_PERIPHERAL.get();
    let usart2 = USART2_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let rx_buffer = RX_BUFFER.get();
    if exti.fpr1().read().fpif1().bit_is_set() {
        // read payload
        send_command(
            &commands::RRxPayload::<32>::bytes(),
            spi1,
            usart2,
            rx_buffer,
        );
        // Clear RX_DR flag
        send_command(
            &commands::WRegister(registers::Status::new().with_rx_dr(true)).bytes(),
            spi1,
            usart2,
            rx_buffer,
        );
        exti.fpr1().write(|w| w.fpif1().clear_bit_by_one());
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = Peripherals::take().unwrap();

    // Enable peripheral clocks - GPIOA, USART2, SPI1
    dp.RCC.ahb2enr1().write(|w| w.gpioaen().enabled());
    dp.RCC.apb1enr1().write(|w| w.usart2en().enabled());
    dp.RCC.apb2enr().write(|w| w.spi1en().enabled());

    // GPIO: A1 (IRQ), A0 (CE)
    // USART2: A2 (TX), A3 (RX) as AF 7
    // SPI1: A4 (NSS), A5 (SCK), A6 (MISO), A7 (MOSI) as AF 5
    dp.GPIOA.moder().write(|w| {
        w.mode0()
            .output()
            .mode1()
            .input()
            .mode2()
            .alternate()
            .mode3()
            .alternate()
            .mode4()
            .alternate()
            .mode5()
            .alternate()
            .mode6()
            .alternate()
            .mode7()
            .alternate()
    });
    dp.GPIOA.otyper().write(|w| w.ot0().push_pull());
    dp.GPIOA.bsrr().write(|w| w.br0().set_bit());
    // NSS, IRQ are active low
    dp.GPIOA
        .pupdr()
        .write(|w| w.pupd1().pull_up().pupd4().pull_up());
    dp.GPIOA.ospeedr().write(|w| {
        w.ospeed2()
            .very_high_speed()
            .ospeed3()
            .very_high_speed()
            .ospeed4()
            .very_high_speed()
            .ospeed5()
            .very_high_speed()
            .ospeed6()
            .very_high_speed()
            .ospeed7()
            .very_high_speed()
    });
    dp.GPIOA.afrl().write(|w| {
        w.afsel2()
            .af7()
            .afsel3()
            .af7()
            .afsel4()
            .af5()
            .afsel5()
            .af5()
            .afsel6()
            .af5()
            .afsel7()
            .af5()
    });

    // USART2: Configure baud rate 9600
    dp.USART2.brr().write(|w| unsafe { w.bits(417) }); // 4Mhz / 9600 approx. 417

    dp.SPI1
        .spi_cfg2()
        .write(|w| w.ssoe().set_bit().master().set_bit());

    // Enable USART, transmitter, receiver and RXNE interrupt
    dp.USART2.cr1_disabled().write(|w| {
        w.re()
            .set_bit()
            .te()
            .set_bit()
            .ue()
            .set_bit()
            .rxfneie()
            .set_bit()
    });

    // Set up EXTI line 1 interrupt for A1
    dp.EXTI.exticr1().write(|w| w.exti1().pa());
    dp.EXTI.ftsr1().write(|w| w.ft1().set_bit());
    dp.EXTI.imr1().write(|w| w.im1().set_bit());

    RX_BUFFER.set(Queue::new());
    GPIOA_PERIPHERAL.set(dp.GPIOA);
    SPI1_PERIPHERAL.set(dp.SPI1);
    USART2_PERIPHERAL.set(dp.USART2);
    EXTI_PERIPHERAL.set(dp.EXTI);
    unsafe {
        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
    }

    // Send initial commands
    let spi1 = SPI1_PERIPHERAL.get();
    let usart2 = USART2_PERIPHERAL.get();
    let gpioa = GPIOA_PERIPHERAL.get();
    let rx_buffer = RX_BUFFER.get();

    send_command(
        &commands::WRegister(registers::RfCh::new().with_rf_ch(110)).bytes(),
        spi1,
        usart2,
        rx_buffer,
    );
    send_command(
        &commands::RRegister::<registers::RfCh>::bytes(),
        spi1,
        usart2,
        rx_buffer,
    );
    send_command(
        &commands::WRegister(registers::RxAddrP0::<5>::new().with_rx_addr_p0(RX_ADDR)).bytes(),
        spi1,
        usart2,
        rx_buffer,
    );
    send_command(
        &commands::RRegister::<registers::RxAddrP0<5>>::bytes(),
        spi1,
        usart2,
        rx_buffer,
    );
    send_command(
        &commands::WRegister(registers::RxPwP0::new().with_rx_pw_p0(32)).bytes(),
        spi1,
        usart2,
        rx_buffer,
    );
    send_command(
        &commands::RRegister::<registers::RxPwP0>::bytes(),
        spi1,
        usart2,
        rx_buffer,
    );
    send_command(
        &commands::WRegister(
            registers::Config::new()
                .with_prim_rx(true)
                .with_pwr_up(true)
                .with_mask_max_rt(true)
                .with_mask_tx_ds(true),
        )
        .bytes(),
        spi1,
        usart2,
        rx_buffer,
    );
    send_command(
        &commands::RRegister::<registers::Config>::bytes(),
        spi1,
        usart2,
        rx_buffer,
    );
    gpioa.bsrr().write(|w| w.bs0().set_bit());

    #[allow(clippy::empty_loop)]
    loop {}
}
