#![no_std]
#![cfg_attr(not(test), no_main)]
#![feature(core_intrinsics)]

extern crate avr_config;
//extern crate avr_delay;
extern crate avrd;
extern crate avr_std_stub;

use avrd::atmega64::*;

use core::intrinsics::roundf32;
use core::ptr::{read_volatile, write_volatile};

//use avr_delay::{delay_ms, delay_us};

const SK1_ROM: [u8; 32767] = include!("sk1_rom.in");

struct AVR {
}

/*
 *   pin assignments beginning with _ are vendor-defined
 *
 *   PORTA       PORTB      PORTC        PORTE       PORTF
 *   0: ROM_D0   0: SD_CS   0: ROM_A8    0: _RXD0    0: ROM_A0
 *   1: ROM_D1   1: _SCK    1: ROM_A9    1: _TXD0    1: ROM_A1
 *   2: ROM_D2   2: _MOSI   2: ROM_A10   2:          2: ROM_A2
 *   3: ROM_D3   3: _MISO   3: ROM_A11   3:          3: ROM_A3
 *   4: ROM_D4   4:         4: ROM_A12   4:          4: ROM_A4
 *   5: ROM_D5   5:         5: ROM_A13   5: ROM_WE   5: ROM_A5
 *   6: ROM_D6   6:         6: ROM_A14   6: ROM_OE   6: ROM_A6
 *   7: ROM_D7   7:         7:           7: ROM_CE   7: ROM_A7
 */

/*fn write_byte(byte: u8, addr: u16) {
    let addr_low = (addr & 255) as u8;
    let addr_high = (addr >> 8) as u8;

    unsafe {
        porta::PORTA::as_ptr();
        porta::porta::W::bits(byte);
    }

    unsafe {
        write_volatile(PORTC, addr_high);
        write_volatile(PORTF.PTR, addr_low);
        write_volatile(porta::PORTA::as_ptr(), byte);

        write_volatile(PORTE, 0b_01000000); //CE low, OE high, WE low
        //delay_us(40);
        write_volatile(PORTE, 0b_01100000); //CE low, OE high, WE high
        //delay_us(40);
    }
}*/

impl AVR {
    fn new() -> AVR {
        AVR {
        }
    }

    fn set_addr() {

    }

    fn set_data() {

    }

    fn enable_write() {

    }

    fn enable_select() {

    }

    fn enable_output() {

    }

    fn bus_on(&mut self) {
        unsafe {
            write_volatile(DDRA, 255);
            write_volatile(DDRF, 255);
            write_volatile(DDRC, 255);
        }
    }

    fn bus_off(&mut self) {
        unsafe {
            write_volatile(PORTA, 0);
            write_volatile(DDRA, 0);
            write_volatile(PORTF, 0);
            write_volatile(DDRF, 0);
            write_volatile(PORTC, 0);
            write_volatile(DDRC, 0);
        }
    }

    fn write_byte(&mut self, byte: u8, addr: u16) {
        self.bus_on();
        let addr_low = (addr & 255) as u8;
        let addr_high = (addr >> 8) as u8;

        unsafe {
            write_volatile(PORTC, addr_high);
            write_volatile(PORTF, addr_low);
            write_volatile(PORTA, byte);
        }
    }

    fn init_usart0(&mut self, baud: u32) {
        //we use floats here so that the calculation might be a bit more cycle-accurate
        let ubrr_float = avr_config::CPU_FREQUENCY_HZ as f32 / 16.0 / baud as f32;
        //gotta call llvm intrinsics for rounding because math isn't in core
        let ubrr = unsafe { roundf32(ubrr_float) } as u16 - 1;
        let ubrrl = (ubrr & 255) as u8;
        let ubrrh = (ubrr >> 8) as u8;

        unsafe {
            //set baud rate
            write_volatile(UBRR0H, ubrrh);
            write_volatile(UBRR0L, ubrrl);

            //set 8n1 TODO: make sure this is correct
            write_volatile(UCSR0B, (1 << *RXEN0) | (1 << *TXEN0));
            write_volatile(UCSR0C, (1 << *USBS0) | (1 << *UCSZ0));
        }
    }

    fn send_usart0(&mut self, bytes: &[u8]) {
        unsafe {
            for &byte in bytes {
                while (read_volatile(UCSR0A) & (1 << *UDRE0)) == 0 {}
                write_volatile(UDR0, byte);
                //TODO: set UDRE bit low
            }
        }
    }

    //ATC27256
    fn disable_sdp(&mut self) {
        self.write_byte(0xaa, 0x5555);
        self.write_byte(0x55, 0x2aaa);
        self.write_byte(0x80, 0x5555);
        self.write_byte(0xaa, 0x5555);
        self.write_byte(0x55, 0x2aaa);
        self.write_byte(0x20, 0x5555);
    }

    //ATC27256
    fn erase(&mut self) {
        self.write_byte(0xAA, 0x5555);
        self.write_byte(0x55, 0x2AAA);
        self.write_byte(0x80, 0x5555);
        self.write_byte(0xAA, 0x5555);
        self.write_byte(0x55, 0x2AAA);
        self.write_byte(0x10, 0x5555);
    }
}

#[no_mangle]
//#[cfg(not(test))]
fn main() {
    let mut avr = AVR::new();

    avr.init_usart0(115200);

    unsafe {
        //data + addr
        write_volatile(DDRA, 0b_11111111);
        write_volatile(DDRF, 0b_11111111);
        write_volatile(DDRC, 0b_01111111);

        //control
        write_volatile(DDRB, 0b_00001111); //SD + reset
        write_volatile(DDRE, 0b_11100000); //CE, OE, WE

        write_volatile(PORTB, 0b_00000001); //SD_CS high
        write_volatile(PORTE, 0b_01100000); //CE low, OE high, WE high
    }

    //delay_ms(40);

    let mut addr: u16 = 0;
    for &byte in &SK1_ROM {
        addr += 1;
        avr.write_byte(byte, addr);
    }

    //delay_ms(40);

    //should be programmed, set everything to tri-state with no pull-ups
    unsafe {
        write_volatile(DDRE, 0b_00100000);
        write_volatile(PORTE, 0b_00100000);

        write_volatile(DDRA, 0b_00000000);
        write_volatile(DDRB, 0b_00000000);
        write_volatile(DDRC, 0b_00000000);
        write_volatile(DDRF, 0b_00000000);

        write_volatile(PORTA, 0b_00000000);
        write_volatile(PORTB, 0b_00000000);
        write_volatile(PORTC, 0b_00000000);
        write_volatile(PORTF, 0b_00000000);
    }

    loop {}
}

