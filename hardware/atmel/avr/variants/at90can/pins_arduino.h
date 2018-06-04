/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

/*
  Modified by Atlantis Specialist Technologies
  by James Blakey-Milner, 1 Aug 2017.
  Added support for the AST CAN485 board
  Note! AST modifications are in early development (alpha)
  and are likely to change without notice.
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            53
#define NUM_ANALOG_INPUTS           8 // port F
#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 45 : -1) // I think this is just supposed to map the numbers 0...3 to the purely ADC ports (port F)

#define digitalPinHasPWM(p)         ((p) == 12 || (p) == 13 || (p) == 14 || (p) == 15 || (p) == 35 || (p) == 36 || (p) == 37)

// PB0 - PB3
#define PIN_SPI_SS    (8)
#define PIN_SPI_MOSI  (10)
#define PIN_SPI_MISO  (11)
#define PIN_SPI_SCK   (9)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// PD0 and PD1
#define PIN_WIRE_SDA        (25)
#define PIN_WIRE_SCL        (24)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define LED_BUILTIN 13 // what should go here? (since there's no built in LED on the at90can)

// analog pins
#define PIN_A0   (45)
#define PIN_A1   (46)
#define PIN_A2   (47)
#define PIN_A3   (48)
#define PIN_A4   (49)
#define PIN_A5   (50)
#define PIN_A6   (51)
#define PIN_A7   (52)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;

// pin change interrupts not supported (CAN485)
// these probably aren't necessary so we probably don't need to port them over to the at90can
#define digitalPinToPCICR(p)    ((uint8_t *)0)
#define digitalPinToPCICRbit(p) (NO_PIN_CHANGE_INTERRUPT)
#define digitalPinToPCMSK(p)    ((uint8_t *)0)
#define digitalPinToPCMSKbit(p) (NO_PIN_CHANGE_INTERRUPT_MASK)

// matches PD0 - PD3 ====> INT0 - INT3 (respectively)
#define digitalPinToInterrupt(p)  (((p) >= 24 && (p) <= 27) ? ((p) - 24) : NOT_AN_INTERRUPT)


#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)

/*
 * See Arduino.h for use of these
 * ports start with PA = 1 (not 0)
 */
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
    (uint16_t) &DDRA,
    (uint16_t) &DDRB,
    (uint16_t) &DDRC,
    (uint16_t) &DDRD,
    (uint16_t) &DDRE,
    (uint16_t) &DDRF,
    (uint16_t) &DDRG
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
    (uint16_t) &PORTA,
    (uint16_t) &PORTB,
    (uint16_t) &PORTC,
    (uint16_t) &PORTD,
    (uint16_t) &PORTE,
    (uint16_t) &PORTF,
    (uint16_t) &PORTG
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
    (uint16_t) &PINA,
    (uint16_t) &PINB,
    (uint16_t) &PINC,
    (uint16_t) &PIND,
    (uint16_t) &PINE,
    (uint16_t) &PINF,
    (uint16_t) &PING
};

// Why did lincomatic choose the order that he did? Is there no real meaning to it?
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PA, /* PORT A D0*/
    PA,
    PA,
    PA,
    PA,
    PA,
    PA,
    PA,
    PB, /* PORT B D8*/
    PB,
    PB,
    PB,
    PB,
    PB,
    PB,
    PB, 
    PC, /* PORT C D16*/
    PC,
    PC,
    PC,
    PC,
    PC,
    PC,
    PC,
    PD, /* PORT D D24*/
    PD,
    PD,
    PD,
    PD,
    PD,
    PD,
    PD,
    PE, /* PORT E D32*/
    PE,
    PE,
    PE,
    PE,
    PE,
    PE,
    PE,
    PG, /* PORT G D40*/
    PG,
    PG,
    PG,
    PG,
    PF, /* PORT F D45*/
    PF,
    PF,
    PF,
    PF,
    PF,
    PF,
    PF
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
    _BV(0), /* 0 PORT A */
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
    _BV(5),
    _BV(6),
    _BV(7), 
    _BV(0), /* 8 PORT B */
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
    _BV(5),
    _BV(6),
    _BV(7), 
    _BV(0), /* 16 PORT C */
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
    _BV(5),
    _BV(6),
    _BV(7), 
    _BV(0), /* 24 PORT D */
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
    _BV(5),
    _BV(6),
    _BV(7), 
    _BV(0), /* 32 PORT E */
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
    _BV(5),
    _BV(6),
    _BV(7), 
    _BV(0), /* 40 PORT G */
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
    _BV(0), /* 45 PORT F  ANALOG*/
    _BV(1),
    _BV(2),
    _BV(3),
    _BV(4),
    _BV(5),
    _BV(6),
    _BV(7)
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
    NOT_ON_TIMER, /* 0 PORT A */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER, /* 8 PORT B */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    TIMER2A, // 12
    TIMER1A, // 13
    TIMER1B, // 14
    TIMER1C, // 15
    NOT_ON_TIMER, /* 16 PORT C */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER, /* 24 PORT D */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER, /* 32 PORT E */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    TIMER3A, // 35
    TIMER3B, // 36
    TIMER3C, // 37
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER, /* PORT G */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER, /* PORT F */
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER,
    NOT_ON_TIMER
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif

/*
 * http://www.avrfreaks.net/forum/setting-jtd-bit-mcucr
 * include in library
 */
#define jtd_set(x) \
{ \
        __asm__ __volatile__ ( \
                "in __tmp_reg__,__SREG__" "\n\t" \
                "cli" "\n\t" \
                "out %1, %0" "\n\t" \
                "out __SREG__, __tmp_reg__" "\n\t"\
                "out %1, %0" "\n\t" \
                : /* no outputs */ \
                : "r" ((uint8_t)(x ? (1<<JTD) : 0)), \
                  "M" (_SFR_IO_ADDR(MCUCR)) \
                : "r0"); \
}

// RS485 control bits
#define RS485_RE 22
#define RS485_SHDN 23

