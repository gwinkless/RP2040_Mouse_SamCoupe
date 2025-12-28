/************************************************************************
  main.c

  Main functions
    RP2040 - USB to Sam Coupe mouse converter
    Copyright (C) 2025 Geoff Winkless
    Copyright (C) 2023 Darren Jones
    Copyright (C) 2017-2020 Simon Inns

  This file is part of RP2040 Mouse based on the original SmallyMouse from Simon Inns.

    RP2040 Mouse is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Email: nz.darren.jones@gmail.com

************************************************************************/
#include <stdlib.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "pico/binary_info.h"

// Configure RP2040 for slower flash
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1

#define VERSION "1.0"

//#include "pio_usb.h"
#include "bsp/rp2040/board.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "main.h"

#include "hardware/uart.h"
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
/*#define UART_TX_PIN 12
#define UART_RX_PIN 13 */


volatile int16_t samYDelta = 0;
volatile int16_t samXDelta = 0;
volatile uint8_t samButts = 0xf; // we start off with no buttons pressed (they're active-low)
volatile bool mouseLive = false;

volatile uint8_t mousedev_addr;
volatile uint8_t mouseinstance;
volatile bool justmounted = false; // usb callback sets justmounted and mouseinstance/dev_addr

auto_init_mutex(samDeltaMutex);
#ifdef DEBUG
#define DEBUG_PRINT(x) printf x
#define CFG_TUSB_DEBUG 3
#else
#define DEBUG_PRINT(x) \
  do                   \
  {                    \
  } while (0)
#define CFG_TUSB_DEBUG 0
#endif

// Cookie's BoaI article says that the original hardware resets after about 30uS
// we'll be kind and make it 40uS
#define SamMouseTimeout_us 40
void 
__attribute__((noinline, long_call, section(".time_critical"))) 
SamRDMTightLoop () {
  static uint32_t LastRDMSelTimeout = 0;
	static int rdmstate = 0;
  static bool ledstate=0;
	static int copyXDelta, copyYDelta;
  static unsigned char copyButtState;
  int nextpins = 0;
  while (1) {

// when RDMSEL is first pulled low, we copy the mouse state to our copyXXX statics, then send 1111 to the mouse port
// (actually we don't send anything - we let all pins float)
// (the keyboard will override that if any of the cursors or ctrl are held down, but we don't need to worry about that)
// for subsequent reads before the timeout expires we send f, buttons, ydelta>>8, (ydelta>>4) & f, ydelta&f, xdelta>>8, (xdelta>>8)&f, xdelta&f, f
// if there's longer than 30uS between two reads then we start again

// it would actually makes more sense to output the next data values to the pins once RDMSel goes inactive, so that
// once it goes active again the data is already waiting. that way we don't need to worry about response times


    while (!gpio_get(RDMSEL_PIN)) { // our pin is inverted, so we're testing for high
      uint32_t newtm;
// if we timeout in a non-zero state while waiting for active RDMSEL, we reset the state to 0 and clear the GPIOs
// so that when we do go active the values will already be correct
      if (rdmstate
        && ((newtm = time_us_32()) > LastRDMSelTimeout)
        && ((LastRDMSelTimeout > SamMouseTimeout_us) || (newtm < (3 * SamMouseTimeout_us)))
      ) {
        if (rdmstate > 1) gpio_put(STATUS_PIN, ledstate ^= 1); // flip the LED every time we timeout, unless we only read a single value
        rdmstate = 0;
        gpio_put_masked(SamMousePinsMask, 0);
      }
    }
//    gpio_put(STATUS_PIN, ledstate^=1);
    // reset the inter-request timeout
    LastRDMSelTimeout = time_us_32() + SamMouseTimeout_us;
    
    switch (rdmstate) {
      default: // shouldn't need this (rdmstate will only ever be 0-8), but just in case...
        rdmstate = 0;
        // falls through
      case 0:
        mutex_enter_blocking(&samDeltaMutex);
        // we add the delta values to our copy because if the last read didn't complete we'll want to remember it
        copyYDelta -= samYDelta; // we negate the Y delta because Sam thinks up is positive, while USB thinks up is negative
        samYDelta = 0;
        copyXDelta += samXDelta;
        samXDelta = 0;
        mutex_exit(&samDeltaMutex);
        if (copyXDelta > 0x7ff) {
          copyXDelta = 0x7ff;
        } else if (copyXDelta < -0x7ff) {
          copyXDelta = -0x7ff;
        }
        if (copyYDelta > 0x7ff) {
          copyYDelta = 0x7ff;
        } else if (copyYDelta < -0x7ff) {
          copyYDelta = -0x7ff;
        }

        copyButtState = samButts;

// nextpins is the value we set the pins to in the _next_ state: we actually
// set them as soon as this active-state ends, because the NAND gates will
// block off the values until RDMSEL goes active again. This way we get to be
// instantly ready, while running the rp2040 at a lower speed (and thus
// saving power)

        nextpins = 0xf;
        break;
      case 1:
        nextpins = copyButtState;
        break;
      case 2:
        nextpins = (copyYDelta >> 8);
        break;
      case 3:
        nextpins = (copyYDelta >> 4);
        break;
      case 4:
        nextpins = (copyYDelta);
        break;
      case 5:
        nextpins = (copyXDelta >> 8);
        break;
      case 6:
        nextpins = (copyXDelta >> 4);
        break;
      case 7:
        nextpins = (copyXDelta);
        break;
      case 8:
        nextpins = 0xf;
        copyYDelta = 0;
        copyXDelta = 0;
        break;
    }
    rdmstate = (rdmstate + 1) % 9;
    if (!mouseLive) nextpins = 0xf; // if the mouse isn't plugged in, then we just keep everything high
    while (gpio_get(RDMSEL_PIN));
    // rdmsel has gone inactive again, so we move to the next state and output the values for that state
  // these values below are all inverted (false:true, not true:false) because we're using open-collector
  // NAND gates: RDMSEL (active high) NAND 1 becomes 0
    gpio_put(SamMouseBit0_PIN, (nextpins & 1) ? false : true);
    gpio_put(SamMouseBit1_PIN, (nextpins & 2) ? false : true);
    gpio_put(SamMouseBit2_PIN, (nextpins & 4) ? false : true);
    gpio_put(SamMouseBit3_PIN, (nextpins & 8) ? false : true);
  }
}

static void blink_status(uint8_t count)
{
  uint8_t i = 0;
  gpio_put(STATUS_PIN, 0);

  while (i < count)
  {
    sleep_ms(200);
    gpio_put(STATUS_PIN, 1);
    sleep_ms(200);
    gpio_put(STATUS_PIN, 0);
    i++;
  }
}


void core1_main()
{
  sleep_ms(10);
  board_init();

  tuh_init(0); // 1 for pio-usb or max3421, 0 for main pico hub

  while (true)
  {
    tuh_task(); // tinyusb host task
    if (justmounted) {
// apparently (according to some tinyusb forum post) we shouldn't be setting these values from the callback, because
// the interface is still enumerating at the point of the callback. So we set justmounted in the callback and do this here instead.
  // Interface protocol (hid_interface_protocol_enum_t)
      uint8_t const itf_protocol = tuh_hid_interface_protocol(mousedev_addr, mouseinstance);
      justmounted = false;
      DEBUG_PRINT(("Protocol: %d\r\n", itf_protocol));

      // Receive report from boot mouse only
      // tuh_hid_report_received_cb() will be invoked when report is available
      if (itf_protocol == HID_ITF_PROTOCOL_MOUSE)
      {
        // Set protocol to full report mode for mouse wheel support
        tuh_hid_set_protocol(mousedev_addr, mouseinstance, HID_PROTOCOL_REPORT);
        if (tuh_hid_receive_report(mousedev_addr, mouseinstance))
        {
          blink_status(3);
        }
        gpio_put(STATUS_PIN, 1); // Turn status LED on
        mouseLive = true;
      } else {
        blink_status(10+itf_protocol);
      }
    }
  }
}

int 
__attribute__((noinline, long_call, section(".time_critical"))) 
main()
{
// i tried setting the clock speed lower (eg 96MHz) but it messed up everything
// If we changed from polling to spot RDMSEL going high to using an interrupt
// (with a timer interrupt to reset the rdmstate value after 30uS) we could probably
// try lowering this clock rate to save power, but for now this is fine.
  set_sys_clock_khz(125000, true);
  stdio_init_all();

// Setup Debug to UART
#ifdef DEBUG
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_init(UART_ID, 2400);
  int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
#endif
  DEBUG_PRINT(("\033[2J"));
  DEBUG_PRINT(("****************************************************\r\n"));
  DEBUG_PRINT(("*         RP2040 USB To Sam Coupé adaptor          *\r\n"));
  DEBUG_PRINT(("*         Copyright 2022 Darren Jones              *\r\n"));
  DEBUG_PRINT(("*         (nz.darren.jones@gmail.com)              *\r\n"));
  DEBUG_PRINT(("*         Copyright 2025 Geoff Winkless            *\r\n"));
  DEBUG_PRINT(("*         (sam@ukku.uk)                            *\r\n"));
  DEBUG_PRINT(("*         Version: %s                             *\r\n", VERSION));
  DEBUG_PRINT(("*         Build Date: %s %s         *\r\n", __DATE__, __TIME__));
  DEBUG_PRINT(("****************************************************\r\n"));
  DEBUG_PRINT(("\r\n"));
  DEBUG_PRINT(("RP2040 USB To Sam Booting.....\r\n"));

  // all USB task run in core1
  multicore_reset_core1();
  DEBUG_PRINT(("Core1 Reset\r\n"));

  multicore_launch_core1(core1_main);
  DEBUG_PRINT(("Core1 Launched\r\n"));
  // Initialise the RP2040 hardware
  initialiseHardware();
  DEBUG_PRINT(("Hardware Initalized\r\n"));

  // Blink Status LED and wait for everything to settle
  blink_status(10);
  // can't believe this (5 seconds) needs to be so long. Also, we don't really care if we run before everything's ready
  // now we (core0) go and sit in a tight loop waiting for RDMSel to change
  SamRDMTightLoop();
}

void initialiseHardware(void)
{
  // Document pins for picotool
  bi_decl(bi_1pin_with_name(SamMouseBit0_PIN, "Sam Mouse Bit0 Output"));
  bi_decl(bi_1pin_with_name(SamMouseBit1_PIN, "Sam Mouse Bit1 Output"));
  bi_decl(bi_1pin_with_name(SamMouseBit2_PIN, "Sam Mouse Bit2 Output"));
  bi_decl(bi_1pin_with_name(SamMouseBit3_PIN, "Sam Mouse Bit3 Output"));
  bi_decl(bi_1pin_with_name(SamMouseBit4_PIN, "Sam Mouse Bit4 Output"));
  bi_decl(bi_1pin_with_name(RDMSEL_PIN, "Sam Mouse RDMSEL input"));
  bi_decl(bi_1pin_with_name(UART_RX_PIN, "UART RX"));
  bi_decl(bi_1pin_with_name(UART_TX_PIN, "UART TX"));
  bi_decl(bi_1pin_with_name(STATUS_PIN, "Status LED"));

  // Initalize the pins
  gpio_init(SamMouseBit0_PIN);
  gpio_init(SamMouseBit1_PIN);
  gpio_init(SamMouseBit2_PIN);
  gpio_init(SamMouseBit3_PIN);
  gpio_init(SamMouseBit4_PIN);
  gpio_init(RDMSEL_PIN);
  gpio_init(STATUS_PIN);
  DEBUG_PRINT(("Pins initalised\r\n"));

  // Set pin directions
// these 4 start out as INputs because we need them to float when they're non-zero. 
// We set the output value to 0, and use set_dir_masked to set any 0 pins to output
  gpio_set_dir_masked(SamMousePinsMask | (1<<RDMSEL_PIN) | (1<<STATUS_PIN), SamMousePinsMask | (1<<STATUS_PIN)); // mouse and status pins are outbound, RDMSEL is inbound
  gpio_pull_up(RDMSEL_PIN); // the internal pullup is about 50-80kOhm, which won't be anywhere near enough to not require our 300ohm pullup resistor, but let's not fight it at least

  DEBUG_PRINT(("Pin directions set\r\n"));
  gpio_put_masked(SamMousePinsMask, 0);
  gpio_put(STATUS_PIN, 0);

}
//--------------------------------------------------------------------+
// Host HID
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len)
{
  (void)desc_report;
  (void)desc_len;
  DEBUG_PRINT(("USB Device Attached\r\n"));
  mousedev_addr = dev_addr;
  mouseinstance = instance;
  justmounted = true;
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  (void)dev_addr;
  (void)instance;
  DEBUG_PRINT(("USB Device Removed\r\n"));
  mouseLive = false;
  gpio_put(STATUS_PIN, 0); // Turn status LED off
  // don't bother changing the interrupt status - we'll just report no movement
}

static void processMouse(uint8_t dev_addr, hid_mouse_report_t const *report)
{
  int16_t tmpsamXDelta, tmpsamYDelta;
  // Blink status LED
  // gpio_put(STATUS_PIN, 0);
  (void)dev_addr;
  // sam driver doesn't have any concept of scroll wheel. we could add it, but software support isn't there
/*  // Handle scroll wheel
  if (report->wheel)
  {
    gpio_init(MB_PIN);
    gpio_set_dir(MB_PIN, GPIO_OUT);
    gpio_put(MB_PIN, 0);
    processMouseMovement(report->wheel, MOUSEY);
    sleep_ms(100);
    DEBUG_PRINT(("Wheel movement %d\r\n", report->wheel));
  }
  else
  {
    gpio_deinit(MB_PIN);
  }
*/
  // Handle mouse buttons
  samButts = ((report->buttons & 1) | ((report->buttons << 1) & 4) | ((report->buttons >> 1) & 2)) ^ 0xf;
// usb mouse buttons are active-high, we want active-low. 
// We swap bits 1 and 2 because USB is left-right-centre, sam is left-centre-right

  // Handle mouse movement
  mutex_enter_blocking(&samDeltaMutex);
  tmpsamXDelta = samXDelta + ((report->x + 1) >> 2);
  tmpsamYDelta = samYDelta + ((report->y + 1) >> 2);
  // we only report back 12-bit values, so restrict the allowable range
  samXDelta = (tmpsamXDelta > 0x7ff) ? 0x7ff : ((tmpsamXDelta < -0x7ff) ? -0x7ff : (int16_t)tmpsamXDelta);
  samYDelta = (tmpsamYDelta > 0x7ff) ? 0x7ff : ((tmpsamYDelta < -0x7ff) ? -0x7ff : (int16_t)tmpsamYDelta);
  mutex_exit(&samDeltaMutex);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len)
{
  (void)len;
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  switch (itf_protocol)
  {
  case HID_ITF_PROTOCOL_MOUSE:
    processMouse(dev_addr, (hid_mouse_report_t const *)report);
    break;

  default:
    break;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance))
  {
    return;
  }
}
