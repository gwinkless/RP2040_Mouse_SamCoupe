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
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"

// don't need to configure RP2040 for slower flash, since our flash is plenty fast enough, thanks
//#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
//#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1

#define VERSION "1.0"

// can't imagine us ever using smaller flash than 1MB. The smaller flash
// chips are actually more expensive than the GD25Q80EEIGR we currently use
#define FLASH_SETTINGS_ADDR (1024*1024 - 4096)
#define XIP_SETTINGS_ADDR (XIP_BASE + FLASH_SETTINGS_ADDR)

#include "main.h"
#include "bsp/board.h"
#include "tusb.h"

#include "hardware/uart.h"
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#ifdef DEBUG
#define UART_TX_PIN 15
#define UART_RX_PIN 16 
#endif

volatile int16_t samYDelta = 0;
volatile int16_t samXDelta = 0;
volatile int16_t samWheelDelta = 0;
volatile uint8_t samButts = 0xf; // we start off with no buttons pressed (they're active-low)
volatile bool mouseLive = false;

volatile uint8_t mouseinstance;
volatile bool justmounted = false; // usb callback sets justmounted and mouseinstance/dev_addr
static uint32_t crc32(const unsigned char *s, size_t length){
  uint32_t crc = 0xffffffff;
  size_t i;
  for (i = 0; i < length ; i++ ) {
    uint8_t byte = s[i];
    crc = crc ^ byte;
    for (uint8_t j = 8; j > 0; --j) {
      crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
    }
    i++;
  }
  return crc ^ 0xffffffff;
}


CFG_TUH_MEM_SECTION struct stDesc {
  TUH_EPBUF_TYPE_DEF(tusb_desc_device_t, device);
  TUH_EPBUF_DEF(serial, 64*sizeof(uint16_t));
  TUH_EPBUF_DEF(buf, 128*sizeof(uint16_t));
} desc;

#define FLASH_CONFIG_SIGNATURE 0x5a3c009e
// just in case we ever change the config format, store a version id with the config
#define FLASH_CONFIG_VERSION 1
struct mconf {
  int scale;
  uint32_t flags;
};
// do we want the status light on (1) or off (0) when this device is seen?
#define MCONF_FLAGS_STATUSLIGHT 1
// do we want to do our acceleration-lite mechanism?
#define MCONF_FLAGS_ACCEL 2
struct onedev {
  struct mconf conf;
  uint32_t serialcrc;
  uint16_t idVendor;
  uint16_t idProduct;
};

struct onedev *mydev;
// assuming struct mconf expands to 8 bytes, onedev is 16 bytes. 
// 255 of them takes 4080 bytes of our 4096 page
#define MaxConfSettings 255
struct settingsStore {
  unsigned int signature;
  unsigned int count;
  unsigned int version;
  struct onedev devconfs[MaxConfSettings];
} mouseConfigs = {.count = 0,.signature = FLASH_CONFIG_SIGNATURE, .version = FLASH_CONFIG_VERSION};


struct onefield {
  int instance;
  int report_id;
  int bit_offset;
  int bit_size;
};
struct {
    struct onefield x;
    struct onefield y;
    struct onefield buttons;
    struct onefield wheel;
    uint8_t dev_addr;
} matchedfields = {0};
void mouseConfigSaveToFlash () {
  // we're running from RAM with pico_set_binary_type(rp2040_mouse copy_to_ram)
  // in CMakeLists.txt, so we don't need to use flash_safe_execute()
  flash_range_erase(FLASH_SETTINGS_ADDR, 4096);
  flash_range_program(FLASH_SETTINGS_ADDR, (uint8_t *)&mouseConfigs, 4096);
//  if (flash_safe_execute(call_flash_range_erase, (uintptr_t []) { FLASH_SETTINGS_ADDR, 4096}, 5000) == PICO_OK) {
//    flash_safe_execute(call_flash_range_program, (uintptr_t []) { FLASH_SETTINGS_ADDR, (uintptr_t)&mouseConfigs, 4096}, 5000);
//  }  
}
// load the configs from flash into our memory block, if there is one
void mouseConfigsReadFromFlash () {
  if (((struct settingsStore *)XIP_SETTINGS_ADDR)->signature == FLASH_CONFIG_SIGNATURE) {
    memcpy(&mouseConfigs, (uint8_t *)XIP_SETTINGS_ADDR, sizeof(struct settingsStore));
  }
}
void setDefaultConfig (struct onedev * dev) {
// for now we just reset to these defaults, but at some point we might decide to change the scale
// based on some precached defaults for certain devices (using dev->idProduct and dev->idVendor)
  dev->conf.scale = 2;
  dev->conf.flags = MCONF_FLAGS_STATUSLIGHT | MCONF_FLAGS_ACCEL;
}
// search through the existing memory block and see if we already have this device
struct onedev * mouseConfigFind (struct stDesc *desc) {
// we only have 255 total (because we restrict ourselves to a single 4096 byte page)
// and this doesn't need to be fast. So we won't bother sorting the values
// or bsearch()ing them, just loop around them all, it'll take less than a millisecond
  unsigned int i;
  unsigned int serialcrc;
  serialcrc = crc32(desc->serial, sizeof(desc->serial));
  struct mconf *mc = NULL, tempmc = {0};
  for (i=0; i < mouseConfigs.count; i++) {
    if (mouseConfigs.devconfs[i].idVendor == desc->device.idVendor && mouseConfigs.devconfs[i].idProduct == desc->device.idProduct) {
// ideally we find one where the vendor, product and serial all match; failing that,
// use the most recent matching vendor/product pair. That way if we do actually have a 
// device with the same vid/pid but with a different serial we can still default to
// something that probably makes sense but also allow us to save different settings if
// we want two of the same device with different serials to have two different configs;
// it also means if we have a badly-behaved device that changes its serial each time
// we always get the most recent saved config for it
      if (mouseConfigs.devconfs[i].serialcrc == serialcrc) {
        return &mouseConfigs.devconfs[i];
      }
      mc = &mouseConfigs.devconfs[i].conf;
    }
  }
  // we might be about to move (or worse, overwrite) our best-match config,
  // so we take a copy of it first
  if (mc) tempmc = *mc;
  // if we got here, we didn't find an exact match, so create a new conf
  // at the end with the current vendor/product/serial triplet
  if (mouseConfigs.count == MaxConfSettings) {
    // yikes! too many configs for our 4k page.
    // we'll delete the first from the list.
    // that could mean we end up deleting the settings for the device you use
    // most often (if it was the first you ever plugged in);
    // we could reorder the list every time a different device is plugged in,
    // to ensure that the ones at the start are the most-used
    // but that would mean a _lot_ of writes, and the potential risk of deleting
    // a used config when someone stores 256 device configs is much lower than ending
    // up with broken flash just because you kept swapping the same two devices
    // remember at this point we haven't deleted the config on flash, it's
    // only when someone changes the config for the device that it gets written
    memmove(mouseConfigs.devconfs, &mouseConfigs.devconfs[1], sizeof(mouseConfigs.devconfs[0])*(MaxConfSettings - 1));
    mouseConfigs.count--;
  }
  if (mc) {
    memcpy(&mouseConfigs.devconfs[mouseConfigs.count].conf, &tempmc, sizeof(tempmc));
  } else {
    setDefaultConfig(&mouseConfigs.devconfs[mouseConfigs.count]);
  }
  mouseConfigs.devconfs[mouseConfigs.count].idVendor = desc->device.idVendor;
  mouseConfigs.devconfs[mouseConfigs.count].idProduct = desc->device.idProduct;
  mouseConfigs.devconfs[mouseConfigs.count].serialcrc = serialcrc;
  return &mouseConfigs.devconfs[mouseConfigs.count++];
}
static inline int
__attribute__((section(".time_critical"))) 
scaledown(int raw) {
    int scale = mydev->conf.scale;

    if (scale > 0) {
        // divide by 2^scale, but preserve tiny movements
        int bias = (1 << scale) - 1;
        raw = (raw + (raw >= 0 ? bias : -bias)) >> scale;
    } else if (scale < 0) {
        // multiply by 2^(-scale)
        raw = raw << (-scale);
    }

    if (mydev->conf.flags & MCONF_FLAGS_ACCEL) {
        int mag = raw >= 0 ? raw : -raw;

        if (mag > 24) {
            raw <<= 3;   // ×8
        } else if (mag > 16) {
            raw <<= 2;   // ×4
        } else if (mag > 8) {
            raw <<= 1;   // ×2
        }
    }
    return raw;
}

auto_init_mutex(samDeltaMutex);
#ifdef DEBUG
#define DEBUG_PRINT(x) printf x
#define CFG_TUSB_DEBUG 3
#else
#define DEBUG_PRINT(x) \
  do                   \
  {                    \
  } while (0)
#ifdef CFG_TUSB_DEBUG
#undef CFG_TUSB_DEBUG
#endif  
#define CFG_TUSB_DEBUG 0
#endif
unsigned long
getJSPins () {
  unsigned long nextpins = gpio_get_all();
  nextpins = 

#if (((JoystickPinsMask >> JoystickFire_PIN) == 31) && (JoystickFirePIN < JoystickUp_PIN))
// we're not on the broken prototype where the joystick pins aren't in the same order as the output pins
            (nextpins & JoystickPinsMask) >> JoystickFire_PIN;
#else
              (((nextpins >> JoystickFire_PIN)  & 1) << 0) |
              (((nextpins >> JoystickUp_PIN)    & 1) << 1) |
              (((nextpins >> JoystickDown_PIN)  & 1) << 2) |
              (((nextpins >> JoystickLeft_PIN)  & 1) << 3) |
              (((nextpins >> JoystickRight_PIN) & 1) << 4);
#endif
  return nextpins;
}

// Cookie's BoaI article says that the original hardware resets after about 30uS
// we'll be kind and make it 40uS
#define SamMouseTimeout_us 40
void 
__attribute__((noinline, long_call, section(".time_critical"))) 
SamRDMTightLoop () {
  static uint32_t LastRDMSelTimeout = 0;
	static int rdmstate = 0;
	static int copyXDelta, copyYDelta, scaledXDelta, scaledYDelta;
  static unsigned char copyButtState;
  static int copyWheel;
  unsigned char jspins;
  int nextpins = 0;
  while (1) {

// when RDMSEL is first pulled low, we copy the mouse state to our copyXXX statics, then send 1111 to the mouse port
// (actually we don't send anything - we let all pins float)
// (the keyboard will override that if any of the cursors or ctrl are held down, but we don't need to worry about that)
// for subsequent reads before the timeout expires we send f, buttons, ydelta>>8, (ydelta>>4) & f, ydelta&f, xdelta>>8, (xdelta>>8)&f, xdelta&f, f
// if there's longer than 30uS between two reads then we start again

// it actually makes more sense to output the next data values to the pins once RDMSel goes inactive, so that
// once it goes active again the data is already waiting; that way we don't need to worry about response times
// so we do that now - each state we just set "nextpins" and then wait for RDMSEL to go low and
// write the next value immediately afterwards


    while (!gpio_get(RDMSEL_PIN)) { // our pin is inverted, so we're testing for high
      uint32_t newtm;
// if we timeout in a non-zero state while waiting for active RDMSEL, we reset the state to 0 and set the GPIOs
// to the JS pins so that when we do go active the values will already be correct
      if (rdmstate
        && ((newtm = time_us_32()) > LastRDMSelTimeout)
        && ((LastRDMSelTimeout > SamMouseTimeout_us) || (newtm < (3 * SamMouseTimeout_us)))
      ) {
        //if (rdmstate > 1) gpio_put(STATUS_PIN, ledstate ^= 1); // for debugging flip the LED every time we timeout, unless we only read a single value
        rdmstate = 0;
        nextpins = getJSPins();
        gpio_put_masked(SamMousePinsMask, 
#if ((SamMousePinsMask >> SamMouseBit0_PIN) == 31)
// we're not on the broken prototype where the pins aren't contiguous
          ((nextpins&31)<<SamMouseBit0_PIN)
#else
          ((nextpins&1)<<SamMouseBit0_PIN)
        | ((nextpins&2)<<(SamMouseBit1_PIN-1))
        | ((nextpins&4)<<(SamMouseBit2_PIN-2))
        | ((nextpins&8)<<(SamMouseBit3_PIN-3))
        | ((nextpins&16)<<(SamMouseBit4_PIN-4))
#endif
        );
      }
    }
    // reset the inter-request timeout
    LastRDMSelTimeout = time_us_32() + SamMouseTimeout_us;
    
    if (mouseLive) {
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
          copyWheel += samWheelDelta;
          samWheelDelta = 0;
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
          if (copyWheel > 63) {
            copyWheel = 63;
          } else if (copyWheel < -64) {
            copyWheel = -64;
          }
          copyButtState = samButts;
          jspins = getJSPins();
          if (((copyButtState & 7) == 7) && (copyYDelta == 0) && (copyXDelta == 0) && (copyWheel == 0) && (jspins != 0x1f)) {
            // if the mouse isn't being used but the joystick is, use the joystick for every read. That way code that
            // reads the same cursors port multiple times in very quick succession (hello Howard!) won't break,
            // as long as you don't wiggle the mouse
            nextpins = jspins;
            rdmstate = 8;
          } else {
  // nextpins is the value we set the pins to in the _next_ state: we actually
  // set them as soon as this active-state ends, because the NAND gates will
  // block off the values until RDMSEL goes active again. This way we get to be
  // instantly ready, while running the rp2040 at a lower speed (and thus
  // saving power)
            nextpins = 0x1f;
          }

          break;
        case 1:
          nextpins = ((copyWheel&64)>>2) | copyButtState;
          break;
        case 2:
          scaledYDelta = scaledown(copyYDelta); 
          nextpins = ((copyWheel&32)>>1) | ((scaledYDelta >> 8) & 0xf);
          break;
        case 3:
          nextpins = ((copyWheel&16)) | ((scaledYDelta >> 4) & 0xf);
          break;
        case 4:
          nextpins = ((copyWheel&8)<<1) | ((scaledYDelta) & 0xf);
          break;
        case 5:
          scaledXDelta = scaledown(copyXDelta);
          nextpins = ((copyWheel&4)<<2) | ((scaledXDelta >> 8) & 0xf);
          break;
        case 6:
          nextpins = ((copyWheel&2)<<3) | ((scaledXDelta >> 4) & 0xf);
          break;
        case 7:
          nextpins = ((copyWheel&1)<<4) | ((scaledXDelta) & 0xf);
          copyYDelta = 0;
          copyXDelta = 0;
          copyWheel = 0;
          break;
        case 8:
          nextpins = getJSPins();
          break;
      }
      rdmstate = (rdmstate + 1) % 9;
    } else {
// if the mouse isn't plugged in, then use the joystick values
      nextpins = getJSPins();
    }
    while (gpio_get(RDMSEL_PIN));
    // rdmsel has gone inactive again, so we move to the next state and output the values for that state
    gpio_put_masked(SamMousePinsMask, 
#if ((SamMousePinsMask >> SamMouseBit0_PIN) == 31)
// we're not on the broken prototype where the pins aren't contiguous
          ((nextpins&31)<<SamMouseBit0_PIN)
#else
          ((nextpins&1)<<SamMouseBit0_PIN)
        | ((nextpins&2)<<(SamMouseBit1_PIN-1))
        | ((nextpins&4)<<(SamMouseBit2_PIN-2))
        | ((nextpins&8)<<(SamMouseBit3_PIN-3))
        | ((nextpins&16)<<(SamMouseBit4_PIN-4))
#endif
      );
  }
}

static void blink_status(uint8_t count, uint8_t delayms)
{
  uint8_t i = 0;
  gpio_put(STATUS_PIN, 0);

  while (i < count)
  {
    sleep_ms(delayms);
    gpio_put(STATUS_PIN, 1);
    sleep_ms(delayms);
    gpio_put(STATUS_PIN, 0);
    i++;
  }
}


void core1_main()
{
  uint32_t configTimeStart = 0;
  uint8_t lastClick = 0;
  int configState = 0;
  struct mconf old_config;
  sleep_ms(10);
  board_init();
  const tusb_rhport_init_t rh_init = {
      .role = TUSB_ROLE_HOST,
      .speed = TUSB_SPEED_FULL,
  };
  tusb_init(0, &rh_init); // rhport 0 is main rp2040 hub
 // even though we've set PICO_DEFAULT_LED_PIN to our STATUS_PIN in main.h, the tinyusb library is prebuilt with it set to 25
 // So we force that pin as input here. Messy, but works.
 #if (STATUS_PIN != 25)
  gpio_set_dir(25, false);
  gpio_pull_up(25);
#endif
// I have run successfully at 100MHz, but probably better to give ourselves some
// additional leeway for timing
  set_sys_clock_khz(133000, true); 
  
  mouseConfigsReadFromFlash();  
  while (true) {
    tuh_task(); // tinyusb host task
// if the device callbacks have found a mouse, they will set matchedfields.dev_addr    
    if (matchedfields.dev_addr) {
      uint8_t buf[64];
      bool gotSerial = false;
      tuh_descriptor_get_string_sync(matchedfields.dev_addr, 0, 0, buf, sizeof(buf));
      uint16_t langid = ((uint16_t*)buf)[1];
  // Get Device Descriptor
      if (tuh_descriptor_get_device_sync(matchedfields.dev_addr, &desc.device, 18) == XFER_RESULT_SUCCESS) {
        if (desc.device.iSerialNumber != 0) {
          if (tuh_descriptor_get_serial_string_sync(matchedfields.dev_addr, langid, desc.serial, sizeof(desc.serial))) {
            gotSerial = true;
          }
        }
      }
      if (!gotSerial) memset(desc.serial, '\0', sizeof(desc.serial));
    // read stored device config, if there is one
      mydev = mouseConfigFind(&desc);
      gpio_put(STATUS_PIN, mydev->conf.flags & MCONF_FLAGS_STATUSLIGHT); // set the light based on the config value      
      matchedfields.dev_addr = 0;
    }

// check if we need to do our mouse config    
    switch (configState) {
      case 0: // not in config. check if 3-buttons are pressed
        if ((samButts & 7) == 0) { // buttons are active-low
          configTimeStart = time_us_32();
          configState = 1; 
        }
        break;
      case 1: // see if 3-buttons are held for 3sec
        if ((samButts & 7) == 0) {
          if (time_us_32() >= (configTimeStart + 3000000)) {
            configState = 2;
            blink_status(2, 200);
          }
        } else configState = 0; // didn't reach 3 seconds, so back to normal
        break;
      case 2: // wait for the user to let go of the buttons
        if ((samButts & 7) == 7) {
          configState = 3;
          memcpy(&old_config, &mydev->conf, sizeof(old_config));
        }
        break;
      case 3: // the user has let go of the buttons, so start recording changes
        if ((samButts & 7) != 7) { // we have a button-click
          lastClick |= ((samButts & 7) ^ 7); // ^7 because we want active-high
          // we OR them because we don't expect the user to be able to release
          // or click all buttons at the same time; this could end up with weird
          // results if you hold L, then click R and then M separately, but I can
          // live with that, I think
        } else if (lastClick) { // buttons released
          switch (lastClick) { // which buttons were set?
            case 1:
              // increase scaling
              mydev->conf.scale++;
              blink_status(1, 50);
              break;
            case 2:
              // decrease scaling
              mydev->conf.scale--;
              blink_status(2, 50);
              break;
            case 3:
            // buttons 1 and 2 mean change status light
              blink_status(3, 50);
              mydev->conf.flags ^= MCONF_FLAGS_STATUSLIGHT;
              break;
            case 5:
            // buttons 1 and 3 mean change acceleration
              blink_status(5, 50);
              mydev->conf.flags ^= MCONF_FLAGS_ACCEL;
              break;
            case 6:
            // buttons 2 and 3 mean reset to default
              setDefaultConfig(mydev);
              break;
            case 7:
              // all three buttons: save config and exit config mode
              configState = 0;
              if (memcmp(&old_config, &mydev->conf, sizeof (old_config))) mouseConfigSaveToFlash();
              blink_status(7, 50);
              gpio_put(STATUS_PIN, mydev->conf.flags & MCONF_FLAGS_STATUSLIGHT); // set the light based on the config value
              break;
          }
          lastClick = 0;
        }
        break;
    }
  }
}

int 
__attribute__((noinline, long_call, section(".time_critical"))) 
main()
{
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
  DEBUG_PRINT(("*         Copyright 2026 Geoff Winkless            *\r\n"));
  DEBUG_PRINT(("*         (sam@ukku.uk)                            *\r\n"));
  DEBUG_PRINT(("*         Version: %s                             *\r\n", VERSION));
  DEBUG_PRINT(("*         Build Date: %s %s         *\r\n", __DATE__, __TIME__));
  DEBUG_PRINT(("****************************************************\r\n"));
  DEBUG_PRINT(("\r\n"));
  DEBUG_PRINT(("RP2040 USB To Sam Booting.....\r\n"));
#if SAMMOUSE_PICOBOARD
// for now, we're going to run core1_main in core 0, so we can debug the USB code
  core1_main();
#endif
  multicore_reset_core1();
  DEBUG_PRINT(("Core1 Reset\r\n"));

  multicore_launch_core1(core1_main);
  DEBUG_PRINT(("Core1 Launched\r\n"));
  // Initialise the RP2040 hardware
  initialiseHardware();
  DEBUG_PRINT(("Hardware Initalized\r\n"));

  // Blink Status LED and wait for everything to settle
//  blink_status(10, 200);
// can't believe this (5 seconds) needs to be so long.
// Also, we don't really care if we run before everything's ready, so I'm just going to take it out

  // there's a race here where if the mouse is already plugged in (or plugged in during our startup wait) we
// end up overriding the status-1 that the usb handler sets with our blink_status. So we just do it here too.
//  if (mouseLive) gpio_put(STATUS_PIN, 1); 
// now we (core0) go and sit in a tight loop waiting for RDMSel to change
  SamRDMTightLoop();
}

void initialiseHardware(void)
{
  // Document pins for picotool
  bi_decl(bi_1pin_with_name(SamMouseBit0_PIN,  "Sam Mouse Bit0 Output"));
  bi_decl(bi_1pin_with_name(SamMouseBit1_PIN,  "Sam Mouse Bit1 Output"));
  bi_decl(bi_1pin_with_name(SamMouseBit2_PIN,  "Sam Mouse Bit2 Output"));
  bi_decl(bi_1pin_with_name(SamMouseBit3_PIN,  "Sam Mouse Bit3 Output"));
  bi_decl(bi_1pin_with_name(SamMouseBit4_PIN,  "Sam Mouse Bit4 Output"));
  bi_decl(bi_1pin_with_name(JoystickFire_PIN,  "DB9 Joystick Fire Input"));
  bi_decl(bi_1pin_with_name(JoystickUp_PIN,    "DB9 Joystick Up Input"));
  bi_decl(bi_1pin_with_name(JoystickDown_PIN,  "DB9 Joystick Down Input"));
  bi_decl(bi_1pin_with_name(JoystickLeft_PIN,  "DB9 Joystick Left Input"));
  bi_decl(bi_1pin_with_name(JoystickRight_PIN, "DB9 Joystick Right Input"));
  #ifdef DEBUG
  bi_decl(bi_1pin_with_name(UART_RX_PIN,       "UART RX"));
  bi_decl(bi_1pin_with_name(UART_TX_PIN,       "UART TX"));
  #endif
  bi_decl(bi_1pin_with_name(STATUS_PIN,        "Status LED"));

  // Initalize the pins
  gpio_init(SamMouseBit0_PIN);
  gpio_init(SamMouseBit1_PIN);
  gpio_init(SamMouseBit2_PIN);
  gpio_init(SamMouseBit3_PIN);
  gpio_init(SamMouseBit4_PIN);
  gpio_init(RDMSEL_PIN);
  gpio_init(STATUS_PIN);
  gpio_init(JoystickFire_PIN);
  gpio_init(JoystickUp_PIN);
  gpio_init(JoystickDown_PIN);
  gpio_init(JoystickLeft_PIN);
  gpio_init(JoystickRight_PIN);
  DEBUG_PRINT(("Pins initialised\r\n"));

  // Set pin directions
  gpio_set_dir_masked(JoystickPinsMask | SamMousePinsMask | (1<<RDMSEL_PIN) | (1<<STATUS_PIN), SamMousePinsMask | (1<<STATUS_PIN)); // mouse and status pins are outbound, RDMSEL and joystick are inbound
  gpio_pull_up(RDMSEL_PIN); // the internal pullup is about 50-80kOhm, which won't be anywhere near enough to not require our 3kOhm pullup resistor, but let's not fight it at least
  gpio_pull_up(JoystickFire_PIN);     // again, we have external pullups on these
  gpio_pull_up(JoystickUp_PIN);       // Not sure they're required - the prototype
  gpio_pull_up(JoystickDown_PIN);     // board didn't have them - but it's
  gpio_pull_up(JoystickLeft_PIN);     // cleaner to have them
  gpio_pull_up(JoystickRight_PIN);    //
  gpio_set_outover(SamMouseBit0_PIN, GPIO_OVERRIDE_INVERT);
  gpio_set_outover(SamMouseBit1_PIN, GPIO_OVERRIDE_INVERT);
  gpio_set_outover(SamMouseBit2_PIN, GPIO_OVERRIDE_INVERT);
  gpio_set_outover(SamMouseBit3_PIN, GPIO_OVERRIDE_INVERT);
  gpio_set_outover(SamMouseBit4_PIN, GPIO_OVERRIDE_INVERT);

  DEBUG_PRINT(("Pin directions set\r\n"));
  gpio_put_masked(SamMousePinsMask, SamMousePinsMask);
  gpio_put(STATUS_PIN, 0);

}
//--------------------------------------------------------------------+
// Host HID
//--------------------------------------------------------------------+
#define MAX_REPORT  8


void parse_hid_report_descriptor(uint8_t dev_addr, int instance, uint8_t const* desc, uint16_t len)
{
    uint16_t bit_offset = 0;
    uint16_t usage_list[32] = {0};
    uint8_t usage_count = 0;
    uint16_t usage_page = 0;
    uint16_t usage_max = 0;
    uint16_t usage_min = 0;
    uint8_t report_size = 0;
    uint8_t report_count = 0;
    int16_t report_id = -1;
    int first_button_bit_offset = -1;
    for (uint16_t i = 0; i < len; ) {
        uint8_t b = desc[i++];
        uint8_t size = b & 0x03;
        uint8_t type = (b >> 2) & 0x03;
        uint8_t tag  = (b >> 4) & 0x0F;

        uint32_t val = 0;
        for (int s = 0; s < size; s++) {
            val |= desc[i++] << (8 * s);
        }

        switch (type) {
            case 0: // Main
                if (tag == 8) { // INPUT
                    for (int c = 0; c < report_count; c++) {
                        struct onefield *item = NULL;
                        if (usage_page == HID_USAGE_PAGE_DESKTOP) {
                            switch (usage_list[c]) {
                            case HID_USAGE_DESKTOP_X:
                                item = &matchedfields.x;
                                break;
                            case HID_USAGE_DESKTOP_Y:
                                item = &matchedfields.y;
                                break;
                            case HID_USAGE_DESKTOP_WHEEL:
                                item = &matchedfields.wheel;
                                break;
                            }
                        } else if (usage_page == HID_USAGE_PAGE_BUTTON) {
                          if (usage_list[c] == 1) {
                              // First button block
                              matchedfields.buttons.report_id = report_id;
                              matchedfields.buttons.bit_offset = bit_offset + ((report_id == -1) ? 0 : 8);
                              matchedfields.buttons.bit_size = report_count;
                              matchedfields.buttons.instance = instance;
                              matchedfields.dev_addr = dev_addr;
                              first_button_bit_offset = bit_offset;
                          } else if (first_button_bit_offset != -1) {
                              // Additional button block — merge if contiguous
                              uint16_t expected_offset =
                                  first_button_bit_offset +
                                  matchedfields.buttons.bit_size;

                              if (bit_offset == expected_offset) {
                                  matchedfields.buttons.bit_size += report_count;
                              }
                              // else: non‑contiguous button block (rare) — ignore or handle separately
                          }                          
                        }
                        if (item) {
                          item->report_id = report_id;
                          item->bit_offset = bit_offset + ((report_id == -1) ? 0 : 8);
                          item->bit_size = report_size;
                          item->instance = instance;
                        }

                        bit_offset += report_size;
                    }

                }
                usage_count = 0; // we reset local usages after every Main item
                memset(usage_list, '\0', sizeof(usage_list));
                break;
        case 1: // Global
            switch (tag) {
            case 0: usage_page = val; break;
//            case 1: logical_min = (int32_t)val; break;
//            case 2: logical_max = (int32_t)val; break;
            case 7: report_size = val; break;
            case 9: report_count = val; break;
            case 8: report_id = val; break;
            default:
            }
            break;
        case 2: // Local
            switch (tag) {
            case 0: // USAGE
                usage_list[usage_count++] = val;
                break;

            case 1: // USAGE_MIN
                usage_min = val;
                break;

            case 2: // USAGE_MAX
                usage_max = val;
                // Expand range
                if ((usage_page == HID_USAGE_PAGE_BUTTON) && usage_max - usage_min + 1 > 32) {
                    // Just treat as a generic buttons bitfield, don’t expand
                    usage_count = 0;  // or leave list empty
                } else {
                    for (uint16_t u = usage_min; u <= usage_max && usage_count < 32; u++) {
                        usage_list[usage_count++] = u;
                    }
                }                
                break;
            }
            break;
        }
    }
}

void tuh_hid_report_descriptor_cb(uint8_t dev_addr, uint8_t instance,
                                  uint8_t const* desc_report, uint16_t desc_len) {
  parse_hid_report_descriptor(dev_addr, instance, desc_report, desc_len);
  if (tuh_hid_receive_report(dev_addr, instance)) {
//    blink_status(3, 200);
// don't need to do this. It just slows down availability when you plug in the device.
  }
}
// Invoked when device with hid interface is mounted
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;
  DEBUG_PRINT(("USB Device Attached\r\n"));
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  parse_hid_report_descriptor(dev_addr, instance, desc_report, desc_len);
  if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
    // Set protocol to full report mode for mouse wheel support
    tuh_hid_set_protocol(dev_addr, instance, HID_PROTOCOL_REPORT);
    mouseLive = true;
    mouseinstance = instance;
  }
  if (tuh_hid_receive_report(dev_addr, instance)) {
    blink_status(3, 200);
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  (void)dev_addr;
  (void)instance;
  DEBUG_PRINT(("USB Device Removed\r\n"));
  if (instance == mouseinstance) mouseLive = false;
  gpio_put(STATUS_PIN, 0); // Turn status LED off
}

static void processMouse(hid_mouse_report_t const *report)
{
  int16_t tmpsamXDelta, tmpsamYDelta, tmpsamWheelDelta;
  // Blink status LED
  // gpio_put(STATUS_PIN, 0);
  // Handle mouse buttons
  samButts = ((report->buttons & 1) | ((report->buttons << 1) & 4) | ((report->buttons >> 1) & 2)) ^ 0xf;
// usb mouse buttons are active-high, we want active-low. 
// We swap bits 1 and 2 because USB is left-right-centre, sam is left-centre-right

  // Handle mouse movement
  mutex_enter_blocking(&samDeltaMutex);
  tmpsamXDelta = samXDelta + report->x;
  tmpsamYDelta = samYDelta + report->y;
  // we only report back 12-bit values, so restrict the allowable range
  samXDelta = (tmpsamXDelta > 0x7ff) ? 0x7ff : ((tmpsamXDelta < -0x7ff) ? -0x7ff : (int16_t)tmpsamXDelta);
  samYDelta = (tmpsamYDelta > 0x7ff) ? 0x7ff : ((tmpsamYDelta < -0x7ff) ? -0x7ff : (int16_t)tmpsamYDelta);
  if (report->wheel) {
    tmpsamWheelDelta = samWheelDelta + report->wheel;
    samWheelDelta = (tmpsamWheelDelta > 63) ? 63 : ((tmpsamWheelDelta < -64) ? -64 : tmpsamWheelDelta);
  }
  mutex_exit(&samDeltaMutex);
}

static uint32_t extract_bits(uint8_t const* rpt,
                             uint16_t bit_offset,
                             uint8_t bit_size)
{
    uint32_t v = 0;
    uint16_t byte = bit_offset / 8;
    uint8_t shift = bit_offset % 8;

    int bytes = (shift + bit_size + 7) / 8;

    for (int i = 0; i < bytes; i++) {
        v |= (uint32_t)rpt[byte + i] << (8 * i);
    }

    v >>= shift;
    uint32_t mask = (bit_size == 32) ? 0xFFFFFFFF : ((1u << bit_size) - 1);
    return v & mask;
}

void decode_report(int instance,
                   uint8_t const* data,
                   uint16_t len)
{
  hid_mouse_report_t m = {0};
  (void) len;
  if (matchedfields.x.instance == instance && (matchedfields.x.report_id == -1 || matchedfields.x.report_id == data[0])) {
    m.x = extract_bits(data, matchedfields.x.bit_offset, matchedfields.x.bit_size);
  }
  if (matchedfields.y.instance == instance && (matchedfields.y.report_id == -1 || matchedfields.y.report_id == data[0])) {
    m.y = extract_bits(data, matchedfields.y.bit_offset, matchedfields.y.bit_size);
  }
  if (matchedfields.buttons.instance == instance && (matchedfields.buttons.report_id == -1 || matchedfields.buttons.report_id == data[0])) {
    m.buttons = extract_bits(data, matchedfields.buttons.bit_offset, matchedfields.buttons.bit_size);
  }
  if (matchedfields.wheel.instance == instance && (matchedfields.wheel.report_id == -1 || matchedfields.wheel.report_id == data[0])) {
    m.wheel = extract_bits(data, matchedfields.wheel.bit_offset, matchedfields.wheel.bit_size);
  }
 // Now pass the boot-style struct to the existing mouse handler
  processMouse(&m);
}
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len)
{
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  switch (itf_protocol)
  {
  case HID_ITF_PROTOCOL_MOUSE:
    decode_report(instance, report, len);
    break;
  case HID_ITF_PROTOCOL_KEYBOARD:
    // do nothing
  default:
    decode_report(instance, report, len);
    break;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    // do we need to do something about this error?
    return;
  }
}
