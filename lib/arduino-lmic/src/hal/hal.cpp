/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#include "hal.h"
#include "../lmic.h"
#include "../lmic/radio.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include <sys/time.h>

// -----------------------------------------------------------------------------
// I/O

OsTime last_int_trigger;
// (initialized by init() with radio RSSI, used by rand1())
uint8_t randbuf[16];

void hal_store_trigger() { last_int_trigger = os_getTime(); }

static void hal_io_init() {
  // NSS and DIO0 are required, DIO1 is required for LoRa
  ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
  ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
  ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN);

  pinMode(lmic_pins.nss, OUTPUT);
  if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
    pinMode(lmic_pins.rxtx, OUTPUT);
  if (lmic_pins.rst != LMIC_UNUSED_PIN)
    pinMode(lmic_pins.rst, OUTPUT);

  pinMode(lmic_pins.dio[0], INPUT);
  if (lmic_pins.dio[1] != LMIC_UNUSED_PIN)
    pinMode(lmic_pins.dio[1], INPUT);
}

// val == 1  => tx 1
void hal_pin_rxtx(uint8_t val) {
  if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
    digitalWrite(lmic_pins.rxtx, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst(uint8_t val) {
  if (lmic_pins.rst == LMIC_UNUSED_PIN)
    return;

  if (val == 0 || val == 1) { // drive pin
    pinMode(lmic_pins.rst, OUTPUT);
    digitalWrite(lmic_pins.rst, val);
  } else { // keep pin floating
    pinMode(lmic_pins.rst, INPUT);
  }
}

static bool dio_states[NUM_DIO] = {0};

void hal_io_check() {
  uint8_t i;
  for (i = 0; i < NUM_DIO; ++i) {
    if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
      continue;

    if (dio_states[i] != digitalRead(lmic_pins.dio[i])) {
      dio_states[i] = !dio_states[i];
      if (dio_states[i])
        LMIC.radio.irq_handler(i, last_int_trigger);
    }
  }
}

// -----------------------------------------------------------------------------
// SPI

static const SPISettings settings(10E6, MSBFIRST, SPI_MODE0);

static void hal_spi_init() { SPI.begin(); }

void hal_pin_nss(uint8_t val) {
  if (!val)
    SPI.beginTransaction(settings);
  else
    SPI.endTransaction();

  // Serial.println(val?">>":"<<");
  digitalWrite(lmic_pins.nss, val);
}

// perform SPI transaction with radio
uint8_t hal_spi(uint8_t out) {
  uint8_t res = SPI.transfer(out);
  /*
      Serial.print(">");
      Serial.print(out, HEX);
      Serial.print("<");
      Serial.println(res, HEX);
      */
  return res;
}

// -----------------------------------------------------------------------------
// TIME

bool is_sleep_allow = false;

bool hal_is_sleep_allow() { return is_sleep_allow; }

void hal_allow_sleep() { is_sleep_allow = true; }

void hal_forbid_sleep() { is_sleep_allow = false; }

OsTime hal_ticks() {
  timeval val;
  gettimeofday(&val, nullptr);

  // uint64_t time = esp_clk_rtc_time() >> US_PER_OSTICK_EXPONENT;
  return OsTime((val.tv_sec * OSTICKS_PER_SEC) +
                (val.tv_usec >> US_PER_OSTICK_EXPONENT));
}

void hal_waitUntil(OsTime const &time) {
  OsDeltaTime delta = time - hal_ticks();
  hal_wait(delta);
}

void hal_wait(OsDeltaTime delta) {
  // From delayMicroseconds docs: Currently, the largest value that
  // will produce an accurate delay is 16383.
  while (delta > OsDeltaTime::from_us(16000)) {
    delay(16);
    delta -= OsDeltaTime::from_us(16000);
  }
  if (delta > 0)
    delayMicroseconds(delta.to_us());
}

// check and rewind for target time
bool hal_checkTimer(OsTime const &time) {

  auto delta = time - hal_ticks();
  if (delta <= OsDeltaTime(0))
    return true;

  // HACK for a bug (I will track it down.)
  if (delta >= OsDeltaTime::from_sec(60 * 60)) {
    PRINT_DEBUG_2("WARN delta is too big, execute now ref : %u, delta : %i",
                  time.tick(), delta.tick());
    return true;
  }
  return false;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs() {
  noInterrupts();
  irqlevel++;
}

void hal_enableIRQs() {
  if (--irqlevel == 0) {
    interrupts();
  }
}

// -----------------------------------------------------------------------------

void hal_init() {
  // configure radio I/O and interrupt handler
  hal_io_init();
  // configure radio SPI
  hal_spi_init();
  // configure timer
}

void hal_init_random() {
  // not init
}

// use esp random
uint8_t hal_rand1() { return esp_random() & 0xFF; }

// use esp random
uint16_t hal_rand2() { return esp_random() & 0xFFFF; }

void hal_failed(const char *file, uint16_t line) {
#if defined(LMIC_FAILURE_TO)
  LMIC_FAILURE_TO.println("FAILURE ");
  LMIC_FAILURE_TO.print(file);
  LMIC_FAILURE_TO.print(':');
  LMIC_FAILURE_TO.println(line);
  LMIC_FAILURE_TO.flush();
#endif
  hal_disableIRQs();
  while (1)
    ;
}
