/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#include <Arduino.h>
#include "HX711.h"

// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU \
    ( \
    ARCH_ESPRESSIF || \
    defined(ARDUINO_ARCH_SAM)     || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_STM32)   || defined(TEENSYDUINO) \
    )

#if HAS_ATOMIC_BLOCK
// Acquire AVR-specific ATOMIC_BLOCK(ATOMIC_RESTORESTATE) macro.
#include <util/atomic.h>
#endif

#if FAST_CPU
// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539
uint8_t shiftInSlow(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(1);
        if(bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
        delayMicroseconds(1);
    }
    return value;
}
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftInSlow(data,clock,order)
#else
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftIn(data,clock,order)
#endif


HX711::HX711() {
}

HX711::~HX711() {
}

void HX711::begin(byte dout, byte pd_sck, byte gain) {
	PD_SCK = pd_sck;
	DOUT = dout;

	pinMode(PD_SCK, OUTPUT);
	pinMode(DOUT, INPUT_PULLUP);

	set_gain(gain);
}

bool HX711::is_ready() {
	return digitalRead(DOUT) == LOW;
}

void HX711::set_channel(byte channel) {
  GAIN = channel;
}

void HX711::set_gain(byte gain) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
      set_channel(1);
			break;
		case 64:		// channel A, gain factor 64
      set_channel(3);
			break;
		case 32:		// channel B, gain factor 32
      set_channel(2);
			break;
	}
}

byte HX711::get_gain() {
	switch (GAIN) {
		case 1:		// channel A, gain factor 128
      return 128;
		case 3:		// channel A, gain factor 64
      return 64;
		case 2:		// channel B, gain factor 32
      return 32;
	}
  return 0;
}


long HX711::read() {

	// Wait for the chip to become ready.
	wait_ready();

	// Define structures for reading data into.
	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;
  // we are going to store the current gain as the next gain.
  // keep track of what the gain is for this reading so we can
  // keep a rolling average below.
  byte last_gain = NEXT_GAIN;

	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read().  The ATOMIC_BLOCK macro disables
	// interrupts during the sequence and then restores the interrupt mask to its previous
	// state after the sequence completes, insuring that the entire read-and-gain-set
	// sequence is not interrupted.  The macro has a few minor advantages over bracketing
	// the sequence between `noInterrupts()` and `interrupts()` calls.
	#if HAS_ATOMIC_BLOCK
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

	#elif IS_FREE_RTOS
	// Begin of critical section.
	// Critical sections are used as a valid protection method
	// against simultaneous access in vanilla FreeRTOS.
	// Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
	// context switches and servicing of ISRs during a critical section.
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);

	#else
	// Disable interrupts.
	noInterrupts();
	#endif

	// Pulse the clock pin 24 times to read the data.
	data[2] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);
	data[1] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);
	data[0] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < GAIN; i++) {
		digitalWrite(PD_SCK, HIGH);
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
		digitalWrite(PD_SCK, LOW);
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
	}

  // reset the gain to mark what the next read will be (so we can check
  // if we need to throw out the next reading)
  NEXT_GAIN = GAIN;

	#if IS_FREE_RTOS
	// End of critical section.
	portEXIT_CRITICAL(&mux);

	#elif HAS_ATOMIC_BLOCK
	}

	#else
	// Enable interrupts again.
	interrupts();
	#endif

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<unsigned long>(filler) << 24
			| static_cast<unsigned long>(data[2]) << 16
			| static_cast<unsigned long>(data[1]) << 8
			| static_cast<unsigned long>(data[0]) );

  // Store the rolling average for this channel
	long reading = static_cast<long>(value);
  long count = static_cast<long>(ROLLING);
  VALUES[last_gain-1] = (VALUES[last_gain-1] * (count-1.0) + reading) / count;

  return reading;
}

void HX711::read_all() {
  // this function is counterintuitive.  Setting the gain sets it on the
  // NEXT read, so the initial read() call will be set based on NEXT_GAIN
  // while the GAIN changes will affect the read() after that.
  switch(NEXT_GAIN) {
     case 1:
       GAIN = 2;
       read();
       GAIN = 3;
       read();
       break;
     case 2:
       GAIN = 1;
       read();
       GAIN = 3;
       read();
       break;
     case 3:
       GAIN = 2;
       read();
       GAIN = 1;
       read();
       break;
  }
  read();
}

void HX711::wait_ready(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!is_ready()) {
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(delay_ms);
	}
}

bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	// https://github.com/bogde/HX711/issues/76
	int count = 0;
	while (count < retries) {
		if (is_ready()) {
			return true;
		}
		delay(delay_ms);
		count++;
	}
	return false;
}

bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (is_ready()) {
			return true;
		}
		delay(delay_ms);
	}
	return false;
}

long HX711::read_average(byte times) {
	long sum = 0;
	for (byte i = 0; i < times; i++) {
		sum += read();
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(0);
	}
	return sum / times;
}

#define CHANNEL_A 3 // Channel A, gain of 64
#define CHANNEL_B 2 // Channel B, gain of 32


double HX711::get_rolling() {
  return VALUES[GAIN-1] - OFFSET[GAIN-1];
}

double HX711::get_rolling_A() {
  return VALUES[CHANNEL_A-1] - OFFSET[CHANNEL_A-1];
}

double HX711::get_rolling_B() {
  return VALUES[CHANNEL_B-1] - OFFSET[CHANNEL_B-1];
}


double HX711::get_value(byte times) {
  if (GAIN != NEXT_GAIN) {
    read(); // read and throw away, resetting the Channel
  }
	return read_average(times) - OFFSET[GAIN-1];
}

float HX711::get_units(byte cache, byte times) {
  if (cache) {
    return (VALUES[GAIN-1] - OFFSET[GAIN-1]) / SCALE[GAIN-1];
  }
	return get_value(times) / SCALE[GAIN-1];
}


double HX711::get_value_A(byte times) {
  byte gain = GAIN;
  GAIN = CHANNEL_A;
  if (GAIN != NEXT_GAIN) {
    read(); // read and throw away, resetting the Channel
  }
  float value = read_average(times) - OFFSET[GAIN-1];
  GAIN = gain; // reset the gain
	return value;
}

double HX711::get_value_B(byte times) {
  byte gain = GAIN;
  GAIN = CHANNEL_B; // Channel A, gain of 64
  if (GAIN != NEXT_GAIN) {
    read(); // read and throw away, resetting the Channel
  }
  float value = read_average(times) - OFFSET[GAIN-1];
  GAIN = gain; // reset the gain
	return value;
}

float HX711::get_units_A(byte cache, byte times) {
  if (cache) {
    return (VALUES[CHANNEL_A-1] - OFFSET[CHANNEL_A-1]) / SCALE[CHANNEL_A-1];
  }
	return get_value_A(times) / SCALE[CHANNEL_A-1];
}

float HX711::get_units_B(byte cache, byte times) {
  if (cache) {
    return (VALUES[CHANNEL_B-1] - OFFSET[CHANNEL_B-1]) / SCALE[CHANNEL_B-1];
  }
	return get_value_B(times) / SCALE[CHANNEL_B-1];
}

// TARE / Zero
void HX711::tare(byte times) {
	double sum = read_average(times);
	set_offset(sum);
}

void HX711::tare_A(byte times) {
  OFFSET[CHANNEL_A-1] += get_value_A(times);
}

void HX711::tare_B(byte times) {
  OFFSET[CHANNEL_B-1] += get_value_B(times);
}

void HX711::set_scale(float scale) {
	SCALE[GAIN-1] = scale;
}

float HX711::get_scale() {
	return SCALE[GAIN-1];
}

void HX711::set_offset(long offset) {
	OFFSET[GAIN-1] = offset;
}

double HX711::get_offset() {
	return OFFSET[GAIN-1];
}

void HX711::set_value_A(long value) {
  SCALE[CHANNEL_A-1] = get_value_A() / value;
}

void HX711::set_value_B(long value) {
  SCALE[CHANNEL_B-1] = get_value_A() / value;
}

void HX711::power_down() {
	digitalWrite(PD_SCK, LOW);
	digitalWrite(PD_SCK, HIGH);
}

void HX711::power_up() {
	digitalWrite(PD_SCK, LOW);
}
