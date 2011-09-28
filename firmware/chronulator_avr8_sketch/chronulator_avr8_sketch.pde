/*
 **********************************************************
 *
 * Copyright 2008-2010 ShareBrained Technology, Inc.
 *
 * This file is part of chronulator-avr8.
 *
 * chronulator-avr8 is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * chronulator-avr8 is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General
 * Public License along with chronulator-avr8. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 **********************************************************
 */

#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>

#define VERSION_RELEASE "200812090122"
#define F_CPU 4000000

typedef enum meter_mode {
  METER_MODE_SHOW_TIME = 0,
  METER_MODE_CALIBRATE_ZERO_SCALE = 1,
  METER_MODE_CALIBRATE_FULL_SCALE = 2,
  METER_MODE_SERIAL_CONTROL = 3
} meter_mode_t;

static meter_mode_t meter_mode = METER_MODE_SHOW_TIME;

static const unsigned char debounce_wait = 4;

static unsigned char debounce_counter_s1 = 0;
static unsigned char debounce_counter_s2 = 0;

static bool hours_button_active = false;
static bool minutes_button_active = false;

#define METER_M OCR2A
#define METER_H OCR2B

void hours_button_pressed() {
  switch( meter_mode ) {
  case METER_MODE_SHOW_TIME:
    if( minutes_button_active ) {
      subtract_minute();
      set_mode_calibrate_zero_scale();
    } else {
      add_hour();
      show_time();
    }
    break;
    
  case METER_MODE_CALIBRATE_ZERO_SCALE:
    if( minutes_button_active ) {
      set_mode_calibrate_full_scale();
    }
    break;
    
  case METER_MODE_CALIBRATE_FULL_SCALE:
    if( minutes_button_active ) {
      set_mode_show_time();
    }
    break;
    
  default:
    break;
  }
}

void hours_button_released() {
}

void minutes_button_pressed() {
  switch( meter_mode ) {
  case METER_MODE_SHOW_TIME:
    if( hours_button_active ) {
      subtract_hour();
      set_mode_calibrate_zero_scale();
    } else {
      add_minute();
      show_time();
    }
    break;
    
  case METER_MODE_CALIBRATE_ZERO_SCALE:
    if( hours_button_active ) {
      set_mode_calibrate_full_scale();
    }
    break;
    
  case METER_MODE_CALIBRATE_FULL_SCALE:
    if( hours_button_active ) {
      set_mode_show_time();
    }
    break;
    
  default:
    break;
  }
}

void minutes_button_released() {
}

#define HOURS_BUTTON_PORT PINB
#define HOURS_BUTTON_BIT _BV(PINB0)

#define MINUTES_BUTTON_PORT PIND
#define MINUTES_BUTTON_BIT _BV(PIND7)

void debounce_buttons() {
  if( hours_button_active ) {
    if( HOURS_BUTTON_PORT & HOURS_BUTTON_BIT ) {
      hours_button_active = false;
      hours_button_released();
    }
  } else {
    if( (HOURS_BUTTON_PORT & HOURS_BUTTON_BIT) == 0 ) {
      debounce_counter_s1++;
      if( debounce_counter_s1 == debounce_wait ) {
        hours_button_active = true;
        hours_button_pressed();
      }
    } else {
      debounce_counter_s1 = 0;
    }
  }
  
  if( minutes_button_active ) {
    if( MINUTES_BUTTON_PORT & MINUTES_BUTTON_BIT ) {
      minutes_button_active = false;
      minutes_button_released();
    }
  } else {
    if( (MINUTES_BUTTON_PORT & MINUTES_BUTTON_BIT) == 0 ) {
      debounce_counter_s2++;
      if( debounce_counter_s2 == debounce_wait ) {
        minutes_button_active = true;
        minutes_button_pressed();
      }
    } else {
      debounce_counter_s2 = 0;
    }
  }
}

static const unsigned char ticksPerSecond = 128;
static const unsigned char secondsPerMinute = 60;
static const unsigned char minutesPerHour = 60;
static const unsigned char maximumHours = 12;

static unsigned char hour = maximumHours / 2;
static unsigned char minute = minutesPerHour / 2;
static unsigned char second = 0;
static unsigned char tick = 0;

static const unsigned char meter_scale_minutes = 4;
static const unsigned char meter_scale_hours = 20;

static unsigned char meter_m_value = 0;
static unsigned char meter_h_value = 0;

void show_time() {
  meter_m_value = minute * meter_scale_minutes;
  meter_h_value = hour * meter_scale_hours;
}

void set_mode_show_time() {
  meter_mode = METER_MODE_SHOW_TIME;
  show_time();
}

void set_mode_calibrate_zero_scale() {
  meter_mode = METER_MODE_CALIBRATE_ZERO_SCALE;
  meter_m_value = 0;
  meter_h_value = 0;
}

void set_mode_calibrate_full_scale() {
  meter_mode = METER_MODE_CALIBRATE_FULL_SCALE;
  meter_m_value = minutesPerHour * meter_scale_minutes;
  meter_h_value = maximumHours * meter_scale_hours;
}

void set_mode_serial_control() {
  if( meter_mode != METER_MODE_SERIAL_CONTROL ) {
    meter_mode = METER_MODE_SERIAL_CONTROL;
    meter_m_value = 0;
    meter_h_value = 0;
  }
}

void tick_hour() {
  if( hour < (maximumHours - 1) ) {
    hour++;
  } else {
    hour = 0;
  }
}

void tick_minute() {
  if( minute < (minutesPerHour - 1) ) {
    minute++;
  } else {
    minute = 0;
    tick_hour();
  }
}

void tick_second() {
  if( second < (secondsPerMinute - 1) ) {
    second++;
  } else {
    second = 0;
    tick_minute();
  }
}

void add_hour() {
  if( hour < (maximumHours - 1) ) {
    hour++;
  } else {
    hour = 0;
  }
}

void subtract_hour() {
  if( hour > 0 ) {
    hour--;
  } else {
    hour = maximumHours - 1;
  }
}

void add_minute() {
  if( minute < (minutesPerHour - 1) ) {
    minute++;
  } else {
    minute = 0;
  }
}

void subtract_minute() {
  if( minute > 0 ) {
    minute--;
  } else {
    minute = minutesPerHour - 1;
  }
}

void tick_tick() {
  if( tick < (ticksPerSecond - 1) ) {
    tick++;
  } else {
    tick = 0;
    tick_second();
    
    if( meter_mode == METER_MODE_SHOW_TIME ) {
      show_time();
    }
  }
}

bool sleepModeCausesSpuriousTimer2Interrupts() {
  switch( SMCR & (_BV(SM2) | _BV(SM1) | _BV(SM0)) ) {
  case (_BV(SM1) | _BV(SM0)):
  case (_BV(SM0)):
    // Power-save or ADC Noise Reduction
    return true;
    
  default:
    return false;
  }
}

void waitForTimer2CycleToEnd() {
  while(ASSR & (_BV(OCR2AUB) | _BV(OCR2BUB)));
}

static bool timer0_enabled = false;
static bool usart0_enabled = false;

void update_sleep_mode() {
  if( timer0_enabled || usart0_enabled ) {
    set_sleep_mode(SLEEP_MODE_IDLE);
  } else {
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  }
}

void enable_timer0() {
  power_timer0_enable();
  
  TIMSK0 = 0;
  
  TCNT0 = 0;  
  OCR0A = 0;
  OCR0B = 0;
  
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) /*| _BV(WGM01)*/ | _BV(WGM00);
  TCCR0B = _BV(CS01) | _BV(CS00);
  
  TIFR0 = 0;

  timer0_enabled = true;
  update_sleep_mode();  
}

void disable_timer0() {
  timer0_enabled = false;
  update_sleep_mode();
  
  power_timer0_disable();
}

static unsigned char led_backlight_m_value = 0;
static unsigned char led_backlight_h_value = 0;

void enable_led_backlights() {
  enable_timer0();

  // TODO: PORTD settings?
  DDRD |= _BV(DDD6) | _BV(DDD5);
}

void disable_led_backlights() {
  DDRD &= ~(_BV(DDD6) | _BV(DDD5));
  // TODO: PORTD settings?
  
  disable_timer0();
}

void update_led_backlights() {
  if( (led_backlight_m_value > 0) || (led_backlight_h_value > 0) ) {
    if( !timer0_enabled ) {
      enable_led_backlights();
    }
    OCR0A = led_backlight_m_value;
    OCR0B = led_backlight_h_value;
  } else {
    if( timer0_enabled ) {
      disable_led_backlights();
    }
  }
}

void set_led_h_brightness(const unsigned char brightness) {
  led_backlight_h_value = brightness;
  update_led_backlights();
}

void set_led_m_brightness(const unsigned char brightness) {
  led_backlight_m_value = brightness;
  update_led_backlights();
}

void enable_usart0() {
  power_usart0_enable();

  // PD0: I, pullup: RXD Serial RX (DDD0=0, PD0=1)
  // PD1: O: TXD Serial TX (DDD1=1, PD1=1)
  DDRD &= ~_BV(DDD0);
  DDRD |= _BV(DDD1);
  PORTD |= _BV(PD1) | _BV(PD0);
  
  Serial.begin(9600);
  
  // TODO: Reset command parser
  Serial.println(VERSION_RELEASE);
  Serial.println("READY");
  
  usart0_enabled = true;
  update_sleep_mode();
}

void disable_usart0() {
  usart0_enabled = false;
  update_sleep_mode();

  DDRD &= ~(_BV(DDD1) | _BV(DDD0));
  PORTD &= ~(_BV(PD1) | _BV(PD0));
  
  power_usart0_disable();
}

typedef enum _PowerMode {
  POWER_MODE_LOW_POWER,
  POWER_MODE_HIGH_POWER
} PowerMode;

static PowerMode power_mode = POWER_MODE_LOW_POWER;

void switch_to_high_power_mode() {
  clock_prescale_set(clock_div_1);
  enable_led_backlights();
  enable_usart0();
  power_mode = POWER_MODE_HIGH_POWER;
}

void switch_to_low_power_mode() {
  power_mode = POWER_MODE_LOW_POWER;
  clock_prescale_set(clock_div_2);
  disable_usart0();
  disable_led_backlights();
}

#define POWER_MODE_PIN_PORT (PINC)
#define POWER_MODE_PIN_BIT (_BV(PINC1))

void update_power_mode() {
  if( (POWER_MODE_PIN_PORT & POWER_MODE_PIN_BIT) == 0 ) {
    if( power_mode != POWER_MODE_HIGH_POWER ) {
      switch_to_high_power_mode();
    }
  } else {
    if( power_mode != POWER_MODE_LOW_POWER ) {
      switch_to_low_power_mode();
    }
  }
}

ISR(TIMER2_OVF_vect) {
  METER_M = meter_m_value;
  METER_H = meter_h_value;

  tick_tick();

  debounce_buttons();
  
  // Ensure TOSC cycle will not cause extra interrupts.
  // a. Write a value to TCCR2x, TCNT2, or OCR2x. 
  // b. Wait until the corresponding Update Busy Flag in ASSR returns to zero. 
  // c. Enter Power-save or ADC Noise Reduction mode.

  // Can't re-enter power save mode until the TOSC1 cycle that woke us is
  // complete. Use the writes to OCR2A/B, performed earlier in this interrupt
  // routine, to indicate when the cycle is over. The ASSR OCR2xUB flags will
  // clear when the cycle is over.
  if( sleepModeCausesSpuriousTimer2Interrupts() ) {
    waitForTimer2CycleToEnd();
  }
}

void initializePorts() {
  // Configure I/O pins for lowest power.
  
  // PB0: I, pullup: Switch (DDB0=0, PB0=1)
  // PB1: I: OC1A PWM output (DDB1=1, PB1=0)
  // PB2: I: OC1B PWM output (DDB2=1, PB2=0)
  // PB3: O: OC2A PWM output, "minutes" meter (DDB3=1, PB3=0)
  // PB4: I:
  // PB5: I:
  // PB6: I: TOSC1 (crystal) (DDB6=0, PB6=0)
  // PB7: I: TOSC2 (crystal) (DDB7=0, PB7=0)
  DDRB = _BV(DDB2) | _BV(DDB1);
  PORTB = _BV(PB0);

  // PC0: I, pullup: High-power mode detect (DDC0=0, PC0=1)
  // PC1: I, pullup: DC plug present (DDC1=0, PC1=1)
  // PC2: I:
  // PC3: I:
  // PC4: I:
  // PC5: I:
  // PC6: I: RESET
  // PC7: I: (no pin)
  DDRC = 0;
  PORTC = _BV(PC1) | _BV(PC0);

  // PD0: I: RXD Serial RX (DDD0=0, PD0=1)
  // PD1: O: TXD Serial TX (DDD1=1, PD1=1)
  // PD2: I:
  // PD3: O: OC2B PWM output, "hours" meter (DDD3=1, PD3=0)
  // PD4: I:
  // PD5: O: OC0B PWM output, LED (DDD5=1, PD5=0)
  // PD6: O: OC0A PWM output, LED (DDD6=1, PD6=0)
  // PD7: I, pullup: Switch (DDD7=0, PD7=1)
  DDRD = 0;
  PORTD = _BV(PD7);

  // Do not disable internal pull-up resistors on ports.
  MCUCR &= ~_BV(PUD);
}

void initializeAnalogToMinimizePower() {
  // Turning off analog stuff saves 120uA:
  ACSR |= _BV(ACD); // Disable analog comparator.
  //ACSR &= ~_BV(ACBG);	// Select analog input as reference (instead of bandgap).
  //DIDR1 = _BV(AIN1D) | _BV(AIN0D); // Disable digital input buffers on analog inputs.
  ADMUX &= ~(_BV(REFS1) | _BV(REFS0)); // Turn off AREF, internal Vref.
  ADCSRA &= ~_BV(ADEN); // Disable ADC.
}

void initializeTimer2For32KHzCrystal() {
  power_timer2_enable();

  // Timer 2 async setup (section 17.9)
  // a. Disable the Timer/Counter2 interrupts by clearing OCIE2x and TOIE2.
  TIMSK2 = 0;
  
  // b. Select clock source by setting AS2 as appropriate.
  ASSR = _BV(AS2);
  
  // b(2). Wait for clock to come up.
  _delay_ms(1000);
  
  // c. Write new values to TCNT2, OCR2x, and TCCR2x.
  TCNT2 = 0;
  OCR2A = 0;
  OCR2B = 0;

  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  
  // d. To switch to asynchronous operation: Wait for TCN2xUB, OCR2xUB, and TCR2xUB.
  while( ASSR & (_BV(TCN2UB) | _BV(OCR2AUB) | _BV(OCR2BUB) | _BV(TCR2AUB) | _BV(TCR2BUB)) );
  
  // e. Clear the Timer/Counter2 Interrupt Flags.
  TIFR2 = 0;
  
  // f. Enable interrupts, if needed.
  TIMSK2 = _BV(TOIE2);

  DDRB |= _BV(DDB3);
  DDRD |= _BV(DDD3);
}

bool is_digit(const char c) {
  return (c >= '0') && (c <= '9');
}

int digit_to_integer(const char c) {
  if( is_digit(c) ) {
    return c - '0';
  } else {
    return -1;
  }
}

class CommandBuffer {
public:
  CommandBuffer() {
    reset();
  }
  
  bool add_character(const char c) {
    if( is_room_in_buffer() ) {
      buffer[buffer_position] = c;
      buffer_position += 1;
      return true;
    } else {
      return false;
    }
  }
  
  bool is_room_in_buffer() {
    static const unsigned char buffer_size = sizeof(buffer) / sizeof(*buffer);
    return buffer_position < buffer_size;
  }
  
  void reset() {
    buffer_position = 0;
  }
  
  void print() const {
    for( int i=0; i<buffer_position; i++ ) {
      Serial.print(buffer[i]);
    }
  }
  
  bool startsWithIgnoreCase(const char * compare) const {
    int compare_len = strlen(compare);
    if( buffer_position < compare_len ) {
      return false;
    }
    
    return strncasecmp(buffer, compare, compare_len) == 0;
  }
  
  int readUnsignedIntegerAt(const int position) const {
    int value = 0;
    int i;
    for( int i=position; i<buffer_position; i++ ) {
      const int digit_value = digit_to_integer(buffer[i]);
      if( digit_value != -1 ) {
        value = (value * 10) + digit_value;
      } else {
        break;
      }
    }

    if( i == position ) {
      // We didn't even read a single digit before something went
      // wrong. Return an error value.
      return -1;
    } else {
      return value;
    }
  }
  
private:
  char buffer[16];
  unsigned char buffer_position;
};

// TODO: Should this be necessary?
extern "C" {
  void __cxa_pure_virtual() {
    Serial.println("FATAL");
  }
}

class Command {
public:
  virtual bool match(const CommandBuffer & buffer) = 0;
  virtual bool execute(const CommandBuffer & buffer) = 0;
};

class CommandHV : public Command {
public:
  bool match(const CommandBuffer & buffer) {
    return buffer.startsWithIgnoreCase("hv=");
  }
  
  bool execute(const CommandBuffer & buffer) {
    int value = buffer.readUnsignedIntegerAt(3);
    if( value >= 0 ) {
      if( value < 256 ) {
        set_mode_serial_control();
        meter_h_value = value;
        return true;
      }
    }
    
    return false;
  }
};

class CommandHL : public Command {
public:
  bool match(const CommandBuffer & buffer) {
    return buffer.startsWithIgnoreCase("hl=");
  }
  
  bool execute(const CommandBuffer & buffer) {
    int value = buffer.readUnsignedIntegerAt(3);
    if( value >= 0 ) {
      if( value < 256 ) {
        set_led_h_brightness(value);
        return true;
      }
    }
    
    return false;
  }
};

class CommandMV : public Command {
public:
  bool match(const CommandBuffer & buffer) {
    return buffer.startsWithIgnoreCase("mv=");
  }
  
  bool execute(const CommandBuffer & buffer) {
    int value = buffer.readUnsignedIntegerAt(3);
    if( value >= 0 ) {
      if( value < 256 ) {
        set_mode_serial_control();
        meter_m_value = value;
        return true;
      }
    }
    
    return false;
  }
};


class CommandML : public Command {
public:
  bool match(const CommandBuffer & buffer) {
    return buffer.startsWithIgnoreCase("ml=");
  }
  
  bool execute(const CommandBuffer & buffer) {
    int value = buffer.readUnsignedIntegerAt(3);
    if( value >= 0 ) {
      if( value < 256 ) {
        set_led_m_brightness(value);
        return true;
      }
    }
    
    return false;
  }
};

class CommandT : public Command {
public:
  bool match(const CommandBuffer & buffer) {
    return buffer.startsWithIgnoreCase("t");
  }
  
  bool execute(const CommandBuffer & buffer) {
    set_mode_show_time();
  }
};

CommandHV command_hv;
CommandHL command_hl;
CommandMV command_mv;
CommandML command_ml;
CommandT command_t;
Command * commands[] = { &command_hv,
                         &command_mv,
                         &command_hl,
                         &command_ml,
                         &command_t };

class CommandParser {
public:
  CommandParser() {
    reset();
  }
  
  void feed(const char c) {
    if( (c == '\r') || (c == '\n') || (c == '.') ) {
      handle_return();
    } else {
      if( buffer.add_character(c) == false ) {
        error = true;
      }
    }
  }
  
private:
  void handle_return() {
    if( error ) {
      print_error();
    } else {
      execute_command();
    }
    
    reset();
  }
  
  void reset() {
    buffer.reset();
    error = false;
  }
  
  void execute_command() {
    static const int command_count = sizeof(commands) / sizeof(*commands);
    
    int i;
    bool success = false;
    for( i=0; i<command_count; i++ ) {
      if( commands[i]->match(buffer) ) {
        success = commands[i]->execute(buffer);
        break;
      }
    }
    
    if( i == command_count ) {
      Serial.println("UNKNOWN");
    } else {
      if( success ) {
        print_success();
      } else {
        print_error();
      }
    }
  }
  
  void print_success() {
    Serial.println("OK");
  }
  
  void print_error() {
    Serial.println("ERROR");
  }

  CommandBuffer buffer;
  bool error;
};

CommandParser command_parser;

void parse_serial() {

  while( Serial.available() ) {
    const char c = Serial.read();
    command_parser.feed(c);
  }
}

void setup() {
  clock_prescale_set(clock_div_2);
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);

  cli();

  initializePorts();
  
  initializeAnalogToMinimizePower();

  power_all_disable();
  
  initializeTimer2For32KHzCrystal();

  update_power_mode();
  
  sei();
}

void loop() {
  sleep_mode();
  
  if( usart0_enabled ) {
    parse_serial();
  }
}
