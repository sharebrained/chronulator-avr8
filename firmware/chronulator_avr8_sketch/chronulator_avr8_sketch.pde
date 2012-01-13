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

static const unsigned short servo_cycle_time = 20000;

static const unsigned short servo_center_minutes = 1500;
static const unsigned short servo_center_hours = 1500;

static const unsigned short servo_range_minutes = 900;
static const unsigned short servo_range_hours = 900;

typedef enum meter_mode {
  METER_MODE_SHOW_TIME = 0,
  METER_MODE_CALIBRATE_ZERO_SCALE = 1,
  METER_MODE_CALIBRATE_FULL_SCALE = 2,
} meter_mode_t;

static meter_mode_t meter_mode = METER_MODE_SHOW_TIME;

class Time {
public:
	Time() :
		hour_(maximumHours / 2),
		minute_(minutesPerHour / 2),
		second_(0),
		tick_(0) {
	}

	void set_hour(unsigned char new_value) {
	  hour_ = new_value;
	}

	unsigned char get_hour() const {
	  return hour_;
	}

	void set_minute(unsigned char new_value) {
	  minute_ = new_value;
	}

	unsigned char get_minute() const {
	  return minute_;
	}

	void set_second(unsigned char new_value) {
	  second_ = new_value;
	}

	unsigned char get_second() const {
	  return second_;
	}

	void set_tick(unsigned char new_value) {
	  tick_ = new_value;
	}

	unsigned char get_tick() const {
	  return tick_;
	}

	void add_hour() {
	  if( get_hour() < (maximumHours - 1) ) {
	    set_hour(get_hour() + 1);
	  } else {
	    set_hour(0);
	  }
	}

	void subtract_hour() {
	  if( get_hour() > 0 ) {
	    set_hour(get_hour() - 1);
	  } else {
	    set_hour(maximumHours - 1);
	  }
	}

	void add_minute() {
	  if( get_minute() < (minutesPerHour - 1) ) {
	    set_minute(get_minute() + 1);
	  } else {
	    set_minute(0);
	  }
	}

	void subtract_minute() {
	  if( get_minute() > 0 ) {
	    set_minute(get_minute() - 1);
	  } else {
	    set_minute(minutesPerHour - 1);
	  }
	}

	void tick() {
		advance_tick();
	}

	static const unsigned char ticksPerSecond = 128;
	static const unsigned char secondsPerMinute = 60;
	static const unsigned char minutesPerHour = 60;
	static const unsigned char maximumHours = 12;

private:
	unsigned char hour_;
	unsigned char minute_;
	unsigned char second_;
	unsigned char tick_;

	void advance_hour() {
      add_hour();
	}

	void advance_minute() {
      add_minute();
      if( get_minute() == 0 ) {
	    advance_hour();
	  }
	}

	void advance_second() {
	  add_second();
	  if( get_second() == 0 ) {
		advance_minute();
	  }
	}

	void advance_tick() {
	  add_tick();
	  if( get_tick() == 0 ) {
		advance_second();
	  }
	}

	void add_second() {
	  if( get_second() < (secondsPerMinute - 1) ) {
		set_second(get_second() + 1);
	  } else {
		set_second(0);
	  }
	}

	void add_tick() {
	  if( get_tick() < (ticksPerSecond - 1) ) {
		set_tick(get_tick() + 1);
	  } else {
		set_tick(0);
	  }
	}
};

#define METER_M OCR2A
#define METER_H OCR2B
#define METER_S OCR0A
#define METER_MS OCR0B

#define SERVO_M OCR1A
#define SERVO_H OCR1B

#define SERVO_M_POWER_PORT (PORTC)
#define SERVO_M_POWER_PORT_BIT (_BV(PORTC2))
#define SERVO_M_POWER_DDR (DDRC)
#define SERVO_M_POWER_DDR_BIT (_BV(DDC2))

#define SERVO_H_POWER_PORT (PORTC)
#define SERVO_H_POWER_PORT_BIT (_BV(PORTC3))
#define SERVO_H_POWER_DDR (DDRC)
#define SERVO_H_POWER_DDR_BIT (_BV(DDC3))

#define HOUR_PULSE_PORT (PORTD)
#define HOUR_PULSE_PORT_BIT (_BV(PORTD4))

#define MINUTE_PULSE_PORT (PORTD)
#define MINUTE_PULSE_PORT_BIT (_BV(PORTD2))

void set_start_of_hour(const bool value) {
  if( value ) {
    HOUR_PULSE_PORT |= HOUR_PULSE_PORT_BIT;
  } else {
    HOUR_PULSE_PORT &= ~HOUR_PULSE_PORT_BIT;
  }
}

void set_start_of_minute(const bool value) {
  if( value ) {
    MINUTE_PULSE_PORT |= MINUTE_PULSE_PORT_BIT;
  } else {
    MINUTE_PULSE_PORT &= ~MINUTE_PULSE_PORT_BIT;
  }
}

static const unsigned short servo_offset_minutes = servo_center_minutes - (servo_range_minutes / 2);
static const unsigned short servo_offset_hours = servo_center_hours - (servo_range_hours / 2);

static const unsigned short servo_scale_minutes = servo_range_minutes / (Time::minutesPerHour - 1);
static const unsigned short servo_scale_hours = servo_range_hours / (Time::maximumHours - 1);

unsigned short meter_m_value(unsigned char minute) {
  // 0-60 mapped to 0-255
  return (minute * 4) + (minute / 4);
}

unsigned short meter_h_value(unsigned char hour) {
  // 0-12 mapped to 0-255
  return (hour * 21) + (hour / 4);
}

unsigned short meter_s_value(unsigned char second) {
  // 0-60 mapped to 0-255
  return (second * 4) + (second / 4);
}

unsigned short meter_ms_value(unsigned char tick) {
  // 0-128 mapped to 0-255
  if( tick > 127 ) {
    return 255;
  } else {
    return tick * 2;
  }
}

unsigned short servo_h_value(unsigned char hour) {
  return servo_offset_hours + hour * servo_scale_hours;
}

unsigned short servo_m_value(unsigned char minute) {
  return servo_offset_minutes + minute * servo_scale_minutes;
}

static Time time;

static unsigned char servo_h_power_counter = 0;
static const unsigned char servo_h_power_timeout = 128;

void enable_servo_h_power() {
    servo_h_power_counter = 0;
    SERVO_H_POWER_PORT |= SERVO_H_POWER_PORT_BIT;
}

void disable_servo_h_power() {
    SERVO_H_POWER_PORT &= ~SERVO_H_POWER_PORT_BIT;
}

void update_servo_h_power() {
    if( servo_h_power_counter < servo_h_power_timeout ) {
        servo_h_power_counter += 1;
    } else {
        disable_servo_h_power();
    }
}

static unsigned char servo_m_power_counter = 0;
static const unsigned char servo_m_power_timeout = 128;

void enable_servo_m_power() {
    servo_m_power_counter = 0;
    SERVO_M_POWER_PORT |= SERVO_M_POWER_PORT_BIT;
}

void disable_servo_m_power() {
    SERVO_M_POWER_PORT &= ~SERVO_M_POWER_PORT_BIT;
}

void update_servo_m_power() {
    if( servo_m_power_counter < servo_m_power_timeout ) {
        servo_m_power_counter += 1;
    } else {
        disable_servo_m_power();
    }
}

void update_servos_power() {
    update_servo_h_power();
    update_servo_m_power();
}

void set_hour_indicators(const unsigned char hour) {
    static unsigned char last_value = 255;
    if( hour != last_value ) {
        last_value = hour;
        METER_H = meter_h_value(hour);
        SERVO_H = servo_h_value(hour);
        enable_servo_h_power();
    }
}

void set_minute_indicators(const unsigned char minute) {
    static unsigned char last_value = 255;
    if( minute != last_value ) {
        last_value = minute;
        METER_M = meter_m_value(minute);
        SERVO_M = servo_m_value(minute);
        enable_servo_m_power();
    }
}

void set_second_indicators(const unsigned char second) {
    static unsigned char last_value = 255;
    if( second != last_value ) {
        last_value = second;
        METER_S = meter_s_value(second);
    }
}

void set_tick_indicators(const unsigned char tick) {
    // No need to track last value -- this value will probably
    // ALWAYS change, so we'll always update it, regardless.
    METER_MS = meter_ms_value(tick);
}

void update_hour_indicators() {
    unsigned char value;
    switch( meter_mode ) {
    default:
    case METER_MODE_SHOW_TIME:
        value = time.get_hour();
        break;
        
    case METER_MODE_CALIBRATE_ZERO_SCALE:
        value = 0;
        break;
        
    case METER_MODE_CALIBRATE_FULL_SCALE:
        value = Time::maximumHours;
        break;
    }
    set_hour_indicators(value);
}

void update_minute_indicators() {
    unsigned char value;
    switch( meter_mode ) {
    default:
    case METER_MODE_SHOW_TIME:
        value = time.get_minute();
        break;
        
    case METER_MODE_CALIBRATE_ZERO_SCALE:
        value = 0;
        break;
        
    case METER_MODE_CALIBRATE_FULL_SCALE:
        value = Time::minutesPerHour;
        break;
    }
    set_minute_indicators(value);
}

void update_second_indicators() {
    unsigned char value;
    switch( meter_mode ) {
    default:
    case METER_MODE_SHOW_TIME:
        value = time.get_second();
        break;
        
    case METER_MODE_CALIBRATE_ZERO_SCALE:
        value = 0;
        break;
        
    case METER_MODE_CALIBRATE_FULL_SCALE:
        value = Time::secondsPerMinute;
        break;
    }
    set_second_indicators(value);
}

void update_tick_indicators() {
    unsigned char value;
    switch( meter_mode ) {
    default:
    case METER_MODE_SHOW_TIME:
        value = time.get_tick();
        break;
        
    case METER_MODE_CALIBRATE_ZERO_SCALE:
        value = 0;
        break;
        
    case METER_MODE_CALIBRATE_FULL_SCALE:
        value = Time::ticksPerSecond;
        break;
    }
    set_tick_indicators(value);
}

void update_cuckoo_signals() {
    if( time.get_second() == 0 ) {
        set_start_of_minute(true);
        if( time.get_minute() == 0 ) {
            set_start_of_hour(true);
        } else {
            set_start_of_hour(false);
        }
    } else {
        set_start_of_minute(false);
        set_start_of_hour(false);
    }
}

void update() {
    update_hour_indicators();
    update_minute_indicators();
    update_second_indicators();
    update_tick_indicators(); 
    
    update_cuckoo_signals();  
}

void set_mode_show_time() {
  meter_mode = METER_MODE_SHOW_TIME;
}

void set_mode_calibrate_zero_scale() {
  meter_mode = METER_MODE_CALIBRATE_ZERO_SCALE;
}

void set_mode_calibrate_full_scale() {
  meter_mode = METER_MODE_CALIBRATE_FULL_SCALE;
}

void hours_button_pressed() {
    if( meter_mode == METER_MODE_SHOW_TIME ) {
        time.add_hour();
    }
}

void minutes_button_pressed() {
    if( meter_mode == METER_MODE_SHOW_TIME ) {
        time.add_minute();
    }
}

void both_buttons_pressed() {
    switch( meter_mode ) {
    case METER_MODE_SHOW_TIME:
        set_mode_calibrate_zero_scale();
        break;
        
    case METER_MODE_CALIBRATE_ZERO_SCALE:
        set_mode_calibrate_full_scale();
        break;
        
    case METER_MODE_CALIBRATE_FULL_SCALE:
        set_mode_show_time();
        break;
        
    default:
        set_mode_show_time();
        break;
    }
}

#define HOURS_BUTTON_PORT PINB
#define HOURS_BUTTON_BIT _BV(PINB0)

#define MINUTES_BUTTON_PORT PIND
#define MINUTES_BUTTON_BIT _BV(PIND7)

bool is_hours_button_pressed() {
    return (HOURS_BUTTON_PORT & HOURS_BUTTON_BIT) ? false : true;
}

bool is_minutes_button_pressed() {
    return (MINUTES_BUTTON_PORT & MINUTES_BUTTON_BIT) ? false : true;
}

typedef enum button_mode {
    BUTTON_MODE_NONE = 0,
    BUTTON_MODE_HOURS = 1,
    BUTTON_MODE_MINUTES = 2,
    BUTTON_MODE_BOTH = 3,
} button_mode_t;

static const unsigned char debounce_wait = 8;
static unsigned char debounce_counter = 0;

static button_mode_t button_mode = BUTTON_MODE_NONE;

button_mode_t read_button_mode() {
    const int result = 
        (is_hours_button_pressed()   ? BUTTON_MODE_HOURS   : 0) |
        (is_minutes_button_pressed() ? BUTTON_MODE_MINUTES : 0);
    return (button_mode_t)result;
}

void fire_button_event(const button_mode_t button_mode) {
    switch( button_mode ) {
    case BUTTON_MODE_HOURS:
        hours_button_pressed();
        break;
        
    case BUTTON_MODE_MINUTES:
        minutes_button_pressed();
        break;
        
    case BUTTON_MODE_BOTH:
        both_buttons_pressed();
        break;
        
    default:
        break;
    }
}

void debounce_buttons() {
    const button_mode_t button_mode_before = button_mode;
    const button_mode_t button_mode_now = read_button_mode();
    if( button_mode_now != button_mode ) {
        debounce_counter += 1;
        if( debounce_counter == debounce_wait ) {
            // Button mode is stable, allow it to change.
            button_mode = button_mode_now;
            if( button_mode_before == BUTTON_MODE_NONE ) {
                // fire an event if the prior state was "no buttons pressed".
                fire_button_event(button_mode);
            }
        }
    } else {
        debounce_counter = 0;
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

void enable_timer0() {
  power_timer0_enable();
  
  TIMSK0 = 0;
  
  TCNT0 = 0;  
  OCR0A = 0;
  OCR0B = 0;
  
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) /*| _BV(WGM01)*/ | _BV(WGM00);
  TCCR0B = _BV(CS01) | _BV(CS00);
  
  TIFR0 = 0;
}

void enable_s_and_ms_meters() {
  enable_timer0();

  // TODO: PORTD settings?
  DDRD |= _BV(DDD6) | _BV(DDD5);
}

void enable_servos() {
  power_timer1_enable();
  
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);     // set prescaler of 8
  ICR1 = servo_cycle_time;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
}

typedef enum _PowerMode {
  POWER_MODE_LOW_POWER,
  POWER_MODE_HIGH_POWER
} PowerMode;

static PowerMode power_mode = POWER_MODE_LOW_POWER;

void initialize_high_power_mode() {
  power_mode = POWER_MODE_HIGH_POWER;

  // Clock is internal RC at 8MHz, assuming operation from a voltage
  // supply >= 2.5V. (See data sheet section 29.3 "Speed Grades".)
  clock_prescale_set(clock_div_1);

  enable_s_and_ms_meters();
  enable_servos();

  // Use idle sleep mode, which allows all the peripherals to continue
  // operating during sleep. This is what enables lots of cool features
  // at the expensive of high power consumption -- too high to run from
  // batteries for a decent amount of time.
  set_sleep_mode(SLEEP_MODE_IDLE);
}

void initialize_low_power_mode() {
  power_mode = POWER_MODE_LOW_POWER;

  // Clock is internal RC at 8MHz, scaled to 4MHz to stay within
  // clock rate limitations for voltage supply < 2.5V.
  // (See data sheet section 29.3 "Speed Grades".)
  clock_prescale_set(clock_div_2);

  // Use power-save mode mode, which shuts down all sections of the AVR
  // except for the asynchronous clock (the 32.768kHz crystal), timer 2
  // (which is driving the hours/minutes meters), and a few other
  // uninteresting peripherals.
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
}

#define POWER_MODE_PIN_PORT (PINC)
#define POWER_MODE_PIN_BIT (_BV(PINC1))

void set_power_mode() {
  if( (POWER_MODE_PIN_PORT & POWER_MODE_PIN_BIT) == 0 ) {
    initialize_high_power_mode();
  } else {
    initialize_low_power_mode();
  }
}

ISR(TIMER2_OVF_vect) {
  time.tick();
}

void initializePorts() {
  // Configure I/O pins for lowest power.
  
  // PB0: I, pullup: Switch (DDB0=0, PB0=1)
  // PB1: O: OC1A PWM output (DDB1=1, PB1=0)
  // PB2: O: OC1B PWM output (DDB2=1, PB2=0)
  // PB3: O: OC2A PWM output, "minutes" meter (DDB3=1, PB3=0)
  // PB4: I:
  // PB5: I:
  // PB6: I: TOSC1 (crystal) (DDB6=0, PB6=0)
  // PB7: I: TOSC2 (crystal) (DDB7=0, PB7=0)
  DDRB = _BV(DDB2) | _BV(DDB1);
  PORTB = _BV(PORTB0);

  // PC0: I, pullup: High-power mode detect (DDC0=0, PC0=1)
  // PC1: I, pullup: DC plug present (DDC1=0, PC1=1)
  // PC2: O: Servo M power (DDC2=1, PC2=0)
  // PC3: O: Servo H power (DDC3=1, PC3=0)
  // PC4: I:
  // PC5: I:
  // PC6: I: RESET
  // PC7: I: (no pin)
  DDRC = _BV(DDC3) | _BV(DDC2);
  PORTC = _BV(PORTC1) | _BV(PORTC0);

  // PD0: I: RXD Serial RX (DDD0=0, PD0=1)
  // PD1: O: TXD Serial TX (DDD1=1, PD1=1)
  // PD2: I: Start-of-minute pulse (DDD2=1, PD2=0)
  // PD3: O: OC2B PWM output, "hours" meter (DDD3=1, PD3=0)
  // PD4: I: Start-of-hour pulse (DDD4=1, PD4=0)
  // PD5: O: OC0B PWM output, LED (DDD5=1, PD5=0)
  // PD6: O: OC0A PWM output, LED (DDD6=1, PD6=0)
  // PD7: I, pullup: Switch (DDD7=0, PD7=1)
  DDRD = _BV(DDD2) | _BV(DDD4);
  PORTD = _BV(PORTD7);

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

void setup() {
  clock_prescale_set(clock_div_2);
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);

  cli();

  initializePorts();
  
  initializeAnalogToMinimizePower();

  power_all_disable();
  
  initializeTimer2For32KHzCrystal();

  set_power_mode();
  
  sei();
}

void loop() {
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

  sleep_mode();

  // Timer 2 interrupt will wake the AVR from the above sleep.
  // The interrupt will execute and return, then this code will
  // be executed.

  debounce_buttons();

  // Update the meters and servos every 8th of a second, plenty
  // often for how fast meters and servos can actually react.
  if( (time.get_tick() & 31) == 0 ) {
    update();
  }

  update_servos_power();
}
