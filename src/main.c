#define F_CPU 16000000UL

#define TESTING 1

// HX711
#define HX711_DT PD2
#define HX711_SCK PD3

// H-bridge Motor driver
#define MOTOR_IN1 PB1
#define MOTOR_IN2 PB2

// ADC
#define CURRENT_SENSOR_ADC 0

// Control parameters

// ADC constants
#define ADC_REF_VOLTAGE 5.0
#define ADC_RESOLUTION 1023.0

// Tensiometer deflection angle (degrees)
#define DEFLECTION_ANGLE_DEG 84.4f
#define DEFLECTION_ANGLE_RAD (DEFLECTION_ANGLE_DEG * 3.14159265f / 180.0f)

// Current sensor sensitivity (mV/mA), calibrated at runtime
// Default fallback, overwritten by calibration
float current_sensitivity = 1.0f;

// Mechanical parameters
// Kt = 0.00417 Nm/A = 0.00000417 Nm/mA (6V 1/120 variant, motor shaft)
#define MOTOR_Kt 0.00000417f
#define GEAR_RATIO 120.0f
#define WINCH_RADIUS 0.025f

// Kalman parameters
#define KALMAN_Q 5.0f
#define KALMAN_R_LOADCELL 1.0f
#define KALMAN_R_CURRENT 50.0f

// PID parameters
#define PID_SETPOINT 2.5f

#define PID_OUTER_KP 15.2f
#define PID_OUTER_KI 3.8f
#define PID_OUTER_KD 2.1f
#define PID_OUTER_DT 0.0125f // 80SPS HX711

#define PID_INNER_KP 2.0f
#define PID_INNER_KI 0.5f
#define PID_INNER_KD 0.0f
#define PID_INNER_DT 0.001f

// Safety thresholds
#define TENSION_MIN 1.0f
#define TENSION_MAX 7.0f
#define TENSION_EMERGENCY 10.0f

// UART
#define BAUD 9600
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)

// Motor
#define MOTOR_PWM_TOP 799     // 16MHz / (799+1) = 20kHz, 800 steps
#define MOTOR_CURR_TOP 600.0f // 600mA

#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdint.h>
#include <util/delay.h>

// Funtion prototypes

// Motor
void motor_init(void);
void motor_forward(uint16_t speed);
void motor_reverse(uint16_t speed);
void motor_stop(void);

// HX711
void hx711_init(void);
long hx711_read_raw(void);
long hx711_average(uint8_t samples);
void hx711_tare(void);
float hx711_get_mass_g(void);
float hx711_get_force_N(void);

// ADC
void adc_init(void);
uint16_t adc_read(uint8_t channel);

// Current sensor
void current_init(void);
float current_read_mA(void);
float current_get_force_N(float current_mA);

// Control functions
void fusion_kalman_predict(void);
float fusion_kalman_update(float measurement, float R);
int16_t pid_inner_compute(float measured);
float pid_outer_compute(float measured);

// UART
void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_print_float(float val, uint8_t decimals);
void uart_print_pad(float val, uint8_t decimals, uint8_t width);
void uart_print_uint8_pad(uint8_t val);

// Global variables
long hx711_offset = 0;
float calibration_factor = 714.5;
float current_zero_mV = 0.0f;
uint16_t current_zero_adc = 0;

volatile uint8_t print_flag = 0;
volatile uint8_t timer2_ovf_count = 0;
volatile uint8_t motor_timeout = 0;
volatile uint8_t motor_timeout_count = 0;

float kalman_x = 0.0f;
float kalman_p = 1000.0f;
uint8_t kalman_initialized = 0;

float pid_inner_integral = 0.0f;
float pid_inner_prev_error = 0.0f;

float pid_outer_integral = 0.0f;
float pid_outer_prev_error = 0.0f;

volatile float current_inner_mA = 0.0f;
volatile int16_t pwm_inner = 0;
volatile float pid_inner_setpoint_mA = 0.0f;

ISR(TIMER0_COMPA_vect) {
  float current = current_read_mA();

  // EMA filter
  current_inner_mA = 0.3f * current + 0.7f * current_inner_mA;

  // Inner PID: current error → PWM
  pwm_inner = pid_inner_compute(current_inner_mA);
  motor_drive(pwm_inner);
}

// Timer2 overflow ISR: 16MHz/1024/256 = ~61Hz, count 6 overflows = ~10Hz print
ISR(TIMER2_OVF_vect) {
  if (++timer2_ovf_count >= 2) {
    timer2_ovf_count = 0;
    print_flag = 1;
    if (++motor_timeout_count >= 450) // ~15 seconds at 30Hz
      motor_timeout = 1;
  }
}

// ---------------- MAIN CONTROL ----------------

#if TESTING
// Average ADC over N samples, returns mV offset from baseline
float calib_avg_mV(uint16_t samples) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += adc_read(CURRENT_SENSOR_ADC);
    _delay_ms(5);
  }
  float avg_adc = (float)sum / (float)samples;
  float mV = (avg_adc * 5000.0f) / 1023.0f;
  return mV - current_zero_mV;
}

int main(void) {
  uart_init();
  motor_init();
  adc_init();
  _delay_ms(1000);
  current_init();

  uart_puts("\r\nCurrent sensor calibration\r\n");
  uart_puts("PWM steps: +100, +200, +300, -100, -200, -300\r\n");
  uart_puts("Measure actual current with multimeter for each step\r\n");
  uart_puts("Format: PWM | delta_mV (from baseline)\r\n\r\n");

  _delay_ms(2000);

  int16_t steps[] = {100, 200, 300, -100, -200, -300};
  float delta_mV[6];

  for (uint8_t s = 0; s < 6; s++) {
    motor_drive(steps[s]);
    _delay_ms(10000); // settle

    delta_mV[s] = calib_avg_mV(200);

    uart_puts("PWM=");
    uart_print_pad((float)steps[s], 0, 5);
    uart_puts(" delta_mV=");
    uart_print_pad(delta_mV[s], 2, 8);
    uart_puts("\r\n");
  }

  motor_stop();

  uart_puts("\r\nDone. Enter measured mA values for each step.\r\n");
  uart_puts("Compute sensitivity = delta_mV / mA for each point,\r\n");
  uart_puts("then average to get CURRENT_SENSITIVITY.\r\n");

  // Hold: print summary on loop so you can read it
  while (1) {
    _delay_ms(5000);
    uart_puts("\r\n--- Summary ---\r\n");
    uart_puts("Baseline mV=");
    uart_print_pad(current_zero_mV, 1, 7);
    uart_puts("\r\n");
    for (uint8_t s = 0; s < 6; s++) {
      uart_puts("PWM=");
      uart_print_pad((float)steps[s], 0, 5);
      uart_puts(" delta_mV=");
      uart_print_pad(delta_mV[s], 2, 8);
      uart_puts("\r\n");
    }
  }
}

#else

int main(void) {
  uart_init();
  hx711_init();
  motor_init();
  adc_init();

  _delay_ms(1000);

  current_init();

  // Timer2: normal mode, prescaler 1024, overflow interrupt
  TCCR2A = 0;
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 = (1 << TOIE2);

  // Timer0 CTC setup: prescaler 64, OCR0A=249 → 1kHz
  TCCR0A = (1 << WGM01);              // CTC mode
  TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);

  sei();

  hx711_tare();

  uart_puts("\r\nReady\r\n");

  while (1) {
    // Read force, convert to wire tension (all in N)
    long raw = hx711_read_raw();
    float mass_g = (raw - hx711_offset) / calibration_factor;
    float force_loadcell = mass_g * 0.00981f;
    if (force_loadcell < 0.0f)
      force_loadcell = 0.0f;
    float tension = force_loadcell / (2.0f * sinf(DEFLECTION_ANGLE_RAD / 2.0f));
    cli();
    float curr_snapshot = current_inner_mA;
    sei();
    float force_current = current_get_force_N(curr_snapshot);

    if (!kalman_initialized && tension > 0.5f) {
      kalman_x = tension;
      kalman_p = 1000.0f;
      kalman_initialized = 1;
    }

    float force_fused = kalman_x;
    if (kalman_initialized) {
      fusion_kalman_predict();
      fusion_kalman_update(tension, KALMAN_R_LOADCELL);
      force_fused = fusion_kalman_update(force_current, KALMAN_R_CURRENT);
    }

    float desired_mA = pid_outer_compute(force_fused);
    pid_inner_setpoint_mA = desired_mA;

    if (force_fused > TENSION_EMERGENCY) {
      motor_stop();
      pid_inner_setpoint_mA = 0.0f;
      pid_inner_integral = 0.0f;
      pid_outer_integral = 0.0f;
    }

    if (motor_timeout) {
      motor_stop();
      pid_inner_setpoint_mA = 0.0f;
      pid_inner_integral = 0.0f;
      pid_outer_integral = 0.0f;
    }

    // Printing data
    if (print_flag) {
      print_flag = 0;
      uart_puts("T_l=");
      uart_print_pad(tension, 3, 8);
      uart_puts(" T_c=");
      uart_print_pad(force_current, 3, 8);
      uart_puts(" T_k=");
      uart_print_pad(force_fused, 3, 8);
      uart_puts(" PWM=");
      uart_print_pad((float)pwm_inner, 0, 4);
      uart_puts("\r\n");
    }
  }
}

#endif

// ---------------- MOTOR PWM ----------------

void motor_init(void) {
  DDRB |= (1 << MOTOR_IN1) | (1 << MOTOR_IN2);

  // TIMER1 Fast PWM, ICR1 as TOP, non-inverting on OC1A and OC1B
  // 16MHz / (799+1) = 20kHz, 800 steps resolution
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // prescaler 1
  ICR1 = MOTOR_PWM_TOP;

  OCR1A = 0;
  OCR1B = 0;
}

void motor_drive(int16_t pwm) {
  if (pwm > 0) {
    OCR1A = (uint16_t)pwm;
    OCR1B = 0;
  } else if (pwm < 0) {
    OCR1A = 0;
    OCR1B = (uint16_t)(-pwm);
  } else {
    OCR1A = 0;
    OCR1B = 0;
  }
}

void motor_forward(uint16_t speed) {
  PORTB |= (1 << MOTOR_IN1);
  PORTB &= ~(1 << MOTOR_IN2);
  OCR1B = speed;
}

void motor_reverse(uint16_t speed) {
  PORTB &= ~(1 << MOTOR_IN1);
  PORTB |= (1 << MOTOR_IN2);
  OCR1B = speed;
}

void motor_stop(void) {
  OCR1A = 0;
  OCR1B = 0;
}

// ---------------- HX711 ----------------

void hx711_init(void) {
  DDRD &= ~(1 << HX711_DT); // DT input
  DDRD |= (1 << HX711_SCK); // SCK output

  PORTD &= ~(1 << HX711_SCK);
}

long hx711_read_raw(void) {
  long count = 0;

  while (PIND & (1 << HX711_DT))
    ; // wait until data ready

  for (uint8_t i = 0; i < 24; i++) {
    PORTD |= (1 << HX711_SCK);
    _delay_us(1);

    count = count << 1;

    PORTD &= ~(1 << HX711_SCK);
    _delay_us(1);

    if (PIND & (1 << HX711_DT)) {
      count++;
    }
  }

  // extra clock pulse: gain 128, channel A
  PORTD |= (1 << HX711_SCK);
  _delay_us(1);
  PORTD &= ~(1 << HX711_SCK);

  // convert 24-bit signed value
  if (count & 0x800000) {
    count |= 0xFF000000;
  }

  return count;
}

long hx711_average(uint8_t samples) {
  long sum = 0;

  for (uint8_t i = 0; i < samples; i++) {
    sum += hx711_read_raw();
  }

  return sum / samples;
}

void hx711_tare(void) { hx711_offset = hx711_average(10); }

float hx711_get_mass_g(void) {
  long raw = hx711_average(10);
  return (raw - hx711_offset) / calibration_factor;
}

float hx711_get_force_N(void) {
  float mass_g = hx711_get_mass_g();
  float mass_kg = mass_g / 1000.0;
  return mass_kg * 9.81;
}

// #---------------------ADC--------------------------
void adc_init(void) {
  ADMUX = (1 << REFS0); // AVCC as reference
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) |
           (1 << ADPS0); // Enable ADC, prescaler 128
}

uint16_t adc_read(uint8_t channel) {
  channel &= 0x07;

  ADMUX = (ADMUX & 0xF0) | channel;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  return ADC;
}

void current_init(void) {
  uint32_t sum = 0;
  for (uint16_t s = 0; s < 512; s++) {
    sum += adc_read(CURRENT_SENSOR_ADC);
  }
  float avg_adc = (float)sum / 512.0f;
  current_zero_adc = (uint16_t)(avg_adc + 0.5f);
  current_zero_mV = (avg_adc * 5000.0f) / 1023.0f;

  uart_puts("Baseline ADC=");
  uart_print_pad(avg_adc, 1, 7);
  uart_puts(" mV=");
  uart_print_pad(current_zero_mV, 1, 7);
  uart_puts("\r\n");
}

float current_read_mA(void) {
  uint16_t adc_val = adc_read(CURRENT_SENSOR_ADC);
  float mV = (adc_val * 5000.0f) / 1023.0f;
  float mA = (mV - current_zero_mV) / current_sensitivity;
  return mA;
}

float current_get_force_N(float current_mA) {
  float torque = MOTOR_Kt * current_mA * GEAR_RATIO;
  return torque / WINCH_RADIUS;
}

// ---------------- CONTROL -------------

void fusion_kalman_predict(void) { kalman_p += KALMAN_Q; }

float fusion_kalman_update(float measurement, float R) {
  float K = kalman_p / (kalman_p + R);
  kalman_x += K * (measurement - kalman_x);
  kalman_p *= (1.0f - K);
  return kalman_x;
}

int16_t pid_inner_compute(float measured) {
  float error = pid_inner_setpoint_mA - measured;

  float derivative = (error - pid_inner_prev_error) / PID_INNER_DT;
  pid_inner_prev_error = error;

  // Compute P+D first, then see how much room integral has
  float pd_output = PID_INNER_KP * error + PID_INNER_KD * derivative;

  // Only accumulate integral if output not saturated (back-calculation
  // anti-windup)
  float i_term = PID_INNER_KI * pid_inner_integral;
  float output = pd_output + i_term;

  if (output > -(float)MOTOR_PWM_TOP && output < (float)MOTOR_PWM_TOP) {
    pid_inner_integral += error * PID_INNER_DT;
  }

  // Clamp integral symmetrically
  float i_max = (float)MOTOR_PWM_TOP * 0.5f / PID_INNER_KI;
  if (pid_inner_integral > i_max)
    pid_inner_integral = i_max;
  if (pid_inner_integral < -i_max)
    pid_inner_integral = -i_max;

  // Recompute with clamped integral
  output = pd_output + PID_INNER_KI * pid_inner_integral;

  // Clamp output: -MOTOR_PWM_TOP to +MOTOR_PWM_TOP
  if (output > (float)MOTOR_PWM_TOP)
    output = (float)MOTOR_PWM_TOP;
  if (output < -(float)MOTOR_PWM_TOP)
    output = -(float)MOTOR_PWM_TOP;

  return (int16_t)output;
}

float pid_outer_compute(float measured) {
  float error = PID_SETPOINT - measured;

  float derivative = (error - pid_outer_prev_error) / PID_OUTER_DT;
  pid_outer_prev_error = error;

  // Compute P+D first, then see how much room integral has
  float pd_output = PID_OUTER_KP * error + PID_OUTER_KD * derivative;

  // Only accumulate integral if output not saturated (back-calculation
  // anti-windup)
  float i_term = PID_OUTER_KI * pid_outer_integral;
  float output = pd_output + i_term;

  if (output > -MOTOR_CURR_TOP && output < MOTOR_CURR_TOP) {
    pid_outer_integral += error * PID_OUTER_DT;
  }

  // Clamp integral symmetrically
  float i_max = MOTOR_CURR_TOP * 0.5f / PID_OUTER_KI;
  if (pid_outer_integral > i_max)
    pid_outer_integral = i_max;
  if (pid_outer_integral < -i_max)
    pid_outer_integral = -i_max;

  // Recompute with clamped integral
  output = pd_output + PID_OUTER_KI * pid_outer_integral;

  // Clamp output: -MOTOR_CURR_TOP to +MOTOR_CURR_TOP
  if (output > MOTOR_CURR_TOP)
    output = MOTOR_CURR_TOP;
  if (output < -MOTOR_CURR_TOP)
    output = -MOTOR_CURR_TOP;

  return output;
}

// ---------------- UART ----------------

void uart_init(void) {
  UBRR0H = (uint8_t)(UBRR_VAL >> 8);
  UBRR0L = (uint8_t)(UBRR_VAL);
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

void uart_putc(char c) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = c;
}

void uart_puts(const char *s) {
  while (*s) {
    uart_putc(*s++);
  }
}

void uart_print_float(float val, uint8_t decimals) {
  if (val < 0.0f) {
    uart_putc('-');
    val = -val;
  }

  long integer_part = (long)val;
  float frac = val - (float)integer_part;

  // print integer part
  char buf[12];
  uint8_t i = 0;
  if (integer_part == 0) {
    buf[i++] = '0';
  } else {
    while (integer_part > 0) {
      buf[i++] = '0' + (integer_part % 10);
      integer_part /= 10;
    }
  }
  for (int8_t j = i - 1; j >= 0; j--) {
    uart_putc(buf[j]);
  }

  if (decimals > 0) {
    uart_putc('.');
    for (uint8_t d = 0; d < decimals; d++) {
      frac *= 10.0f;
      uint8_t digit = (uint8_t)frac;
      uart_putc('0' + digit);
      frac -= digit;
    }
  }
}

void uart_print_pad(float val, uint8_t decimals, uint8_t width) {
  // count chars needed
  float tmp = (val < 0.0f) ? -val : val;
  uint8_t len = 0;
  if (val < 0.0f)
    len++;
  long ip = (long)tmp;
  if (ip == 0) {
    len++;
  } else {
    long t = ip;
    while (t > 0) {
      len++;
      t /= 10;
    }
  }
  if (decimals > 0)
    len += 1 + decimals;
  while (len < width) {
    uart_putc(' ');
    len++;
  }
  uart_print_float(val, decimals);
}

void uart_print_uint8_pad(uint8_t val) {
  if (val >= 100) {
    uart_putc('0' + val / 100);
    uart_putc('0' + (val / 10) % 10);
  } else if (val >= 10) {
    uart_putc(' ');
    uart_putc('0' + val / 10);
  } else {
    uart_putc(' ');
    uart_putc(' ');
  }
  uart_putc('0' + val % 10);
}
