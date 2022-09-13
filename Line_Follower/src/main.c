#include <avr/io.h>
#include <avr/interrupt.h>
#include "serial_printf.h"
#include <util/delay.h>
#include "timer_tools.h"
#define LED PB4
float Kp = 50;
float Ki = 0.001;
float Kd = 130;
int w0=0,w1=0,w2=0,w3=4,w4=0;
int ADCREAD(uint8_t adctouse)
{
    int ADCval;
    ADMUX = adctouse;         // use ADC
    ADMUX |= (1 << REFS0);    // use AVcc as the reference
    ADMUX &= ~(1 << ADLAR);   
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescale for 8Mhz
    ADCSRA |= (1 << ADEN);    // Enable the ADC
    ADCSRA |= (1 << ADSC);    // Start the ADC conversion
    while(ADCSRA & (1 << ADSC));   
    ADCval = ADCL;
    ADCval = (ADCH << 8) + ADCval;   
    return ADCval;
}
void set_LED(uint8_t on)
{
  if (on) {
    PORTB |= (1 << LED);
  } else {
    PORTB &= ~(1 << LED);
  }
}
void pinWrite(int pin_number, int state)    {
    if  (pin_number <= 7 &&  state == 1) {
      PORTD |= 1 << pin_number;
      return;
    }
    if  (pin_number <= 7 &&  state == 0) {
      PORTD &= ~(1 << pin_number);
      return;
    }
    if  (pin_number <= 13 && pin_number >= 8 &&  state == 1) {
      PORTB |= 1 << (pin_number-8);
      return;
    } 
    if  (pin_number <= 13 && pin_number >= 8 &&  state == 0) {
      PORTB &= ~(1 << (pin_number-8));
      return;
    }
    if  (pin_number <= 0 && state == 0) {
      PORTC &= ~(1 << (-pin_number));
      return;
    }
    if  (pin_number <= 0 && state == 1) {
      PORTC = 1;
      return;
    }
}
int pinRead(int pin_number)    {
    if (pin_number <= 7)    {
        return (1 && (PIND & (1 << pin_number))); 
    }
    if (pin_number <= 13 && pin_number >7)    {
        return (1 && (PINB & (1 << (pin_number-8)))); 
    }
    return -1;  
}
void pinInput(int pin_number)   {
    if (pin_number <= 7) {
        DDRD &= ~(1 << pin_number);
        return;
    }
    if (pin_number <= 13 && pin_number >= 8) {
        DDRB &= ~(1 << (pin_number-8));
        return;
    }
    if(pin_number <= 0)  {
      DDRC &= ~(1 << (-pin_number));
      return;
    }  
}
void pinOutput(int pin_number)   {
    if (pin_number <= 7) {
        DDRD |= (1 << pin_number);
        return;
    }
    if (pin_number <= 13 && pin_number >= 8) {
        DDRB |= (1 << (pin_number-8));
        return;
    }
    if(pin_number <= 0)  {
      DDRC = 1;
      return;
    } 
}
void serial_start() {   
  uint8_t serial_data;
  if (serial_receive_ready()) {       
    serial_data = serial_receive();   
    if (serial_data == 'L') {  
      set_LED(1);
    } 
    else if (serial_data == 'l') {  
      set_LED(0);
    }
  }  
}
int pwm_generator(float speed,float freq, mili_timer* timer1)  {
  float period = (1.0/freq)*1000.0;
  float duty_cycle = speed/255.0;
  if (get_timer(timer1))  {
    start_timer(timer1,period);
  }
  return (get_timer_time(timer1) < (duty_cycle * period));
}
void pwmA_motor() {
  TCCR0A |= 1<<(COM0A1) | 1<<(WGM00) | 1<<(WGM01);
	TCCR0B |= 1<<(CS00);
}
int pwmA(int v)  {
  OCR0A = v;
  return 0;
}
void pwmB_motor() {
  TCCR2A |= 1<<(COM2B1) | 1<<(WGM20) | 1<<(WGM21);
  TCCR2B |= 1<<(CS20);
}
int pwmB(int v) {
  OCR2B = v;
  return 0;
}
int PID(int posicao,int* last_proportional, int* last_integral, int* last_derivative)  {
  int proportional = posicao; 
  int integral = *last_integral + proportional;
  int derivative = proportional - *last_proportional;
  *last_integral = integral;
  *last_proportional = proportional;
  *last_derivative = derivative;
  int power_error = proportional * Kp + integral * Ki + derivative * Kd;
  return power_error;
}
int main(void)
{                        
  sei();                              // começa interrupçoes
  set_LED(1); 
  serial_start();
  pinOutput(3);
  pinOutput(2);
  pinOutput(7);
  pinOutput(0);
  pinOutput(6);
  pinOutput(5);
  pinOutput(4);
  PORTC = 0;
  int base_power = 70;
  int power_error = 0;
  int integral=0, derivative=0, proportional=0;
  pinWrite(4,0);
  pinWrite(0,1);
  pinWrite(2,0);
  pinWrite(7,1);
  pinWrite(5,1);
  pwmA_motor();
  pwmB_motor();
  while (1) {                         
    int  value0 =1024- ADCREAD(0);
    int  value1 =1024- ADCREAD(1);
    int  value2 =1024-ADCREAD(2);
    int  value3 = 1024-ADCREAD(3);
    int  value4 = 1024-ADCREAD(4);
    value0 -= w0;
    value1 -= w1;
    value2 -= w2;
    value3 -= w3;
    value4 -= w4;
    int posicao = (0L * (long)value0 + 1000L * (long)value1 + 2000L * (long)value2 + 3000L * (long)value3 +4000L * (long)value4) / (value0 + value1 + value2 + value3 + value4);
    power_error = PID(posicao,&proportional, &integral, &derivative);
    int speedA = base_power - power_error;
    if(speedA>240) speedA=240;
    if(speedA<0)  speedA=0; 
    int speedB = base_power + power_error;
    if(speedB>240) speedB=240;
    if(speedB<0)  speedB=0; 
    pwmA(speedA);
    pwmB(speedB);
  }
}