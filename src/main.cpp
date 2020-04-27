/*
=========================================================================
  Ultrasonic Parametric Speaker Source Code for STM32
  Copyright (C) 2019,2020 Gene Ruebsamen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

==========================================================================
*/

#include <Arduino.h>
#include <STM32ADC.h>

// 1800 = 40Khz, 3000 = 24Khz
#define PWM_OVERFLOW 1800
#define PWM_OUT PA8       //PWM output
#define PWM_OUT_COMP PB13 //complementary output
#define ANALOG_PIN PA7
#define maxSamples 1

HardwareTimer hTimer1 = HardwareTimer(1);
STM32ADC myADC(ADC1);
char sinetable [32];

uint16_t buffer[maxSamples];
uint8_t count = 0;
uint8 pins = 7;

void isr(void);

uint8_t  sine_wave[16] = {
0x80,0xb0,0xda,0xf5,0xff,0xf5,0xda,0xb0,
0x80,0x4f,0x25,0x0a,0x00,0x0a,0x25,0x4f
};

void setup() {
  pinMode(PA7, INPUT_ANALOG);  // setup PIN A7 for analog in
  pinMode(PWM_OUT, PWM);  // Uses Timer1 / Channel 3
  pinMode(PWM_OUT_COMP, PWM);
  pinMode(PC13, OUTPUT);  // LED out

 /*
  * Set STM32 Timer 1 prescale factor to 1 and choose PWM overflow
  * 40kHz transducers should use a PWM_OVERFLOW of 1800, while 24kHz 
  * transducers need to use 3000.  This is based upon the 72 Mhz clock
  * of the STM32. (70 Mhz / <Transducer Freq>) = PWM OVERFLOW
  *
  */

  hTimer1.pause();
  hTimer1.setPrescaleFactor(1);  // Timer PreScale Factor 1
  hTimer1.setOverflow(PWM_OVERFLOW);  // 72000000 / PWM OVERFLOW = 40Khz
  hTimer1.setMode(4,TIMER_OUTPUT_COMPARE);
  hTimer1.setCompare(4,PWM_OVERFLOW);
  hTimer1.attachInterrupt(4,isr); // Attach ISR

  timer_dev *t = TIMER1; //refers t to Timer 1 memory location
  timer_reg_map r = t->regs;
  
 /* 
  * Enable complimentary outputs for improved efficiency when driving
  * the TC4427A 
  *
  */
  bitSet(r.adv->CCER,0); //this should enable complimentary outputs
  bitSet(r.adv->CCER,2);

  //hTimer1.setPeriod(25); // calculate overflow based upon period
  hTimer1.refresh();
  hTimer1.resume();
  pwmWrite(PWM_OUT,PWM_OVERFLOW/2);  // 50% duty cycle

 /*
  * Set ADC to free running mode and transfer data via DMA
  */ 

  Timer3.setPeriod(1000 / 200);
  Timer3.setMasterModeTrGo(TIMER_CR2_MMS_UPDATE);

  myADC.calibrate();
  myADC.setSampleRate(ADC_SMPR_1_5);
  myADC.setPins(&pins,1);
  myADC.setDMA(buffer, maxSamples, (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_HALF_TRNS | DMA_TRNS_CMPLT), NULL);
  myADC.setTrigger(ADC_EXT_EV_TIM3_TRGO);
  myADC.startConversion();
}

void loop() {
    // Blink the LED so we know we haven't crashed
    digitalWrite(PC13, HIGH);
    delay(1000);
    digitalWrite(PC13, LOW);
    delay(1000);
}

/* 
 * Fire Interrupt on Timer Overflow 
 * MODULATION SCHEME - change output duty cycle based on adc reading
 */
void isr(void) {
  // convert frequency to Pulse Width Modulation Duty Cycle Using DMA
  // center on PWM_OVERFLOW
  uint16_t pDuty = (uint16_t)map(buffer[0],0,4095,0,PWM_OVERFLOW/2-1);
  pwmWrite(PWM_OUT,PWM_OVERFLOW/2+pDuty);

  // alternate modulation #2 - louder but worse quality
  //uint16_t pDuty = (uint16_t)map(buffer[0],0,4095,0,PWM_OVERFLOW);
  //pwmWrite(PWM_OUT,pDuty-1);

  // alternate #3
  //pwmWrite(PWM_OUT,(((PWM_OVERFLOW-2) * buffer[0]/4096))+1);


  // slower method performing an ADC on each interrupt
  //int16_t pDuty = (PWM_OVERFLOW -2)/2 * sine_wave[count++]/255;
  //int16_t pDuty = (PWM_OVERFLOW -2)/2 * ((float)analogRead(PA7)/4095);
  //int16_t pDuty = (PWM_OVERFLOW -2)/2 * ((float)analogRead(PA7)/4095);
  //pwmWrite(PWM_OUT,pDuty + (PWM_OVERFLOW / 2));

  // sine wave test
  //int16_t pDuty = PWM_OVERFLOW/2 * sine_wave[count++]/128;
  //pwmWrite(PWM_OUT,pDuty);
  //if(count >= 15) 
  //  count = 0;
}
