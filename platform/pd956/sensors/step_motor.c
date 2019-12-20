/*
 * Copyright © 2019, Peter Mikkelsen
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "contiki-conf.h"
#include "platform-conf.h"
#include "compiler.h"
#include "lib/sensors.h"
#include "step_motor.h"
#include "gpio.h"
#include <stdint.h>
#include "board-peripherals.h"

#ifdef NODE_STEP_MOTOR
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BIT_B2 PIO_PA6_IDX //J03 => step1_GPIO on module
#define BIT_A2 PIO_PA8_IDX //J04 => step2_GPIO on module
#define BIT_B1 PIO_PA9_IDX //J05 => step3_GPIO on module
#define BIT_A1 PIO_PA10_IDX//J07 => step4_GPIO on module

#define step1_GPIO            (1ul<<BIT_B2)
#define step2_GPIO            (1ul<<BIT_A2)
#define step3_GPIO            (1ul<<BIT_B1)
#define step4_GPIO            (1ul<<BIT_A1)

Pio *step_base = (Pio *)PIOA;


static int sensor_status = SENSOR_STATUS_DISABLED;
volatile int32_t stepPosition ,stepPosition_request;
volatile uint8_t Motor_running;
// Contains data for timer interrupt.
speedRampData srd;

void speed_cntr_Move(int step, int accel, int decel, int speed);



// Table with control signals for stepper motor
//(NB: should be 32 bit, but since the highest pin is 10 we can save the flash)
const uint16_t steptabel[] = {
	((1<<BIT_A1) | (0<<BIT_A2) | (0<<BIT_B1) | (0<<BIT_B2)),
	((1<<BIT_A1) | (0<<BIT_A2) | (1<<BIT_B1) | (0<<BIT_B2)),
	((0<<BIT_A1) | (0<<BIT_A2) | (1<<BIT_B1) | (0<<BIT_B2)),
	((0<<BIT_A1) | (1<<BIT_A2) | (1<<BIT_B1) | (0<<BIT_B2)),
	((0<<BIT_A1) | (1<<BIT_A2) | (0<<BIT_B1) | (0<<BIT_B2)),
	((0<<BIT_A1) | (1<<BIT_A2) | (0<<BIT_B1) | (1<<BIT_B2)),
	((0<<BIT_A1) | (0<<BIT_A2) | (0<<BIT_B1) | (1<<BIT_B2)),
	((1<<BIT_A1) | (0<<BIT_A2) | (0<<BIT_B1) | (1<<BIT_B2))};


/*---------------------------------------------------------------------------*/
void driver_StepCounter(uint8_t dir)
{
	if(dir)		stepPosition++;
	else		stepPosition--;

	step_base->PIO_ODSR = (unsigned)steptabel[stepPosition&7];
}

/*---------------------------------------------------------------------------*/
int
configure_step(void)
{
	//pio_set_input(PIOA,PIO_PA3 | PIO_PA28,0); // NB: pa3 and pa28 is connected to pa9 and pa10.

	// Enable PIO to controle the pin
	step_base->PIO_PER  = step1_GPIO | step2_GPIO | step3_GPIO | step4_GPIO;
	// Output and low
	step_base->PIO_OER  = step1_GPIO | step2_GPIO | step3_GPIO | step4_GPIO;
	step_base->PIO_CODR = step1_GPIO | step2_GPIO | step3_GPIO | step4_GPIO;



	pmc_enable_periph_clk(step_TIMER_ID);
	//937500Hz
	#if !LOW_CLOCK //120Mhz
		step_TIMER.TC_CMR= 3 + (2<<13); //3=MCK/128  2<<13=up rc compare
	#else //30Mhz
		step_TIMER.TC_CMR= 2 + (2<<13); //2=MCK/32   2<<13=up rc compare
	#endif

	step_TIMER.TC_IER=1<<4; //CPCS
	NVIC_ClearPendingIRQ(step_TIMER_IRQ);
	NVIC_SetPriority((IRQn_Type) step_TIMER_ID, 0);//level 0 is the highest interrupt priority (0-15)
	NVIC_EnableIRQ(step_TIMER_IRQ);

	return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static int
step_init(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			configure_step();
			stepPosition_request = stepPosition=0;
			sensor_status = SENSOR_STATUS_INITIALISED;

			// No one but the motor driver uses PIO_ODSR. Atmels framework will
			// for some reason set this doing initialization. As a hotfix clear all bits.
			// NB: This is taken care of but the comment stays as a reminder
			//step_base->PIO_OWDR = 0xffffffff;

			break;

		case SENSORS_ACTIVE:
			if(sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			if(enable==1) {
				//Enables writing PIO_ODSR for the I/O line
				step_base->PIO_OWER = step1_GPIO | step2_GPIO | step3_GPIO | step4_GPIO;
				step_base->PIO_ODSR = (unsigned)steptabel[stepPosition&7];
				speed_cntr_Move(stepPosition_request-stepPosition, 5, 5, 40);
				sensor_status = SENSOR_STATUS_READY;
			}else if(enable == 10){
				srd.run_state = STOP;
			} else {
				//Disables writing PIO_ODSR for the I/O line
				step_base->PIO_OWDR = step1_GPIO | step2_GPIO | step3_GPIO | step4_GPIO;
				sensor_status = SENSOR_STATUS_INITIALISED;
			}
			break;
	}
	return sensor_status;
}
/*---------------------------------------------------------------------------*/
int
step_set(int value)
{
	if(value != SENSOR_ERROR){
		if(Motor_running)
			return SENSOR_ERROR;

		stepPosition_request = value;
		//step_base->PIO_ODSR = (unsigned)steptabel[stepPosition&7];
		//speed_cntr_Move(value-stepPosition, 20, 20, 40);

		// freq = srd.min_delay/speed
		// =>
		// freq = speed * FSPR /(pi*100) => 40*4096/(pi*100) = 521Hz (motor is rated to 600Hz)
		// 40*0.01*9.55 = 3.82rpm    60[s]/3.82rmp = 15.7[s]
	}

	return stepPosition;
}
/*---------------------------------------------------------------------------*/
static int
step_status(int type)
{
	return sensor_status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(step_sensor, "Step motor", step_set, step_init, step_status);
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
// - AppNote:            AVR446 - Linear speed control of stepper motor

static unsigned long sqrt_lc(unsigned long x)
{
	register unsigned long xr;  // result register
	register unsigned long q2;// scan-bit register
	register unsigned char f;// flag (one bit)

	xr = 0;// clear result
	q2 = 0x40000000L;// higest possible result bit
	do
	{
		if((xr + q2) <= x){
			x -= xr + q2;
			f = 1;                  // set flag
		}
		else{
			f = 0;                  // clear flag
		}
		xr >>= 1;
		if(f){
			xr += q2;               // test flag
		}
	}while(q2 >>= 2);          // shift twice

	if(xr < x){
		return xr +1;             // add for rounding
	} else{
		return xr;
	}
}


#define BACKWARD	0
#define FORWARD 	1

#define TRUE 1
#define FALSE 0






/*! \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelerate with given acceleration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param step  Number of steps to move (pos = FORWARD, neg = BACKWARD).
 *  \param accel  Acceleration to use, in 0.01*rad/sec^2.
 *  \param decel  Deceleration to use, in 0.01*rad/sec^2.
 *  \param speed  Max speed, in 0.01*rad/sec.
 *
 *  1 rad/sec = 9.55 rpm
 *
 *  for this motor 4096 steps is 2*pi rad
 */
void speed_cntr_Move(int step, int accel, int decel, int speed)
{
	//! Number of steps before we hit max speed.
	int max_s_lim;
	//! Number of steps before we must start deceleration (if accel does not hit max speed).
	int accel_lim;

	// Set direction from sign on step value.
	if(step < 0){
		srd.dir = BACKWARD;
		step = -step;
	}
	else{
		srd.dir = FORWARD;
	}

	// If moving only 1 step.
	if(step == 1){
		// Move one step...
		srd.accel_count = -1;
		// ...in DECEL state.
		srd.run_state = DECEL;
		// Just a short delay so main() can act on 'running'.
		srd.step_delay = 1000;
		Motor_running = TRUE;
		step_TIMER.TC_RC = 10;
		// Run Timer/Counter
		step_TIMER.TC_CCR=1;
		step_TIMER.TC_CCR=4;
	}
	// Only move if number of steps to move is not zero.
	if(step == 0)
		return;

	// Set max speed limit, by calc min_delay to use in timer.
	// min_delay = (alpha / tt)/ w
	srd.min_delay = A_T_x100 / speed;

	// Set accelration by calc the first (c0) step delay .
	// step_delay = 1/tt * sqrt_lc(2*alpha/accel)
	// step_delay = ( tfreq*0.676/100 )*100 * sqrt_lc( (2*alpha*10000000000) / (accel*100) )/10000
	srd.step_delay = (T1_FREQ_148 * sqrt_lc(A_SQ / accel))/100;

	// Find out after how many steps does the speed hit the max speed limit.
	// max_s_lim = speed^2 / (2*alpha*accel)
	max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);
	// If we hit max speed limit before 0,5 step it will round to 0.
	// But in practice we need to move atleast 1 step to get any speed at all.
	if(max_s_lim == 0){
		max_s_lim = 1;
	}

	// Find out after how many steps we must start deceleration.
	// n1 = (n1+n2)decel / (accel + decel)
	accel_lim = (step*decel) / (accel+decel);
	// We must accelerate at least 1 step before we can start deceleration.
	if(accel_lim == 0){
		accel_lim = 1;
	}

	// Use the limit we hit first to calc decel.
	if(accel_lim <= max_s_lim){
		srd.decel_val = accel_lim - step;
	}
	else{
		srd.decel_val = -(max_s_lim*accel)/decel;
	}
	// We must decelerate at least 1 step to stop.
	if(srd.decel_val == 0){
		srd.decel_val = -1;
	}

	// Find step to start deceleration.
	srd.decel_start = step + srd.decel_val;

	// If the maximum speed is so low that we don't need to go via acceleration state.
	if(srd.step_delay <= srd.min_delay){
		srd.step_delay = srd.min_delay;
		srd.run_state = RUN;
	}
	else{
		srd.run_state = ACCEL;
	}

	// Reset counter.
	srd.accel_count = 0;
	Motor_running = TRUE;
	step_TIMER.TC_RC = 10;
	// Set Timer/Counter
	step_TIMER.TC_CCR=1;
	step_TIMER.TC_CCR=4;
}


void TC2_Handler(void)
{
	step_TIMER.TC_SR;
	// Holds next delay period.
	int new_step_delay = 0;
	// Remember the last step delay used when accelerating.
	static int last_accel_delay;
	// Counting steps when moving.
	static int step_count = 0;
	// Keep track of remainder from new_step-delay calculation to increase accuracy
	static int rest = 0;

	step_TIMER.TC_RC = srd.step_delay;

	switch(srd.run_state) {
		case STOP:
		  step_count = 0;
		  rest = 0;
		  // Stop Timer/Counter 1.
		  step_TIMER.TC_CCR=2;
		  Motor_running = FALSE;
		  step_base->PIO_ODSR = 0;
		  sensors_changed(&step_sensor);
		  break;

		case ACCEL:
		  driver_StepCounter(srd.dir);
		  step_count++;
		  srd.accel_count++;
		  new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
		  rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
		  // Check if we should start deceleration.
		  if(step_count >= srd.decel_start) {
			  srd.accel_count = srd.decel_val;
			  srd.run_state = DECEL;
		  }
		  // Check if we have hit max speed.
		  else if(new_step_delay <= srd.min_delay) {
			  last_accel_delay = new_step_delay;
			  new_step_delay = srd.min_delay;
			  rest = 0;
			  srd.run_state = RUN;
		  }
		  break;

		case RUN:
		  driver_StepCounter(srd.dir);
		  step_count++;
		  new_step_delay = srd.min_delay;
		  // Check if we should start deceleration.
		  if(step_count >= srd.decel_start) {
			  srd.accel_count = srd.decel_val;
			  // Start deceleration with same delay as accel ended with.
			  new_step_delay = last_accel_delay;
			  srd.run_state = DECEL;
		  }
		  break;

		case DECEL:
		  driver_StepCounter(srd.dir);
		  step_count++;
		  srd.accel_count++;
		  new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
		  rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
		  // Check if we at last step
		  if(srd.accel_count >= 0){
			  srd.run_state = STOP;
		  }
		  break;
	}
	if(new_step_delay>0xffff)
		new_step_delay = 0xffff;
	srd.step_delay = new_step_delay;
}


#endif
