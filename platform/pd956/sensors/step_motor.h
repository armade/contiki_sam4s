#ifndef step_motor_H_
#define step_motor_H_

// Speed ramp states
typedef enum{
	STOP,
	ACCEL,
	DECEL,
	RUN,
}step_state_t;

/*---------------------------------------------------------------------------*/
/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
typedef struct {
	//! What part of the speed ramp we are in.
	step_state_t run_state;
	//! Direction stepper motor should move.
	unsigned char dir;
	//! Peroid of next timer delay. At start this value set the accelerating rate.
	int step_delay;
	//! What step_pos to start decelerating
	int decel_start;
	//! Sets deceleration rate.
	int decel_val;
	//! Minimum time delay (max speed)
	int min_delay;
	//! Counter used when accelerating/decelerating to calculate step_delay.
	int accel_count;
} speedRampData;
/*---------------------------------------------------------------------------*/
// USER DEFINED VARIABELS - please modify
/*! \Brief Frequency of timer in [Hz].
 * Modify this according to frequency used.
 */
#define T1_FREQ 937500

//! Number of (full)steps per round on stepper motor in use.
#define FSPR 4096 //64 steps + gear (64:1) 64*64=4096

#define SPR (FSPR*2) // only half step
/*---------------------------------------------------------------------------*/

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000

/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor step_sensor;
/*---------------------------------------------------------------------------*/
#endif
