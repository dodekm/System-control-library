/**
 * @file stepper_motor.h
 */

#ifndef INC_STEPPER_MOTOR_H_
#define INC_STEPPER_MOTOR_H_

#include "io_interface.h"
#include "H_bridge.h"

/** @addtogroup stepper Stepper motor
 * @brief
 *
 * # Implemented functionality
 *
 * @{
 */


#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 */
typedef enum {
	stepper_motor_hold = 0,
	stepper_motor_CW = 1,
	stepper_motor_CCW = -1,
	stepper_motor_release = 2
} stepper_motor_direction;

/**
 *
 */
typedef enum {
	stepper_motor_stepping_sequence_full_step_one_phase_active = 1, stepper_motor_stepping_sequence_full_step_two_phase_active, stepper_motor_stepping_sequence_half_step, stepper_motor_stepping_sequence_quarter_step, stepper_motor_stepping_sequence_eight_step,
} stepper_motor_stepping_sequence_t;

/**
 *
 */
typedef struct stepper_motor_command
{
	real_t theta;
	real_t omega;
}stepper_motor_command_t;

/**
 *
 */
struct stepper_motor_generic;
typedef return_code (*stepper_motor_step_fcn)(struct stepper_motor_generic*, stepper_motor_direction);
typedef struct stepper_motor_generic {
	long steps_cnt;
	int rotor_phase;
	uint n_microsteps;
	const real_t* microsteps_sequence_ptr;
	stepper_motor_step_fcn step_fcn;
	real_t phi_per_step;
	real_t moment_gain;
} stepper_motor_generic_t;

/**
 *
 * @param stepper_motor
 * @param direction
 * @return
 */
static inline return_code stepper_motor_generic_step(stepper_motor_generic_t* stepper_motor, stepper_motor_direction direction) {
	return stepper_motor->step_fcn(stepper_motor, direction);
}

/**
 *
 * @param stepper_motor
 * @return
 */
static inline real_t stepper_motor_generic_get_angle(stepper_motor_generic_t* stepper_motor) {
	return (real_t) (stepper_motor->steps_cnt) / (real_t) (stepper_motor->n_microsteps / 4) * stepper_motor->phi_per_step;
}

/**
 *
 * @param stepper_motor
 * @param omega
 * @return
 */
static inline real_t stepper_motor_generic_get_delay(stepper_motor_generic_t* stepper_motor,real_t omega) {
	return  stepper_motor->phi_per_step/omega/(real_t)stepper_motor->n_microsteps;
}
/**
 *
 * @param stepper_motor
 */
static inline long stepper_motor_generic_get_position_whole(stepper_motor_generic_t* stepper_motor) {
	return stepper_motor->steps_cnt / (long) (stepper_motor->n_microsteps / 4);
}
/**
 *
 * @param stepper_motor
 */
static inline long stepper_motor_generic_get_position_microsteps(stepper_motor_generic_t* stepper_motor) {
	return stepper_motor->steps_cnt;
}

return_code stepper_motor_generic_set_moment_gain(stepper_motor_generic_t*, real_t);
return_code stepper_motor_generic_step_to_position_whole(stepper_motor_generic_t*, long);
return_code stepper_motor_generic_step_to_position_microsteps(stepper_motor_generic_t*, long);
return_code stepper_motor_generic_step_to_position_angle(stepper_motor_generic_t*, real_t);
return_code stepper_motor_generic_set_stepping_sequence(stepper_motor_generic_t*, stepper_motor_stepping_sequence_t);

/**
 *
 */
typedef struct stepper_motor_bipolar {
	stepper_motor_generic_t;
	io_H_bridge_generic_t* H_bridge_channel_A_ptr;
	io_H_bridge_generic_t* H_bridge_channel_B_ptr;
} stepper_motor_bipolar_t;

return_code stepper_motor_bipolar_init(stepper_motor_bipolar_t*, io_H_bridge_generic_t*, io_H_bridge_generic_t*, stepper_motor_stepping_sequence_t, real_t);

/**
 *
 */
typedef struct stepper_motor_unipolar {
	stepper_motor_generic_t;
	io_PWM_output_generic_t* A1_pwm_out;
	io_PWM_output_generic_t* A2_pwm_out;
	io_PWM_output_generic_t* B1_pwm_out;
	io_PWM_output_generic_t* B2_pwm_out;
} stepper_motor_unipolar_t;

return_code stepper_motor_unipolar_init(stepper_motor_unipolar_t*, io_PWM_output_generic_t*, io_PWM_output_generic_t*, io_PWM_output_generic_t*, io_PWM_output_generic_t*, stepper_motor_stepping_sequence_t, real_t);


#ifdef __cplusplus
}
#endif

/** @} */

#endif /* INC_STEPPER_MOTOR_H_ */
