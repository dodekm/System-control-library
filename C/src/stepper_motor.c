#include "stepper_motor.h"

static const real_t stepper_motor_full_step_one_phase_active_seq[4] = { 1.0, 0.0, -1.0, 0.0 };
static const real_t stepper_motor_full_step_two_phase_active_seq[4] = { 1.0, 1.0, -1.0, -1.0 };
static const real_t stepper_motor_half_step_seq[8] = { 1.0, 0.7071, 0.0, -0.7071, -1.0, -0.7071, 0.0, 0.7071 };
//static const real_t stepper_motor_quarter_step_seq[16] = { 1.0000, 0.9239, 0.7071, 0.3827, 0.0000, -0.3827, -0.7071, -0.9239, -1.0000, -0.9239, -0.7071, -0.3827, -0.0000, 0.3827, 0.7071, 0.9239 };
//static const real_t stepper_motor_quarter_step_seq[16] = { 1.0000, 0.8536, 0.5000, 0.1464, 0.0000, -0.1464, -0.5000, -0.8536, -1.0000, -0.8536, -0.5000, -0.1464, -0.0000, 0.1464, 0.5000, 0.8536 };
//static const real_t stepper_motor_quarter_step_seq[16] = { 1.0000, 0.9612, 0.8409, 0.6186, 0.0000, -0.6186, -0.8409, -0.9612, -1.0000, -0.9612, -0.8409, -0.6186, -0.0000, 0.6186, 0.8409, 0.9612 };
static const real_t stepper_motor_quarter_step_seq[16] = { 1.0000, 0.9804, 0.9170, 0.7865, 0.0001, -0.7865, -0.9170, -0.9804, -1.0000, -0.9804, -0.9170, -0.7865, -0.0001, 0.7865, 0.9170, 0.9804 };

//static const real_t stepper_motor_eight_step_seq[32] = { 1.0000, 0.9808, 0.9239, 0.8315, 0.7071, 0.5556, 0.3827, 0.1951, 0.0000, -0.1951, -0.3827, -0.5556, -0.7071, -0.8315, -0.9239, -0.9808, -1.0000, -0.9808, -0.9239, -0.8315, -0.7071, -0.5556, -0.3827, -0.1951, -0.0000, 0.1951, 0.3827, 0.5556, 0.7071, 0.8315, 0.9239, 0.9808 };
static const real_t stepper_motor_eight_step_seq[32] = { 1.0000, 0.9952, 0.9804, 0.9549, 0.9170, 0.8633, 0.7865, 0.6646, 0.0001, -0.6646, -0.7865, -0.8633, -0.9170, -0.9549, -0.9804, -0.9952, -1.0000, -0.9952, -0.9804, -0.9549, -0.9170, -0.8633, -0.7865, -0.6646, -0.0001, 0.6646, 0.7865, 0.8633, 0.9170, 0.9549, 0.9804, 0.9952 };

typedef struct stepper_motor_phase_voltages {
	real_t A_phase_voltage;
	real_t B_phase_voltage;
} stepper_motor_phase_voltages_t;

static stepper_motor_phase_voltages_t stepper_motor_generic_step_private(stepper_motor_generic_t* stepper_motor, stepper_motor_direction direction) {

	stepper_motor_phase_voltages_t A_B_phase_voltages = { 0 };

	uint A_phase_sequence_idx = MOD(stepper_motor->rotor_phase + direction, stepper_motor->n_microsteps);
	uint B_phase_sequence_idx = MOD(A_phase_sequence_idx - stepper_motor->n_microsteps / 4, stepper_motor->n_microsteps);

	stepper_motor->rotor_phase = A_phase_sequence_idx;
	stepper_motor->steps_cnt += direction;

	A_B_phase_voltages.A_phase_voltage = stepper_motor->microsteps_sequence_ptr[A_phase_sequence_idx] * stepper_motor->moment_gain;
	A_B_phase_voltages.B_phase_voltage = stepper_motor->microsteps_sequence_ptr[B_phase_sequence_idx] * stepper_motor->moment_gain;

	return A_B_phase_voltages;
}

static return_code stepper_motor_bipolar_step_fcn(stepper_motor_bipolar_t* stepper_motor, stepper_motor_direction direction) {

	return_code returnval = return_OK;
	if (direction == stepper_motor_CW || direction == stepper_motor_CCW || direction == stepper_motor_hold) {

		stepper_motor_phase_voltages_t phase_voltages = stepper_motor_generic_step_private((stepper_motor_generic_t*) stepper_motor, direction);
		returnval = io_H_bridge_generic_set_output_real(stepper_motor->H_bridge_channel_A_ptr, phase_voltages.A_phase_voltage);
		if (returnval != return_OK)
			return returnval;
		returnval = io_H_bridge_generic_set_output_real(stepper_motor->H_bridge_channel_B_ptr, phase_voltages.B_phase_voltage);
		if (returnval != return_OK)
			return returnval;

	} else if (direction == stepper_motor_release) {
		returnval = io_H_bridge_generic_set_mode(stepper_motor->H_bridge_channel_A_ptr, H_bridge_mode_free_run);
		if (returnval != return_OK)
			return returnval;
		returnval = io_H_bridge_generic_set_mode(stepper_motor->H_bridge_channel_B_ptr, H_bridge_mode_free_run);
		if (returnval != return_OK)
			return returnval;
	} else
		return return_ERROR;

	return return_OK;
}
return_code stepper_motor_generic_step_to_position_whole(stepper_motor_generic_t* stepper_motor, long position_whole_steps) {

	return_code returnval = stepper_motor_generic_step(stepper_motor, (stepper_motor_direction) SIGN(position_whole_steps - stepper_motor_generic_get_position_whole(stepper_motor)));
	if (returnval != return_OK)
		return returnval;
	if (position_whole_steps == stepper_motor_generic_get_position_whole(stepper_motor))
		return return_OK;
	else
		return return_BUSY;

}

return_code stepper_motor_generic_set_moment_gain(stepper_motor_generic_t* stepper_motor, real_t moment_gain) {
	if (moment_gain < 0 || moment_gain > 1)
		return return_ERROR;
	stepper_motor->moment_gain = moment_gain;
	return return_OK;
}

return_code stepper_motor_generic_step_to_position_microsteps(stepper_motor_generic_t* stepper_motor, long position_microsteps) {
	return_code returnval = stepper_motor_generic_step(stepper_motor, (stepper_motor_direction) SIGN(position_microsteps - stepper_motor_generic_get_position_microsteps(stepper_motor)));
	if (returnval != return_OK)
		return returnval;
	if (position_microsteps == stepper_motor_generic_get_position_microsteps(stepper_motor))
		return return_OK;
	else
		return return_BUSY;

}
return_code stepper_motor_generic_step_to_position_angle(stepper_motor_generic_t* stepper_motor, real_t angle) {
	return stepper_motor_generic_step_to_position_microsteps(stepper_motor, (long) round((angle / stepper_motor->phi_per_step / 4.0 * (real_t) (stepper_motor->n_microsteps))));
}

return_code stepper_motor_generic_set_stepping_sequence(stepper_motor_generic_t* stepper_motor, stepper_motor_stepping_sequence_t sequence) {
#ifdef ASSERT_NULLPTR
	if (stepper_motor == NULL)
		return return_NULLPTR;
#endif
	switch (sequence) {
	case stepper_motor_stepping_sequence_full_step_one_phase_active:
		stepper_motor->microsteps_sequence_ptr = stepper_motor_full_step_one_phase_active_seq;
		stepper_motor->n_microsteps = 4;
		break;
	case stepper_motor_stepping_sequence_full_step_two_phase_active:
		stepper_motor->microsteps_sequence_ptr = stepper_motor_full_step_two_phase_active_seq;
		stepper_motor->n_microsteps = 4;
		break;
	case stepper_motor_stepping_sequence_half_step:
		stepper_motor->microsteps_sequence_ptr = stepper_motor_half_step_seq;
		stepper_motor->n_microsteps = 8;
		break;
	case stepper_motor_stepping_sequence_quarter_step:
		stepper_motor->microsteps_sequence_ptr = stepper_motor_quarter_step_seq;
		stepper_motor->n_microsteps = 16;
		break;
	case stepper_motor_stepping_sequence_eight_step:
		stepper_motor->microsteps_sequence_ptr = stepper_motor_eight_step_seq;
		stepper_motor->n_microsteps = 32;
		break;
	default:
		return return_ERROR;

	}
	stepper_motor->rotor_phase = 0;
	return return_OK;

}

static void stepper_motor_generic_init(stepper_motor_generic_t* stepper_motor, stepper_motor_step_fcn step_fcn, real_t phi_per_step) {

	stepper_motor->phi_per_step = phi_per_step;
	stepper_motor->steps_cnt = 0;
	stepper_motor->rotor_phase = 0;
	stepper_motor->moment_gain = 1.0;
	stepper_motor->step_fcn = step_fcn;
}

return_code stepper_motor_bipolar_init(stepper_motor_bipolar_t* stepper_motor, io_H_bridge_generic_t* H_bridge_channel_A_ptr, io_H_bridge_generic_t* H_bridge_channel_B_ptr, stepper_motor_stepping_sequence_t sequence, real_t phi_per_step) {

#ifdef ASSERT_NULLPTR
	if (stepper_motor == NULL)
		return return_NULLPTR;
	if (H_bridge_channel_A_ptr == NULL)
		return return_NULLPTR;
	if (H_bridge_channel_B_ptr == NULL)
		return return_NULLPTR;
#endif

	stepper_motor->H_bridge_channel_A_ptr = H_bridge_channel_A_ptr;
	stepper_motor->H_bridge_channel_B_ptr = H_bridge_channel_B_ptr;

	stepper_motor_generic_init((stepper_motor_generic_t*) stepper_motor, (stepper_motor_step_fcn) stepper_motor_bipolar_step_fcn, phi_per_step);
	return_code returnval = stepper_motor_generic_set_stepping_sequence((stepper_motor_generic_t*) stepper_motor, sequence);
	if (returnval != return_OK)
		return returnval;

	return return_OK;
}

static return_code stepper_motor_unipolar_step_fcn(stepper_motor_unipolar_t* stepper_motor, stepper_motor_direction direction) {

	if (direction == stepper_motor_CW || direction == stepper_motor_CCW || direction == stepper_motor_hold) {
		stepper_motor_phase_voltages_t phase_voltages = stepper_motor_generic_step_private((stepper_motor_generic_t*) stepper_motor, direction);
		if (phase_voltages.A_phase_voltage > 0) {
			io_PWM_output_write_real(stepper_motor->A1_pwm_out, phase_voltages.A_phase_voltage);
			io_PWM_output_write_real(stepper_motor->A2_pwm_out, 0);
		} else {
			io_PWM_output_write_real(stepper_motor->A2_pwm_out, -phase_voltages.A_phase_voltage);
			io_PWM_output_write_real(stepper_motor->A1_pwm_out, 0);
		}
		if (phase_voltages.B_phase_voltage > 0) {
			io_PWM_output_write_real(stepper_motor->B1_pwm_out, phase_voltages.B_phase_voltage);
			io_PWM_output_write_real(stepper_motor->B2_pwm_out, 0);
		} else {
			io_PWM_output_write_real(stepper_motor->B2_pwm_out, -phase_voltages.B_phase_voltage);
			io_PWM_output_write_real(stepper_motor->B1_pwm_out, 0);
		}
	} else if (direction == stepper_motor_release) {
		io_PWM_output_write_real(stepper_motor->A2_pwm_out, 0);
		io_PWM_output_write_real(stepper_motor->A1_pwm_out, 0);
		io_PWM_output_write_real(stepper_motor->B2_pwm_out, 0);
		io_PWM_output_write_real(stepper_motor->B1_pwm_out, 0);
	}

	return return_OK;
}

return_code stepper_motor_unipolar_init(stepper_motor_unipolar_t* stepper_motor, io_PWM_output_generic_t* A1_pwm_out, io_PWM_output_generic_t* A2_pwm_out, io_PWM_output_generic_t* B1_pwm_out, io_PWM_output_generic_t* B2_pwm_out, stepper_motor_stepping_sequence_t sequence, real_t phi_per_step) {

#ifdef ASSERT_NULLPTR
	if (stepper_motor == NULL)
		return return_NULLPTR;
	if (A1_pwm_out == NULL || A2_pwm_out == NULL || B1_pwm_out == NULL || B2_pwm_out == NULL)
		return return_NULLPTR;
#endif

	stepper_motor->A1_pwm_out = A1_pwm_out;
	stepper_motor->A2_pwm_out = A2_pwm_out;
	stepper_motor->B1_pwm_out = B1_pwm_out;
	stepper_motor->B2_pwm_out = B2_pwm_out;

	stepper_motor_generic_init((stepper_motor_generic_t*) stepper_motor, (stepper_motor_step_fcn) stepper_motor_unipolar_step_fcn, phi_per_step);

	return_code returnval = stepper_motor_generic_set_stepping_sequence((stepper_motor_generic_t*) stepper_motor, sequence);
	if (returnval != return_OK)
		return returnval;

	return return_OK;
}
