
#include "H_bridge.h"

static return_code io_H_bridge_L298_channel_set_mode(io_H_bridge_L298_single_channel_t* H_bridge, io_H_bridge_mode_t mode) {

	switch (mode) {
	case H_bridge_mode_free_run: {
		io_PWM_output_write_real(H_bridge->input_EN, 0);
		io_digital_output_write(H_bridge->input_1, 0);
		io_digital_output_write(H_bridge->input_2, 0);
		break;
	}
	case H_bridge_mode_forward: {
		io_digital_output_write(H_bridge->input_1, 0);
		io_digital_output_write(H_bridge->input_2, 1);
		break;
	}
	case H_bridge_mode_backward: {
		io_digital_output_write(H_bridge->input_1, 1);
		io_digital_output_write(H_bridge->input_2, 0);
		break;
	}
	case H_bridge_mode_brake: {
		io_PWM_output_write_real(H_bridge->input_EN, 1);
		io_digital_output_write(H_bridge->input_1, 0);
		io_digital_output_write(H_bridge->input_2, 0);
		break;
	}
	default:
		return return_ERROR;
	}
	return return_OK;

}
static return_code io_H_bridge_L298_channel_set_duty(io_H_bridge_L298_single_channel_t* H_bridge, real_t duty) {
	return io_PWM_output_write_real(H_bridge->input_EN, duty);
}

return_code io_H_bridge_generic_set_output_real(io_H_bridge_generic_t* H_bridge, real_t speed) {

	return_code returnval = return_OK;
	returnval = H_bridge->set_mode_fcn(H_bridge, (io_H_bridge_mode_t) SIGN(speed));
	if (returnval != return_OK)
		return returnval;
	return H_bridge->set_duty_fcn(H_bridge, fabs(speed));
}

return_code io_H_bridge_L298_channel_init(io_H_bridge_L298_single_channel_t* H_bridge, io_PWM_output_generic_t* input_EN, io_digital_output_generic_t* input_1, io_digital_output_generic_t* input_2) {

#ifdef ASSERT_NULLPTR
	if (H_bridge == NULL)
		return return_NULLPTR;
	if (input_EN == NULL || input_1 == NULL || input_2 == NULL)
		return return_NULLPTR;
#endif
	H_bridge->set_mode_fcn = (return_code (*)(struct H_bridge_generic*, io_H_bridge_mode_t)) io_H_bridge_L298_channel_set_mode;
	H_bridge->set_duty_fcn = (return_code (*)(struct H_bridge_generic*, real_t)) io_H_bridge_L298_channel_set_duty;
	H_bridge->input_EN = input_EN;
	H_bridge->input_1 = input_1;
	H_bridge->input_2 = input_2;
	return return_OK;
}
