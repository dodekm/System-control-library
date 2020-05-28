
/**
 * @file H_bridge.h
 * @author Martin Dodek
 * @brief
 */

#ifndef INC_H_BRIDGE_H_
#define INC_H_BRIDGE_H_

#include "io_interface.h"

typedef enum {
	H_bridge_mode_free_run = 0, H_bridge_mode_forward = 1, H_bridge_mode_backward = -1, H_bridge_mode_brake = -2,
} io_H_bridge_mode_t;

typedef struct H_bridge_generic {
	return_code (*set_mode_fcn)(struct H_bridge_generic*, io_H_bridge_mode_t);
	return_code (*set_duty_fcn)(struct H_bridge_generic*, real_t);
} io_H_bridge_generic_t;

typedef struct io_H_bridge_L298_single_channel {
	io_H_bridge_generic_t;
	io_PWM_output_generic_t* input_EN;
	io_digital_output_generic_t* input_1;
	io_digital_output_generic_t* input_2;
} io_H_bridge_L298_single_channel_t;

typedef struct io_H_bridge_L298_IC {
	io_H_bridge_L298_single_channel_t channel_A;
	io_H_bridge_L298_single_channel_t channel_B;
} io_H_bridge_L298_IC_t;

#ifdef __cplusplus
extern "C" {
#endif

static inline return_code io_H_bridge_generic_set_mode(io_H_bridge_generic_t* H_bridge, io_H_bridge_mode_t mode) {
	return H_bridge->set_mode_fcn(H_bridge, mode);
}
static inline return_code io_H_bridge_generic_brake(io_H_bridge_generic_t* H_bridge) {
	return io_H_bridge_generic_set_mode(H_bridge, H_bridge_mode_brake);
}
static inline return_code io_H_bridge_generic_free_run(io_H_bridge_generic_t* H_bridge) {
	return io_H_bridge_generic_set_mode(H_bridge, H_bridge_mode_free_run);
}
return_code io_H_bridge_generic_set_output_real(io_H_bridge_generic_t*, real_t);
return_code io_H_bridge_L298_channel_init(io_H_bridge_L298_single_channel_t*, io_PWM_output_generic_t*, io_digital_output_generic_t*, io_digital_output_generic_t*);

#ifdef __cplusplus
}
#endif

#endif /* INC_H_BRIDGE_H_ */
