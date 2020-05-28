/**
 * @file examples.h
 * @brief
 *
 *
 */

#ifndef INC_EXAMPLES_H_
#define INC_EXAMPLES_H_

#include "discrete_systems.h"
#include "continuous_systems.h"
#include "static_systems.h"

typedef struct {
	real_t temp;
	real_t temp_amb;
} temp_control_signals_t;

typedef struct {
	real_t omega;
	real_t theta;
} motor_control_signals_t;

typedef struct continuous_system_DC_motor {
	continuous_MIMO_system_generic_T;
	real_t dX_data[3];
	real_t X_data[3];
	real_t Cu;
	real_t Te;
	real_t R;
	real_t J;
	real_t B;
	real_t input[2];
	real_t output[3];
} continuous_system_DC_motor_T;

typedef struct continuous_subystem_DC_motor {
	system_generic_T;
	system_generic_T direct_feed_subsystem;
	system_generic_T speed_subsystem;
	system_generic_T position_subsystem;
	continuous_transfer_function_first_order_T TF_electrical;
	continuous_transfer_function_first_order_T TF_mechanical;
	continuous_integrator_T integrator;
	static_system_sub_T sub_U;
	static_system_sub_T sub_M;
	static_system_gain_T gain_Cu_direct;
	static_system_gain_T gain_Cu_back;
} continuous_subsystem_DC_motor_T;

#ifdef __cplusplus
extern "C" {
#endif

return_code continuous_system_DC_motor_init(continuous_system_DC_motor_T* model, real_t Cu, real_t Te, real_t R, real_t J, real_t B);
return_code continuous_subsystem_DC_motor_init(continuous_subsystem_DC_motor_T* model, real_t Cu, real_t Te, real_t R, real_t J, real_t B) ;

typedef return_code (*continuous_nonlinear_system_derrivatives_fcn_T)(real_t[],const real_t[],const real_t[], real_t, real_t);
typedef return_code (*continuous_custom_system_output_fcn_T)(const real_t[],const real_t[], real_t, real_t*);

return_code continuous_system_pendulum_derrivatives_fcn(real_t[],const real_t[],const real_t[], real_t, real_t);
return_code continuous_system_pendulum_output_fcn(const real_t[],const real_t[], real_t , real_t*);

#ifdef __cplusplus
}
#endif



#endif /* INC_EXAMPLES_H_ */
