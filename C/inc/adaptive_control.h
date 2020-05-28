/**
 * @file adaptive_control.h
 * @brief
 *
 */


#ifndef INC_ADAPTIVE_CONTROL_H_
#define INC_ADAPTIVE_CONTROL_H_

#include "common_def.h"
#include "polynom.h"
#include "signal_process.h"
#include "discrete_systems.h"
#include "system_identification.h"
#include "control_synthesis.h"

typedef struct adaptive_control {
	real_t Ts;
	polynom_T* P;
	signal_sampled_T* ident_signal;
	control_loop_signals_T signals;
	bool_t enable_adaptation;
	bool_t identified;
	vector_generic_T B_ident;
	vector_generic_T A_ident;
	vector_generic_T h;
	system_ident_recursive_least_squares_T recursive_least_squares;
	discrete_RST_regulator_T regulator;
} adaptive_control_T;

#ifdef __cplusplus
extern "C" {
#endif

return_code adaptive_control_init(adaptive_control_T*, size_t, size_t, real_t, real_t, polynom_T*, signal_sampled_T*);
return_code adaptive_control_deinit(adaptive_control_T*);
return_code adaptive_control_iterate(adaptive_control_T*, real_t, real_t*, bool_t);

static inline void adaptive_control_enable_adaptation(adaptive_control_T* adaptive_control, bool_t enable_state) {
	adaptive_control->enable_adaptation = enable_state;
}


static inline void adaptive_control_set_lambda(adaptive_control_T* adaptive_control, real_t lambda) {
	system_ident_recursive_least_squares_set_lambda(&adaptive_control->recursive_least_squares, lambda);
}

static inline void adaptive_control_set_setpoint(adaptive_control_T* adaptive_control, real_t w) {
	adaptive_control->signals.w = w;
}

static inline void adaptive_control_set_saturation(adaptive_control_T* adaptive_control, real_t saturation_high, real_t saturation_low) {
	discrete_RST_regulator_set_saturation(&adaptive_control->regulator, saturation_high, saturation_low);
}

static inline real_t adaptive_control_get_time(const adaptive_control_T* adaptive_control) {
	return adaptive_control->signals.tick*adaptive_control->Ts;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_ADAPTIVE_CONTROL_H_ */
