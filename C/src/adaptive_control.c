
#include "adaptive_control.h"

return_code adaptive_control_init(adaptive_control_T* adaptive_control, size_t na, size_t nb, real_t Ts, real_t lambda, polynom_T* P, signal_sampled_T* ident_signal) {
	return_code returnval = return_OK;

#ifdef ASSERT_NULLPTR
	if (adaptive_control == NULL)
		return return_NULLPTR;
#endif

	returnval = vector_assert((vector_generic_T*) P);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert((vector_generic_T*) ident_signal);
	if (returnval != return_OK)
		return returnval;

	adaptive_control->enable_adaptation = bool_true;
	adaptive_control->identified = bool_false;
	adaptive_control->Ts = Ts;
	adaptive_control->P = P;
	adaptive_control->ident_signal = ident_signal;

	returnval = vector_type_init(&adaptive_control->B_ident, real_t, nb + 1, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&adaptive_control->A_ident, real_t, na + 1, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&adaptive_control->h, real_t, nb + na, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = system_ident_recursive_least_squares_init(&adaptive_control->recursive_least_squares, nb + na, lambda, 1e10);
	if (returnval != return_OK)
		return returnval;
	returnval = discrete_RST_regulator_init(&adaptive_control->regulator, nb, na - 1, 0, NULL, NULL, NULL, NULL, NULL, NULL);
	if (returnval != return_OK)
		return returnval;

	return return_OK;
}

return_code adaptive_control_deinit(adaptive_control_T* adaptive_control) {
	return_code returnval = return_OK;
	returnval = system_ident_recursive_least_squares_deinit(&adaptive_control->recursive_least_squares);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&adaptive_control->A_ident);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&adaptive_control->B_ident);
	if (returnval != return_OK)
		return returnval;
	returnval = system_deinit((system_generic_T*) &adaptive_control->regulator);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

return_code adaptive_control_iterate(adaptive_control_T* adaptive_control, real_t yk_1, real_t* u_k1, bool_t adaptation_condition) {
	return_code returnval = return_OK;

	adaptive_control->signals.y = yk_1;

	if (!adaptive_control->identified) {
		returnval = system_ident_recursive_least_squares_estimate_discrete_transfer_function(&adaptive_control->recursive_least_squares, &adaptive_control->h, adaptive_control->signals.y, &adaptive_control->B_ident, &adaptive_control->A_ident);
		if (returnval != return_OK)
			return returnval;
		returnval = signal_sampled_read_value(adaptive_control->ident_signal, adaptive_control->signals.tick*adaptive_control->Ts, &adaptive_control->signals.u);
		if (returnval == return_INDEX_OUT_OF_RANGE) {
			returnval = control_synthesis_RST_poleplace(&adaptive_control->A_ident, &adaptive_control->B_ident, adaptive_control->P, &adaptive_control->regulator.R_coeffs, &adaptive_control->regulator.S_coeffs, &adaptive_control->regulator.T_coeffs);
			if (returnval != return_OK)
				return returnval;
			discrete_RST_regulator_states_set_all(&adaptive_control->regulator, adaptive_control->signals.u, adaptive_control->signals.y, 0);
			adaptive_control->signals.tick = 0;
			adaptive_control->identified = bool_true;
		}
	} else {
		system_input_port_real_value((system_generic_T*)&adaptive_control->regulator,1) = adaptive_control->signals.y;
		system_input_port_real_value((system_generic_T*)&adaptive_control->regulator,0) = adaptive_control->signals.w;

		if (adaptive_control->enable_adaptation && adaptation_condition) {
			returnval = system_ident_recursive_least_squares_estimate_discrete_transfer_function(&adaptive_control->recursive_least_squares, &adaptive_control->h, adaptive_control->signals.y, &adaptive_control->B_ident, &adaptive_control->A_ident);
			if (returnval != return_OK)
				return returnval;
			returnval = control_synthesis_RST_poleplace(&adaptive_control->A_ident, &adaptive_control->B_ident, adaptive_control->P, &adaptive_control->regulator.R_coeffs, &adaptive_control->regulator.S_coeffs, &adaptive_control->regulator.T_coeffs);
			if (returnval != return_OK)
				return returnval;
		}
		discrete_system_generic_step_linked((discrete_system_generic_T*) &adaptive_control->regulator);
		adaptive_control->signals.u = system_output_port_real_value((system_generic_T* )&adaptive_control->regulator, 0);
	}
	system_ident_transfer_function_vector_h_iterate(&adaptive_control->h, adaptive_control->signals.u, adaptive_control->signals.y, vector_length(&adaptive_control->B_ident) - 1, vector_length(&adaptive_control->A_ident) - 1);
	adaptive_control->signals.tick ++;

	if (u_k1 != NULL)
		*u_k1 = adaptive_control->signals.u;

	return return_OK;
}
