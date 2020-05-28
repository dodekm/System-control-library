#include "systems.h"

/**
 * Transfer function initialization.
 * Initializes numerator and denominator coefficients vectors.
 * @param transfer_function transfer function to initialize
 * @param numerator_order order of numerator
 * @param denominator_order order of denominator
 * @param numerator_coeffs pointer to numerator coefficients data, if NULL - function performs dynamic allocation
 * @param denominator_coeffs pointer to denominator coefficients data, if NULL - function performs dynamic allocation
 * @return system control library error code
 */

return_code transfer_function_init(transfer_function_T* transfer_function, size_t numerator_order, size_t denominator_order, real_t* numerator_coeffs, real_t* denominator_coeffs) {

	return_code returnval = return_OK;

	returnval = vector_type_init(&transfer_function->numerator_coeffs, real_t, numerator_order + 1, numerator_coeffs);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&transfer_function->denominator_coeffs, real_t, denominator_order + 1, denominator_coeffs);
	if (returnval != return_OK)
		return returnval;
	return return_OK;

}

/**
 * Transfer function structure de-initialization.
 * De-initializes transfer function numerator and denominator coefficients vectors.
 * @param transfer_function transfer function structure to de-initialize
 * @return system control library error code
 */
return_code transfer_function_deinit(transfer_function_T* transfer_function) {

	return_code returnval = return_OK;
	returnval = vector_generic_deinit(&transfer_function->numerator_coeffs);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&transfer_function->denominator_coeffs);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * Transfer function structure verification function.
 * Asserts numerator and denominator coefficients vectors
 * @param transfer_function transfer function structure to assert
 * @return system control library error code
 */
return_code transfer_function_assert(const transfer_function_T* transfer_function) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(&transfer_function->numerator_coeffs);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(&transfer_function->denominator_coeffs);
	if (returnval != return_OK)
		return returnval;
#endif

#ifdef ASSERT_DIMENSIONS
	if (!vector_is_type(&transfer_function->numerator_coeffs, real_t) || !vector_is_type(&transfer_function->denominator_coeffs, real_t))
		return return_WRONG_DIMENSIONS;
#endif
	return return_OK;
}

/**
 * Sets transfer function numerator and denominator coefficients.
 * Loads data to numerator/denominator coefficients vectors
 * @see vector_generic_load
 * @param transfer_function discrete transfer function structure pointer
 * @param numerator_coeffs  numerator coefficients data pointer, if NULL - does not load
 * @param denominator_coeffs denominator coefficients data pointer, if NULL - does not load
 * @return system control library error code
 */

return_code transfer_function_coeffs_set(transfer_function_T* transfer_function, const real_t* numerator_coeffs, const real_t* denominator_coeffs) {

	return_code returnval = return_OK;
	if (numerator_coeffs != NULL) {
		returnval = vector_type_load(&transfer_function->numerator_coeffs, numerator_coeffs);
		if (returnval != return_OK)
			return returnval;

	}
	if (denominator_coeffs != NULL) {
		returnval = vector_type_load(&transfer_function->denominator_coeffs, denominator_coeffs);
		if (returnval != return_OK)
			return returnval;
	}
	return return_OK;
}

/**
 * State space initialization.
 * Initializes A matrix and B,C vectors according to system order
 * @see matrix_init
 * @see vector_type_init
 * @param state_space state space structure pointer
 * @param order order of system
 * @param A pointer to A matrix data, if NULL - function performs dynamic allocation
 * @param B pointer to B vector data, if NULL - function performs dynamic allocation
 * @param C pointer to C vector data, if NULL - function performs dynamic allocation
 * @return system control library error code
 */

return_code state_space_init(state_space_T* state_space, size_t order, real_t* A, real_t* B, real_t* C) {
	return_code returnval = return_OK;
	returnval = matrix_init(&state_space->A, order, order, A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&state_space->B,real_t ,order, B);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&state_space->C,real_t,order, C);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * State space structure de-initialization.
 * De-initializes state space A matrix and B, C vectors
 * @param state_space state space structure to de-initialize
 * @return system control library error code
 */
return_code state_space_deinit(state_space_T* state_space) {
	return_code returnval = return_OK;
	returnval = matrix_deinit(&state_space->A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&state_space->B);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&state_space->C);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * Loads data to state space coefficients matrix  A and B,C vectors.
 * @see matrix_load
 * @see vector_generic_load
 * @param state_space state space structure pointer
 * @param A pointer to A matrix data, if NULL - function does not load
 * @param B pointer to B vector data, if NULL - function does not load
 * @param C pointer to C vector data, if NULL - function does not load
 * @return system control library error code
 */
return_code state_space_coeffs_set(state_space_T* state_space, const real_t* A, const real_t* B, const real_t* C) {

	return_code returnval = return_OK;
	if (A != NULL) {
		returnval = matrix_load(&state_space->A, A);
		if (returnval != return_OK)
			return returnval;
	}
	if (B != NULL) {
		returnval = vector_generic_load(&state_space->B, B);
		if (returnval != return_OK)
			return returnval;
	}
	if (C != NULL) {
		returnval = vector_generic_load(&state_space->C, C);
		if (returnval != return_OK)
			return returnval;
	}
	return return_OK;

}

/**
 * State space structure verification function.
 * Asserts state space coefficients matrix A and B,C vectors
 * @param state_space state space structure to assert
 * @return system control library error code
 */
return_code state_space_assert(const state_space_T* state_space) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = matrix_assert(&state_space->A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(&state_space->B);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(&state_space->C);
	if (returnval != return_OK)
		return returnval;
#endif

#ifdef ASSERT_DIMENSIONS
	if (!matrix_is_square(&state_space->A))
		return return_WRONG_DIMENSIONS;
	if (state_space->B.length != state_space->A.n_rows || state_space->C.length != state_space->A.n_rows)
		return return_WRONG_DIMENSIONS;
#endif

	return return_OK;
}

/**
 * Zeros-poles-gain initialization.
 * Initializes zeros and poles complex vectors
 * @param zpk
 * @param n_zeros number of zeros
 * @param zeros_ptr pointer to zeros data, if NULL - function performs dynamic allocation
 * @param n_poles number of poles
 * @param poles_ptr pointer to poles data, if NULL - function performs dynamic allocation
 * @param gain gain value
 * @return system control library error code
 */
return_code zeros_poles_gain_init(zeros_poles_gain_T* zpk, size_t n_zeros, complex_t* zeros_ptr, size_t n_poles, complex_t* poles_ptr,real_t gain) {

	return_code returnval = return_OK;
	returnval = vector_type_init(&zpk->zeros, complex_t, n_zeros, zeros_ptr);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_type_init(&zpk->poles, complex_t, n_poles, poles_ptr);
		if (returnval != return_OK)
			return returnval;
	zpk->gain=gain;
	return return_OK;

}

/**
 * Zeros-poles-gain structure de-initialization.
 * De-initializes zeros and poles vectors.
 * @param zpk zeros-poles-gain structure to de-initialize
 * @return system control library error code
 */

return_code zeros_poles_gain_deinit(zeros_poles_gain_T* zpk) {
	return_code returnval = return_OK;
	returnval = vector_generic_deinit(&zpk->zeros);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&zpk->poles);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * Zeros-poles-gain structure verification function.
 * Asserts zeros and poles vectors
 * @param zpk zeros-poles-gain structure to assert
 * @return system control library error code
 */
return_code zeros_poles_gain_assert(const zeros_poles_gain_T* zpk) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = vector_assert(&zpk->zeros);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(&zpk->poles);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS

	if (!vector_is_type(&zpk->zeros, complex_t) || !vector_is_type(&zpk->poles, complex_t))
		return return_WRONG_DIMENSIONS;
#endif

	return return_OK;
}

static return_code system_assign_port_pointers(system_generic_T* system, real_t* input_port_ptr, real_t* output_port_ptr) {

	signal_realtime_T signal_struct = { 0 };
	signal_struct.owner = system;

	if (input_port_ptr != NULL) {
		for (uint i = 0; i < vector_length(&system->input_port); i++) {
			signal_struct.ptr = input_port_ptr + i;
			vector_type_at(&system->input_port,signal_realtime_T,i) = signal_struct;
		}
	}
	if (output_port_ptr != NULL) {
		for (uint i = 0; i < vector_length(&system->output_port); i++) {
			signal_struct.ptr = output_port_ptr + i;
			vector_type_at(&system->output_port,signal_realtime_T,i) = signal_struct;
		}
	}
	return return_OK;
}

/**
 * Initializes generic system IO interface and sets de-initialization function.
 * Function does assign input/output ports pointers of system according to port pointers as it's own
 * @param system generic system pointer
 * @param system_type type of system
 * @param time_domain time domain of system - discrete or continuous
 * @param input_port_width number of system input signals
 * @param input_port_ptr pointer to system's input port linear array - can be NULL
 * @param output_port_width number of system output signals
 * @param output_port_ptr pointer to system's output port linear array - can be NULL
 * @param deinit_fcn custom system deinitialization function pointer - can be NULL
 * @return system control library error code
 */

return_code system_interface_init(system_generic_T* system, system_type system_type, system_time_domain time_domain, size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr, system_deinit_fcn deinit_fcn) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (system == NULL)
		return return_NULLPTR;
#endif

	if (input_port_width) {
		returnval = vector_type_init(&system->input_port, signal_realtime_T, input_port_width, NULL);
		if (returnval != return_OK)
			return returnval;
	}
	if (output_port_width) {
		returnval = vector_type_init(&system->output_port, signal_realtime_T, output_port_width, NULL);
		if (returnval != return_OK)
			return returnval;
	}
	system->type = system_type;
	system->time_domain = time_domain;
	system->deinit_fcn = deinit_fcn;

	return system_assign_port_pointers(system, input_port_ptr, output_port_ptr);
}

/**
 * Deinitializes generic system interface, clears subsystems list, deinitializes IO ports vectors.
 * @param system pointer to generic system
 * @return system control library error code
 */

return_code system_interface_deinit(system_generic_T* system) {

#ifdef ASSERT_NULLPTR
	if (system == NULL)
		return return_NULLPTR;
#endif
	return_code returnval = return_OK;
	returnval = vector_generic_deinit(&system->input_port);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_generic_deinit(&system->output_port);
	if (returnval != return_OK)
		return returnval;
	returnval = list_clear(&system->execution_list);
	if (returnval != return_OK)
		return returnval;
	system->type = system_type_uninitialized;
	system->time_domain = system_time_domain_uninitialized;
	system->deinit_fcn = NULL;

	return return_OK;
}
/**
 * Deinitializes generic system using deinit function pointer.
 * If system is subsystem, calls function recursive
 * @param system_interface pointer to generic system
 * @return system control library error code
 */

return_code system_deinit(system_generic_T* system_interface) {

	return_code returnval = return_OK;
	if (system_interface->type == system_type_subsystem) {
		list_item_t* item = system_interface->execution_list.front;
		while (item != NULL) {
			system_generic_T* system = (system_generic_T*) item->object;
			returnval = system_deinit(system);
			if (returnval != return_OK)
				return returnval;
			item = item->next;
		}
	}
	if (system_interface->deinit_fcn != NULL) {
		returnval = system_interface->deinit_fcn(system_interface);
		if (returnval != return_OK)
			return returnval;
	}
	return system_interface_deinit(system_interface);
}

/**
 * Creates serially connected subsystem from input subsystem array.
 * Connects according input and output ports signals of systems.
 * Fills execution list of subsystem.
 * @param systems_list array of system we want to connect serially
 * @param n_systems systems count - number of systems
 * @param input_port_numbers array of input port numbers in case of MIMO system
 * @param output_port_numbers array of output port numbers in case of MIMO system
 * @param subsystem target subsystem structure pointer
 * @return system control library error code
 */

return_code systems_connect_serial(system_generic_T* systems_list[], size_t n_systems, uint input_port_numbers[], uint output_port_numbers[], system_generic_T* subsystem) {

#ifdef ASSERT_NULLPTR
	if (systems_list == NULL)
		return return_NULLPTR;
	if (input_port_numbers == NULL || output_port_numbers == NULL)
		return return_NULLPTR;
#endif
	return_code returnval = return_OK;
	returnval = system_interface_init(subsystem, system_type_subsystem, system_time_domain_uninitialized, 1, NULL, 1, NULL, NULL);
	if (returnval != return_OK)
		return returnval;
	for (uint i = 0; i < n_systems - 1; i++) {
#ifdef ASSERT_NULLPTR
		if (systems_list[i] == NULL || systems_list[i + 1] == NULL)
			return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
		if ( system_input_port_size(systems_list[i+1]) <= input_port_numbers[i + 1])
			return return_WRONG_DIMENSIONS;
		if ( system_output_port_size(systems_list[i]) <= output_port_numbers[i])
			return return_WRONG_DIMENSIONS;
#endif

		system_input_port_struct(system_input_port_struct(systems_list[i+1],input_port_numbers[i+1]).owner,input_port_numbers[i+1]) = system_output_port_struct(systems_list[i], output_port_numbers[i]);
		system_input_port_struct(systems_list[i+1],input_port_numbers[i+1]) = system_output_port_struct(systems_list[i], output_port_numbers[i]);
		subsystem->time_domain |= systems_list[i]->time_domain;
		returnval = list_push_back(&subsystem->execution_list, systems_list[i]);
		if (returnval != return_OK)
			return returnval;
	}
	returnval = list_push_back(&subsystem->execution_list, systems_list[n_systems - 1]);
	if (returnval != return_OK)
		return returnval;
	system_input_port_struct(subsystem, 0) = system_input_port_struct(systems_list[0], input_port_numbers[0]);
	system_output_port_struct(subsystem, 0) = system_output_port_struct(systems_list[n_systems - 1], output_port_numbers[n_systems - 1]);

	return return_OK;
}

/**
 * Creates paralelly connected subsystem from input subsystem array routing to one multiport static operator.
 * Connects according input and output ports signals of systems.
 * Fills execution list of subsystem.
 * @param systems_from array of source systems we connect paralelly
 * @param n_systems systems count - number of systems
 * @param system_to static port operator system pointer
 * @param subsystem target subsystem structure pointer
 * @return system control library error code
 */

return_code systems_connect_paralel(system_generic_T* systems_from[], size_t n_systems, system_generic_T* system_to, system_generic_T* subsystem) {
#ifdef ASSERT_NULLPTR
	if (system_to == NULL || systems_from == NULL)
		return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (system_input_port_size(system_to) != n_systems || system_output_port_size(system_to) != 1)
		return return_WRONG_DIMENSIONS;
#endif
	return_code returnval = return_OK;
	bool_t common_input = bool_true;
	signal_realtime_T common_input_signal = system_input_port_struct(systems_from[0], 0);
	for (uint i = 0; i < n_systems; i++) {
#ifdef ASSERT_NULLPTR
		if (systems_from[i] == NULL)
			return return_NULLPTR;
#endif
		if (system_input_port_struct(systems_from[i], 0).owner != common_input_signal.owner || system_input_port_struct(systems_from[i], 0).ptr != common_input_signal.ptr) {
			common_input = bool_false;
			break;
		}
	}
	returnval = system_interface_init(subsystem, system_type_subsystem, system_to->time_domain, common_input ? 1 : n_systems, NULL, 1, NULL, NULL);
	if (returnval != return_OK)
		return returnval;

	for (uint i = 0; i < n_systems; i++) {
#ifdef ASSERT_DIMENSIONS
		if (system_input_port_size(systems_from[i]) != 1 || system_output_port_size(systems_from[i]) != 1)
			return return_WRONG_DIMENSIONS;
#endif
		system_input_port_struct(system_input_port_struct(system_to,i).owner,i) = system_output_port_struct(systems_from[i], 0);
		system_input_port_struct(system_to,i) = system_output_port_struct(systems_from[i], 0);
		subsystem->time_domain |= systems_from[i]->time_domain;
		returnval = list_push_back(&subsystem->execution_list, systems_from[i]);
		if (returnval != return_OK)
			return returnval;
		if (common_input == bool_false)
			system_input_port_struct(subsystem, i) = system_input_port_struct(systems_from[i], 0);
	}
	returnval = list_push_back(&subsystem->execution_list, system_to);
	if (returnval != return_OK)
		return returnval;
	if (common_input == bool_true)
		system_input_port_struct(subsystem, 0) = common_input_signal;
	system_output_port_struct(subsystem, 0) = system_output_port_struct(system_to, 0);
	return return_OK;
}

/**
 * Creates system feedback loop from using static operator (sum, difference).
 * Connects according input and output ports signals of systems.
 * Fills execution list of subsystem.
 * @param operator_system pointer to static port operator system
 * @param feedDirect_system pointer to direct feed dynamic system (e.g. open loop)
 * @param feedBack_system  pointer to  feed back dynamic system (e.g. compensator or unit gain)
 * @param subsystem target subsystem structure pointer
 * @return system control library error code
 */
return_code systems_connect_feedback(system_generic_T* operator_system, system_generic_T* feedDirect_system, system_generic_T* feedBack_system, system_generic_T* subsystem) {

#ifdef ASSERT_NULLPTR
	if (operator_system == NULL || feedDirect_system == NULL || feedBack_system == NULL)
		return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (system_input_port_size(operator_system) != 2)
		return return_WRONG_DIMENSIONS;
	if (system_output_port_size(operator_system) != 1)
		return return_WRONG_DIMENSIONS;
	if (system_input_port_size(feedDirect_system) != 1)
		return return_WRONG_DIMENSIONS;
	if (system_output_port_size(feedDirect_system) != 1)
		return return_WRONG_DIMENSIONS;
	if (system_input_port_size(feedBack_system) != 1)
		return return_WRONG_DIMENSIONS;
	if (system_output_port_size(feedBack_system) != 1)
		return return_WRONG_DIMENSIONS;
#endif

	system_input_port_struct(system_input_port_struct(feedDirect_system,0).owner,0) = system_output_port_struct(operator_system, 0);
	system_input_port_struct(feedDirect_system,0) = system_output_port_struct(operator_system, 0);
	system_input_port_struct(system_input_port_struct(operator_system,1).owner,1) = system_output_port_struct(feedBack_system, 0);
	system_input_port_struct(operator_system,1) = system_output_port_struct(feedBack_system, 0);
	system_input_port_struct(system_input_port_struct(feedBack_system,0).owner,0) = system_output_port_struct(feedDirect_system, 0);
	system_input_port_struct(feedBack_system,0) = system_output_port_struct(feedDirect_system, 0);

	return_code returnval = return_OK;
	returnval = system_interface_init(subsystem, system_type_subsystem, feedDirect_system->time_domain | feedBack_system->time_domain | operator_system->time_domain, 1, NULL, 1, NULL, NULL);
	if (returnval != return_OK)
		return returnval;

	system_input_port_struct(subsystem, 0) = system_input_port_struct(operator_system, 0);
	system_output_port_struct(subsystem, 0) = system_output_port_struct(feedDirect_system, 0);
	returnval = list_push_back(&subsystem->execution_list, feedBack_system);
	if (returnval != return_OK)
		return returnval;
	returnval = list_push_back(&subsystem->execution_list, operator_system);
	if (returnval != return_OK)
		return returnval;
	returnval = list_push_back(&subsystem->execution_list, feedDirect_system);
	if (returnval != return_OK)
		return returnval;

	return return_OK;

}

