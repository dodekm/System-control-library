#include "model.h"

extern return_code continuous_system_generic_init(continuous_system_generic_T*, size_t, real_t*, real_t*, continuous_system_derivatives_fcn, continuous_system_update_output_fcn);
extern return_code continuous_system_generic_deinit(continuous_system_generic_T*);

static return_code model_discrete_step_fcn(model_discrete_T* model, const vector_generic_T* u_vector, vector_generic_T* y_vector) {
	return_code returnval = return_OK;
	returnval = subsystem_output_update((system_generic_T*)model, model->tick * model->sample_time, ODE_solver_step_major);
	if (returnval != return_OK)
		return returnval;
	model->tick++;
	return return_OK;
}

/**
 * Initializes discrete model system.
 * Copies original system's interface.
 * Registers specific step callback functions.
 * @param model discrete model structure pointer
 * @param system original discrete system structure pointer
 * @param sample_time sample time of model
 * @return system control library error code
 */
return_code model_discrete_init(model_discrete_T* model,const system_generic_T* system, real_t sample_time) {
#ifdef ASSERT_NULLPTR
	if (system == NULL || model == NULL)
		return return_NULLPTR;
#endif
	model->step_fcn = (discrete_system_step_fcn_T) model_discrete_step_fcn;
	model->interface = *system;
	model->sample_time = sample_time;
	model->tick=0;
	return return_OK;
}

static uint model_continuous_add_continuous_systems_to_list(list_t* list,const system_generic_T* subsystem, uint order) {
	list_item_t* item = subsystem->execution_list.front;
	while (item != NULL) {
		system_generic_T* system = (system_generic_T*) item->object;
		if (system->time_domain == system_time_domain_continuous && system->type == system_type_dynamic_system) {
			list_push_back(list, system);
			order += continuous_system_generic_get_order((continuous_system_generic_T*) system);
		} else if (system->type == system_type_subsystem) {
			order += model_continuous_add_continuous_systems_to_list(list, system, order);
		}
		item = item->next;
	}
	return order;
}
/**
 * Initializes continuous model structure.
 * Copies original system's interface.
 * Registers specific states derivatives and output update callback functions.
 * Filters out continuous systems list.
 * @param model continuous model structure pointer
 * @param system original continuous system structure pointer
 * @return system control library error code
 */
return_code model_continuous_init(model_continuous_T* model,const  system_generic_T* system) {
#ifdef ASSERT_NULLPTR
	if (model == NULL || system == NULL)
		return return_NULLPTR;
#endif
	return_code returnval = return_OK;
	model->interface = *system;
	model->interface.deinit_fcn = (system_deinit_fcn)model_continuous_deinit;
	size_t order = model_continuous_add_continuous_systems_to_list(&model->continuous_systems_list, system, 0);
	returnval = continuous_system_generic_init((continuous_system_generic_T*) model, order, NULL, NULL, (continuous_system_derivatives_fcn) model_continuous_get_derivatives, (continuous_system_update_output_fcn) model_continuous_update_outputs);
	if (returnval != return_OK)
		return returnval;
	return return_OK;
}

/**
 * De-initializes continuous model structure.
 * De-initializes states and states derivatives matrices.
 * Clears continuous systems linked list.
 * @param model continuous model structure pointer
 * @return system control library error code
 */
return_code model_continuous_deinit(model_continuous_T* model) {
	return_code returnval = return_OK;
	returnval = continuous_system_generic_deinit((continuous_system_generic_T*)model);
	if (returnval != return_OK)
		return returnval;
	return list_clear(&model->continuous_systems_list);
}

/**
 * Continuous model states derivatives function.
 * Processes states derivatives of all continuous subsystems and fills derivatives vector of model.
 * @param model continuous model structure pointer
 * @param time  current step time
 * @param step_type current step type
 * @return continuous system's states derivatives vector pointer
 */
const vector_generic_T* model_continuous_get_derivatives(model_continuous_T* model, real_t time, ODE_solver_step_t step_type) {

	list_item_t* item = model->continuous_systems_list.front;
	uint state = 0;
	while (item != NULL) {
		continuous_system_generic_T* system = (continuous_system_generic_T*) item->object;
		const vector_generic_T* dX = system->derrivatives_fcn(system, time, step_type);
		vector_generic_T dX_subvector = { 0 };
		vector_subvector(&dX_subvector, &model->dX, continuous_system_generic_get_order(system), state);
		vector_generic_copy(&dX_subvector,dX);
		state += continuous_system_generic_get_order(system);
		item = item->next;
	}
	return &model->dX;
}

/**
 * Continuous model states output update function.
 * Processes model states updated by solver and loads data to all continuous subsystems states vectors.
 * @param model continuous model structure pointer
 * @param time current step time
 * @param step_type current step type
 * @return system control library error code
 */
return_code model_continuous_update_outputs(model_continuous_T* model, real_t time, ODE_solver_step_t step_type) {
	return_code returnval = return_OK;
	list_item_t* item = model->continuous_systems_list.front;
	uint state = 0;
	while (item != NULL) {
		continuous_system_generic_T* system = (continuous_system_generic_T*) item->object;
		vector_generic_T X_subvector={0};
		vector_subvector(&X_subvector,&model->X, continuous_system_generic_get_order(system),state);
		vector_generic_copy(&system->X,&X_subvector);
		returnval = continuous_system_generic_update_output(system, time, step_type);
		if (returnval != return_OK)
			return returnval;
		state += continuous_system_generic_get_order(system);
		item = item->next;
	}
	return subsystem_output_update(&model->interface, time, step_type);

}

/**
 * Evaluation of generic system - state/output update.
 * According to system's type @ref system_type type  and time domain @ref system_time_domain, function call target system interface.
 * Processes all system types:
 *  - @ref static_systems
 *  - @ref discrete_systems
 *  - @ref continuous_systems
 *  - @ref signal_sources
 *  - @ref signal_sinks
 *
 * @param system pointer to generic system structure
 * @param time current step time
 * @param step_type current step type
 * @return system control library error code
 */
return_code subsystem_output_update(system_generic_T* system, real_t time, ODE_solver_step_t step_type) {

	return_code returnval = return_OK;
	list_item_t* item = system->execution_list.front;
	while (item != NULL) {
		system_generic_T* system = (system_generic_T*) item->object;
		if (system == NULL)
			return return_NULLPTR;

		switch (system->type) {
		case system_type_static_system: {
			returnval = static_system_eval_linked((static_system_generic_T*) system);
			if (returnval != return_OK)
				return returnval;
			break;
		}
		case system_type_dynamic_system: {
			if (system->time_domain == system_time_domain_continuous) {
				returnval = continuous_system_generic_update_output((continuous_system_generic_T*) system, time, step_type);
			} else if (system->time_domain == system_time_domain_discrete && step_type == ODE_solver_step_major) {
				returnval = discrete_system_generic_step_linked((discrete_system_generic_T*) system);
			} else {
				returnval = return_ERROR;
			}
			if (returnval != return_OK)
				return returnval;
		}
			break;
		case system_type_signal_source: {
			returnval = signal_source_eval_linked((signal_source_generic_T*) system, time);
			if (returnval != return_OK)
				return returnval;
			break;
		}
		case system_type_signal_sink: {
			returnval = signal_sink_eval_linked((signal_sink_generic_T*) system, time);
			if (returnval != return_OK)
				return returnval;
			break;
		}
		case system_type_subsystem: {
			returnval = subsystem_output_update(system, time, step_type);
			if (returnval != return_OK)
				return returnval;
			break;
		}
		default:
			return return_ERROR;
		}
		item = item->next;
	}

	return return_OK;
}

