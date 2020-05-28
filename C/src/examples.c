#include "examples.h"

extern return_code continuous_MIMO_system_generic_deinit(continuous_MIMO_system_generic_T*);
extern return_code continuous_MIMO_system_generic_init(continuous_MIMO_system_generic_T*, size_t, real_t*, real_t*, continuous_system_derivatives_fcn, continuous_system_update_output_fcn, size_t, real_t*, size_t, real_t*, system_deinit_fcn);

return_code continuous_system_pendulum_derrivatives_fcn(real_t dX[], const real_t X[], const real_t params[], real_t u, real_t t) {
	real_t m = params[0];
	real_t l = params[1];
	real_t g = params[2];
	real_t b = params[3];

	real_t theta = X[0];
	real_t omega = X[1];

	dX[0] = omega;
	dX[1] = -g / l * sin(theta) - b / m * omega + u / m / l / l;

	return return_OK;
}

return_code continuous_system_pendulum_output_fcn(const real_t X[], const real_t params[], real_t u, real_t* y) {
	y[0] = X[0];
	return return_OK;
}

static vector_generic_T* continuous_system_DC_motor_get_derivatives_fcn(continuous_system_DC_motor_T*, real_t, ODE_solver_step_t);
static return_code continuous_system_DC_motor_upadate_output_fcn(continuous_system_DC_motor_T*, real_t, ODE_solver_step_t);

static return_code continuous_system_DC_motor_deinit(continuous_system_DC_motor_T* model) {

	model->Cu = 0;
	model->Te = 0;
	model->R = 0;
	model->J = 0;
	model->B = 0;
	return continuous_MIMO_system_generic_deinit((continuous_MIMO_system_generic_T*) model);
}

return_code continuous_system_DC_motor_init(continuous_system_DC_motor_T* model, real_t Cu, real_t Te, real_t R, real_t J, real_t B) {

	model->Cu = Cu;
	model->Te = Te;
	model->R = R;
	model->J = J;
	model->B = B;

	return continuous_MIMO_system_generic_init((continuous_MIMO_system_generic_T*) model, 3, model->dX_data, model->X_data, (continuous_system_derivatives_fcn) continuous_system_DC_motor_get_derivatives_fcn, (continuous_system_update_output_fcn) continuous_system_DC_motor_upadate_output_fcn, 2, model->input, 3, model->output, (system_deinit_fcn) continuous_system_DC_motor_deinit);
}

static vector_generic_T* continuous_system_DC_motor_get_derivatives_fcn(continuous_system_DC_motor_T* model, real_t time, ODE_solver_step_t step_type) {

	real_t current = vector_type_at(&model->X, real_t, 0);
	real_t omega = vector_type_at(&model->X, real_t, 1);
	real_t theta = vector_type_at(&model->X, real_t, 2);

	(void) theta;
	real_t U = system_input_port_real_value(&model->interface, 0);
	real_t M_ext = system_input_port_real_value(&model->interface, 1);

	vector_type_at(&model->dX,real_t,0) = ((U - model->Cu * omega) - current * model->R) / model->Te / model->R;
	vector_type_at(&model->dX,real_t,1) = (model->Cu * current - M_ext - model->B * omega) / model->J;
	vector_type_at(&model->dX,real_t,2) = omega;

	return &model->dX;
}
static return_code continuous_system_DC_motor_upadate_output_fcn(continuous_system_DC_motor_T* model, real_t time, ODE_solver_step_t step_type) {

	(void) time;
	(void) step_type;
	system_output_port_real_value(&model->interface, 0) = vector_type_at(&model->X,real_t, 0); //current
	system_output_port_real_value(&model->interface, 1) = vector_type_at(&model->X,real_t, 1); //omega
	system_output_port_real_value(&model->interface, 2) = vector_type_at(&model->X,real_t ,2); //theta
	return return_OK;
}

return_code continuous_subsystem_DC_motor_init(continuous_subsystem_DC_motor_T* model, real_t Cu, real_t Te, real_t R, real_t J, real_t B) {
	return_code returnval = return_OK;
	returnval = continuous_transfer_function_first_order_init(&model->TF_electrical, Te, 1 / R);
	if (returnval != return_OK)
		return returnval;
	returnval = continuous_transfer_function_first_order_init(&model->TF_mechanical, J / B, 1 / B);
	if (returnval != return_OK)
		return returnval;
	returnval = continuous_integrator_init(&model->integrator);
	if (returnval != return_OK)
		return returnval;
	returnval = static_system_gain_init(&model->gain_Cu_direct, Cu);
	if (returnval != return_OK)
		return returnval;
	returnval = static_system_gain_init(&model->gain_Cu_back, Cu);
	if (returnval != return_OK)
		return returnval;
	returnval = static_system_sub_init(&model->sub_U);
	if (returnval != return_OK)
		return returnval;
	returnval = static_system_sub_init(&model->sub_M);
	if (returnval != return_OK)
		return returnval;

	{
		system_generic_T* direct_feed_systems_list[] = { (system_generic_T*) &model->TF_electrical, (system_generic_T*) &model->gain_Cu_direct, (system_generic_T*) &model->sub_M, (system_generic_T*) &model->TF_mechanical };
		uint input_port_numbers[] = { 0, 0, 0, 0 };
		uint output_port_numbers[] = { 0, 0, 0, 0 };
		returnval = systems_connect_serial(direct_feed_systems_list, array_length(direct_feed_systems_list), input_port_numbers, output_port_numbers, &model->direct_feed_subsystem);
		if (returnval != return_OK)
			return returnval;
	}
	returnval = systems_connect_feedback((system_generic_T*) &model->sub_U, (system_generic_T*) &model->direct_feed_subsystem, (system_generic_T*) &model->gain_Cu_back, &model->speed_subsystem);
	if (returnval != return_OK)
		return returnval;
	{
		uint input_port_numbers[] = { 0, 0 };
		uint output_port_numbers[] = { 0, 0 };
		system_generic_T* position_systems_list[] = { (system_generic_T*) &model->speed_subsystem, (system_generic_T*) &model->integrator };
		returnval = systems_connect_serial(position_systems_list, array_length(position_systems_list), input_port_numbers, output_port_numbers, (system_generic_T*) &model->position_subsystem);
		if (returnval != return_OK)
			return returnval;
	}
	returnval = system_interface_init((system_generic_T*) model, system_type_subsystem, system_time_domain_continuous, 2, NULL, 3, NULL, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = list_push_back(&model->execution_list, (void*) &model->position_subsystem);
	if (returnval != return_OK)
		return returnval;
	system_input_port_struct((system_generic_T*)model,0) = system_input_port_struct(&model->position_subsystem, 0);
	system_input_port_struct((system_generic_T*)model,1) = system_input_port_struct((system_generic_T* )&model->sub_M, 1);
	system_output_port_struct((system_generic_T*)model,0) = system_output_port_struct((system_generic_T* )&model->TF_electrical, 0);
	system_output_port_struct((system_generic_T*)model,1) = system_output_port_struct(&model->speed_subsystem, 0);
	system_output_port_struct((system_generic_T*)model,2) = system_output_port_struct(&model->position_subsystem, 0);
	return return_OK;
}
