#include "model.h"

using namespace SystemControl;

ContinuousSubsystem::ContinuousSubsystem(SubSystem& subsystem) :
		subsystem(subsystem) {
}

uint ContinuousSubsystem::add_continuous_systems_to_list(System* system, uint order) {

	if (system == NULL)
		throw exception_NULLPTR;
	if (system->time_domain == System::system_time_domain_continuous && system->type == System::system_type_dynamic_system) {
		ContinuousSystem* continuous_system = dynamic_cast<ContinuousSystem*>(system);
		if (continuous_system == NULL)
			throw exception_NULLPTR;
		continuous_systems_list.push_back(continuous_system);
		order += continuous_system->get_order();
	} else if (system->type == System::system_type_subsystem) {
		SubSystem* subsystem = dynamic_cast<SubSystem*>(system);
		if (subsystem == NULL)
			throw exception_NULLPTR;
		auto lambda = [&](System* system,uint i)->auto {
			order+=add_continuous_systems_to_list(system,order);
			return true;
		};
		subsystem->execution_list.for_each(lambda);
	} else
		return 0;

	return order;
}

ModelContinuous::ModelContinuous(SubSystem& subsystem) :
		ContinuousSubsystem(subsystem), ContinuousSystemInterface(add_continuous_systems_to_list(&subsystem, 0), NULL, NULL) {
}

void ModelContinuous::update_derivatives_fcn(real_t time, step_type step_type) {

	uint state = 0;
	auto lambda = [&](ContinuousSystem* system_cont,uint i)->auto {
		if (system_cont == NULL)
		throw exception_NULLPTR;
		system_cont->update_derivatives_fcn(time, step_type);
		//VectorReal dX_subvector=dX.subvector(system_cont->get_order(),state);
		VectorReal dX_subvector(system_cont->get_order(),dX.get_data_ptr()+state);
		dX_subvector = system_cont->get_derivatives();
		state += system_cont->get_order();
		return true;
		};
	continuous_systems_list.for_each(lambda);

}
void ModelContinuous::update_output_fcn(real_t time, step_type step_type) {

	uint state = 0;
	auto lambda = [&](ContinuousSystem* system_cont,uint i)->auto {
		if (system_cont == NULL)
		throw exception_NULLPTR;
		//VectorReal X_subvector=X.subvector(system_cont->get_order(),state);
		VectorReal X_subvector(system_cont->get_order(),X.get_data_ptr()+state);
		system_cont->get_states() = X_subvector;
		system_cont->update_output_fcn(time, step_type);
		state += system_cont->get_order();
		return true;
		};
	continuous_systems_list.for_each(lambda);
	subsystem.update(time, step_type);
}

