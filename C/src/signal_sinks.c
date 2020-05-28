
#include "signal_sinks.h"



return_code signal_sink_generic_deinit(signal_sink_generic_T* system) {
	system->input = 0;
	system->function = NULL;
	return return_OK;
}

return_code signal_sink_generic_init(signal_sink_generic_T* system, signal_sink_fcn function, system_deinit_fcn deinit_fcn) {
	return_code returnval = return_OK;

	if (deinit_fcn == NULL)
		deinit_fcn = (system_deinit_fcn) signal_sink_generic_deinit;

	returnval = system_interface_init(&system->interface, system_type_signal_sink, system_time_domain_discrete | system_time_domain_continuous, 1, &system->input, 0, NULL, deinit_fcn);
	if (returnval != return_OK)
		return returnval;
	system->input = 0;
	system->function = function;
	return return_OK;
}

static return_code signal_sink_timeseries_fcn(signal_sink_timeseries_T* signal_sink, real_t time,const vector_generic_T* u_vector) {

	return signal_timeseries_write(signal_sink->timeseries, time, *vector_type_at(u_vector,signal_realtime_T,0).ptr);
}

return_code signal_sink_timeseries_init(signal_sink_timeseries_T* signal_sink, signal_timeseries_T* timeseries) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (signal_sink == NULL || timeseries == NULL)
		return return_NULLPTR;
	returnval = vector_assert(&timeseries->time_vector);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(&timeseries->values_vector);
	if (returnval != return_OK)
		return returnval;
#endif
	signal_sink->timeseries = timeseries;
	return signal_sink_generic_init((signal_sink_generic_T*) signal_sink, (signal_sink_fcn) signal_sink_timeseries_fcn, NULL);
}

static return_code signal_sink_sampled_fcn(signal_sink_sampled_T* signal_sink, real_t time, const vector_generic_T* u_vector) {
	return signal_sampled_write_value(signal_sink->sampled_data, time,*vector_type_at(u_vector,signal_realtime_T,0).ptr);
}

return_code signal_sink_sampled_init(signal_sink_sampled_T* signal_sink, signal_sampled_T* sampled_data) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (signal_sink == NULL || sampled_data == NULL)
		return return_NULLPTR;
	returnval = vector_assert((vector_generic_T*) (sampled_data));
	if (returnval != return_OK)
		return returnval;
#endif
	signal_sink->sampled_data = sampled_data;
	return signal_sink_generic_init((signal_sink_generic_T*) signal_sink, (signal_sink_fcn) signal_sink_sampled_fcn, NULL);
}
