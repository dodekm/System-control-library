
#include "io_sources_sinks.h"

static return_code signal_source_digital_input_fcn(signal_source_digital_input_T* signal_source, real_t time,  vector_generic_T* y_vector) {

	(void) time;
	*vector_type_at(y_vector,signal_realtime_T,0).ptr = (real_t) io_digital_input_read(signal_source->io_digital_input_ptr);
	return return_OK;
}

return_code signal_source_digital_input_init(signal_source_digital_input_T* signal_source, io_digital_input_generic_t* io_digital_input) {

#ifdef ASSERT_NULLPTR
	if (signal_source == NULL)
		return return_NULLPTR;
	if (io_digital_input == NULL)
		return return_NULLPTR;
#endif

	signal_source->io_digital_input_ptr = io_digital_input;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_digital_input_fcn, NULL);
}

static return_code signal_sink_digital_output_fcn(signal_sink_digital_output_T* signal_sink, real_t time,  const vector_generic_T* u_vector) {
	(void) time;
	io_digital_output_write(signal_sink->io_digital_output_ptr, (bool_t) *vector_type_at(u_vector,signal_realtime_T,0).ptr);
	return return_OK;
}

return_code signal_sink_digital_output_init(signal_sink_digital_output_T* signal_sink, io_digital_output_generic_t* io_digital_output) {

#ifdef ASSERT_NULLPTR
	if (signal_sink == NULL)
		return return_NULLPTR;
	if (io_digital_output == NULL)
		return return_NULLPTR;
#endif

	signal_sink->io_digital_output_ptr = io_digital_output;
	return signal_sink_generic_init((signal_sink_generic_T*) signal_sink, (signal_sink_fcn) signal_sink_digital_output_fcn, NULL);
}

static return_code signal_source_analog_input_fcn(signal_source_analog_input_T* signal_source, real_t time,  vector_generic_T* y_vector) {

	(void) time;
	return_code returnval = io_analog_input_sample(signal_source->io_analog_input_ptr);
	if (returnval != return_OK)
		return returnval;
	*vector_type_at(y_vector,signal_realtime_T,0).ptr = io_analog_input_read_real(signal_source->io_analog_input_ptr, signal_source->channel);

	return return_OK;
}

return_code signal_source_analog_input_init(signal_source_analog_input_T* signal_source, io_analog_input_generic_t* io_analog_input, uint channel) {

#ifdef ASSERT_NULLPTR
	if (signal_source == NULL)
		return return_NULLPTR;
	if (io_analog_input == NULL)
		return return_NULLPTR;
#endif

	signal_source->io_analog_input_ptr = io_analog_input;
	signal_source->channel = channel;
	return signal_source_generic_init((signal_source_generic_T*) signal_source, (signal_source_fcn) signal_source_analog_input_fcn, NULL);
}

static return_code signal_sink_analog_output_fcn(signal_sink_analog_output_T* signal_sink, real_t time,  const vector_generic_T* u_vector) {
	(void) time;
	return io_analog_output_write_real(signal_sink->io_analog_output_ptr, *vector_type_at(u_vector,signal_realtime_T,0).ptr);
}

return_code signal_sink_analog_output_init(signal_sink_analog_output_T* signal_sink, io_analog_output_generic_t* io_analog_output) {

#ifdef ASSERT_NULLPTR
	if (signal_sink == NULL)
		return return_NULLPTR;
	if (io_analog_output == NULL)
		return return_NULLPTR;
#endif

	signal_sink->io_analog_output_ptr = io_analog_output;
	return signal_sink_generic_init((signal_sink_generic_T*) signal_sink, (signal_sink_fcn) signal_sink_analog_output_fcn, NULL);
}

static return_code signal_sink_PWM_output_fcn(signal_sink_PWM_output_T* signal_sink, real_t time, const vector_generic_T* u_vector) {
	(void) time;
	return io_PWM_output_write_real(signal_sink->io_PWM_output_ptr, *vector_type_at(u_vector,signal_realtime_T,0).ptr);
}

return_code signal_sink_PWM_output_init(signal_sink_PWM_output_T* signal_sink, io_PWM_output_generic_t* io_PWM_output) {

#ifdef ASSERT_NULLPTR
	if (signal_sink == NULL)
		return return_NULLPTR;
	if (io_PWM_output == NULL)
		return return_NULLPTR;
#endif

	signal_sink->io_PWM_output_ptr = io_PWM_output;
	return signal_sink_generic_init((signal_sink_generic_T*) signal_sink, (signal_sink_fcn) signal_sink_PWM_output_fcn, NULL);
}


