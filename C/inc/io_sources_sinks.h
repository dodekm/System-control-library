/**
 * @file io_sources_sinks.h
 * @brief
 */


#ifndef IO_SOURCES_SINKS_H_
#define IO_SOURCES_SINKS_H_

#include "signal_sources.h"
#include "signal_sinks.h"
#include "io_interface.h"
#include "data_stream.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct signal_source_digital_input {
	signal_source_generic_T;
	io_digital_input_generic_t* io_digital_input_ptr;
} signal_source_digital_input_T;

return_code signal_source_digital_input_init(signal_source_digital_input_T*, io_digital_input_generic_t*);

typedef struct signal_sink_digital_output {
	signal_sink_generic_T;
	io_digital_output_generic_t* io_digital_output_ptr;
} signal_sink_digital_output_T;

return_code signal_sink_digital_output_init(signal_sink_digital_output_T*, io_digital_output_generic_t*);

typedef struct signal_source_analog_input {
	signal_source_generic_T;
	io_analog_input_generic_t* io_analog_input_ptr;
	uint channel;
} signal_source_analog_input_T;

return_code signal_source_analog_input_init(signal_source_analog_input_T*, io_analog_input_generic_t*, uint);

typedef struct signal_sink_analog_output {
	signal_sink_generic_T;
	io_analog_output_generic_t* io_analog_output_ptr;
} signal_sink_analog_output_T;

return_code signal_sink_analog_output_init(signal_sink_analog_output_T*, io_analog_output_generic_t*);

typedef struct signal_sink_PWM_output {
	signal_sink_generic_T;
	io_PWM_output_generic_t* io_PWM_output_ptr;
} signal_sink_PWM_output_T;

return_code signal_sink_PWM_output_init(signal_sink_PWM_output_T*, io_PWM_output_generic_t*);

typedef struct signal_source_data_stream {
	signal_source_generic_T;
	data_stream_instance* data_stream_ptr;
	data_stream_id_t stream_id;

} signal_source_data_stream_T;



#ifdef __cplusplus
}
#endif

#endif /* IO_SOURCES_SINKS_H_ */
