/**
 * @file signal_sinks.h
 */

#ifndef SIGNAL_SINKS_H_
#define SIGNAL_SINKS_H_

#include "common_def.h"
#include "systems.h"
#include "signal_process.h"

/** @addtogroup signal_sinks Signal sinks
 * @brief
 *
 * @{
 */

struct signal_sink_generic;

/**
 *
 * @param - pointer to signal sink structure
 * @param - time
 * @param - source signal vector
 * @return system control library error code
 */
typedef return_code (*signal_sink_fcn)(struct signal_sink_generic*, real_t, const vector_generic_T*);

/**
 *
 */
typedef struct signal_sink_generic {
	system_generic_T interface;
	real_t input;
	signal_sink_fcn function;
} signal_sink_generic_T;

/**
 *
 */
typedef struct signal_sink_sampled {
	signal_sink_generic_T;
	signal_sampled_T* sampled_data;
} signal_sink_sampled_T;

/**
 *
 */
typedef struct signal_sink_timeseries {
	signal_sink_generic_T;
	signal_timeseries_T* timeseries;
} signal_sink_timeseries_T;

#ifdef __cplusplus
extern "C" {
#endif

return_code signal_sink_generic_deinit(signal_sink_generic_T*);
return_code signal_sink_generic_init(signal_sink_generic_T*, signal_sink_fcn, system_deinit_fcn);

/**
 *
 * @param signal_sink
 * @param time
 * @param u_vector
 * @return
 */
static inline return_code signal_sink_eval_direct(signal_sink_generic_T* signal_sink, real_t time, const vector_generic_T* u_vector) {
	return signal_sink->function(signal_sink, time, u_vector);
}

static inline return_code signal_sink_eval_direct_scalar(signal_sink_generic_T* signal_sink, real_t time, real_t u) {

	vector_generic_T u_vector = { 0 };
	signal_realtime_T signal_u;
	signal_u.owner = NULL;
	signal_u.ptr = &u;
	vector_type_init(&u_vector, signal_realtime_T, 1, &signal_u);
	return signal_sink->function(signal_sink, time, &u_vector);
}

/**
 *
 * @param signal_sink
 * @param time
 * @return
 */
static inline return_code signal_sink_eval_linked(signal_sink_generic_T* signal_sink, real_t time) {
	return signal_sink_eval_direct(signal_sink, time, system_input_port_vector_ptr(&signal_sink->interface));
}

return_code signal_sink_timeseries_init(signal_sink_timeseries_T*, signal_timeseries_T*);
return_code signal_sink_sampled_init(signal_sink_sampled_T*, signal_sampled_T*);

#ifdef __cplusplus
}
#endif

/** @} */
#endif /* SIGNAL_SINKS_H_ */
