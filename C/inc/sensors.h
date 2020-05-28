/**
 * @file sensors.h
 * @brief
 */



#ifndef TEMP_SENSOR_H_
#define TEMP_SENSOR_H_

#include "common_def.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct sensor_NTC_thermistor {
	real_t A;
	real_t B;
	real_t C;
} sensor_NTC_thermistor_t;

static inline real_t sensor_voltage_divider_U_to_R(real_t U_in, real_t U_ref, real_t R_serial) {
	return (U_in / (U_ref - U_in)) * R_serial;
}
static inline real_t sensor_voltage_divider_analog_input_to_R(real_t in, real_t R_serial) {
	return (in / (1.0 - in)) * R_serial;
}
return_code sensor_NTC_thermistor_init(sensor_NTC_thermistor_t*, real_t, real_t, real_t);
real_t sensor_NTC_thermistor_R_to_T(sensor_NTC_thermistor_t*, real_t);

typedef struct sensor_linear {
	real_t k;
	real_t q;
} sensor_linear_t;

return_code sensor_linear_init(sensor_linear_t*, real_t, real_t);
real_t sensor_linear_convert(sensor_linear_t*, real_t);


#ifdef __cplusplus
}
#endif

#endif
