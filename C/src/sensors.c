

#include "sensors.h"

#define KELVIN_TO_CELSIUS(T) ((T)-273.15)

return_code sensor_NTC_thermistor_init(sensor_NTC_thermistor_t* thermistor, real_t A, real_t B, real_t C) {
#ifdef ASSERT_NULLPTR
	if (thermistor == NULL)
		return return_NULLPTR;
#endif
	thermistor->A = A;
	thermistor->B = B;
	thermistor->C = C;
	return return_OK;
}
real_t sensor_NTC_thermistor_R_to_T(sensor_NTC_thermistor_t* thermistor, real_t R) {
	real_t ln_R = log(R);
	real_t ln_R_pow3 = POW3(ln_R);
	real_t T_recip = thermistor->A + thermistor->B * ln_R + thermistor->C * ln_R_pow3;
	return KELVIN_TO_CELSIUS(1.0 / T_recip);
}

return_code sensor_linear_init(sensor_linear_t* sensor, real_t k, real_t q) {
#ifdef ASSERT_NULLPTR
	if (sensor == NULL)
		return return_NULLPTR;
#endif
	sensor->k = k;
	sensor->q = q;
	return return_OK;

}
real_t sensor_linear_convert(sensor_linear_t* sensor, real_t in) {
	return transform_linear(in, sensor->k, sensor->q);
}
