/**
 * @file signal_process.h
 */

#ifndef INC_SIGNAL_PROCESS_H_
#define INC_SIGNAL_PROCESS_H_

#include "common_def.h"
#include "complex.h"
#include "vector.h"

/** @addtogroup signal_process Signal processing
 * @brief Real and complex vectors extension functions.
 *
 * # Implemented functionality:
 * 	- elementary real vector operations (sum,difference,integrate,dot product,reverse elements...)
 * 	- generic binary and unary element wise operations
 * 	- real and complex discrete convolution
 * 	- linear interpolation
 * 	- sampled signal structure
 * 	- timeseries signal structure
 *
 * @{
 */

/**
 * Structure for sampled signal.
 * Generic real number vector with sample time information and sampling index
 */
typedef struct signal_sampled {
	vector_generic_T;	///<inherited generic vector structure
	uint idx; 			///<sampling index - current index of vector to sample/read
	real_t Ts; 			///<sample time
} signal_sampled_T;


/**
 * Structure for timeseries signal.
 * Real number data paired to corresponding timestamp
 */
typedef struct signal_timeseries {
	vector_generic_T values_vector; ///< vector of real number values
	vector_generic_T time_vector; 	///< vector of corresponding timestamp values
	uint idx;						///< last indexed
} signal_timeseries_T;

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Linear to logarithmic scale.
 * Converts magnitude to decibels
 * @param mag magnitude
 * @return decibels
 */
static inline real_t mag2dB(real_t mag) {
	if (mag > 0.0)
		return 20.0 * log10(mag);
	else
		return NAN;
}

/**
 * Logarithmic scale to linear.
 * Converts decibels to magnitude
 * @param dB decibels
 * @return magnitude
 */
static inline real_t dB2mag(real_t dB) {
	return exp(dB / 20.0 * log(10));
}

return_code vector_real_convolute(const vector_generic_T*,const vector_generic_T*, vector_generic_T*);
return_code vector_complex_convolute(const vector_generic_T*,const vector_generic_T*, vector_generic_T*);
return_code vector_real_convolute_symetric_kernel(const vector_generic_T*,const vector_generic_T*, vector_generic_T*);

return_code vector_real_reverse(vector_generic_T*);
return_code vector_real_interpolate(const vector_generic_T*,const vector_generic_T*,const vector_generic_T*, vector_generic_T*);
return_code vector_real_diff(const vector_generic_T*, vector_generic_T*);
return_code vector_real_integrate(const vector_generic_T*, vector_generic_T*);
return_code vector_real_sum(const vector_generic_T*, real_t*);
return_code vector_real_dot_product(const vector_generic_T*,const vector_generic_T*, real_t*);
return_code vector_real_transform_linear(const vector_generic_T*, vector_generic_T*, real_t, real_t);
return_code vector_real_element_binary_operator(const vector_generic_T*,const vector_generic_T*, vector_generic_T*, real_binary_operator_T);
return_code vector_real_element_unary_operator(const vector_generic_T*, vector_generic_T*, real_unary_operator_T);

return_code vector_real_scalar_operator(const vector_generic_T*, vector_generic_T*, real_t, real_binary_operator_T);
return_code vector_real_filtrate(const vector_generic_T*,const vector_generic_T*, vector_generic_T*);
return_code vector_real_filtrate_symetric(const vector_generic_T*,const vector_generic_T*,vector_generic_T*);

return_code signal_sampled_init(signal_sampled_T*, size_t, real_t, real_t*);

return_code signal_sampled_reset(signal_sampled_T*);
return_code signal_sampled_deinit(signal_sampled_T*);

return_code signal_sampled_sample_value(signal_sampled_T*, real_t);


/**
 * Reads value form time indexed sampled signal.
 * @param signal pointer to sampled signal structure
 * @param time time to index signal at
 * @param value_ptr pointer to result/return value
 * @return return_INDEX_OUT_OF_RANGE if time is not within signal duration
 */
static inline return_code signal_sampled_read_value(const signal_sampled_T* signal, real_t time, real_t* value_ptr) {
	return vector_type_get_value_safe((vector_generic_T*) signal,real_t, (uint) (time / signal->Ts), value_ptr);
}

/**
 * Writes value to time indexed sampled signal.
 * @param signal pointer to sampled signal structure
 * @param time time to index signal at
 * @param value source value
 * @return return_INDEX_OUT_OF_RANGE if time is not within signal duration
 */
static inline return_code signal_sampled_write_value(signal_sampled_T* signal, real_t time, real_t value) {
	return vector_type_set_value_safe((vector_generic_T*) signal,real_t ,(uint) (time / signal->Ts), value);
}

return_code signal_timeseries_init(signal_timeseries_T*, size_t, real_t*, real_t*);
return_code signal_timeseries_deinit(signal_timeseries_T*);

return_code signal_timeseries_load_data(signal_timeseries_T*,const real_t*,const real_t*);
return_code signal_timeseries_reset(signal_timeseries_T*);

return_code signal_timeseries_write(signal_timeseries_T*, real_t, real_t);
return_code signal_timeseries_read(signal_timeseries_T*, real_t, real_t*);


#ifdef __cplusplus
}
#endif

/**@} */

#endif /* INC_SIGNAL_PROCESS_H_ */
