/**
 * @file control_synthesis.h
 */

#ifndef INC_CONTROL_SYNTHESIS_H_
#define INC_CONTROL_SYNTHESIS_H_

#include "common_def.h"
#include "vector.h"
#include "polynom.h"
#include "matrix.h"

/**
 * @addtogroup control_synthesis Control synthesis
 * @brief Control systems parameters synthesis algorithms.
 * Regulators parameters design using pole-placement method.
 * # Implemented functionality:
 *	- pole-placement method for RST polynomial regulator and generic discrete transfer function
 * 	- pole-placement method for PI/IP regulator and first order transfer function
 * 	- pole-placement method for PIV (P+IP) regulator and first order astatic transfer function
 * 	- desired characteristic polynomial structures for second and third order systems
 * 	- desired discrete characteristic polynomial shaping from continuous roots or multiple time constant
 * 	- convolution matrix subroutine
 */


#ifdef __cplusplus
extern "C" {
#endif

return_code control_synthesis_create_discrete_polynom_from_continuous_poles(const vector_generic_T*, polynom_T*, real_t);
return_code control_synthesis_create_discrete_aperiodic_polynom_for_multiple_time_constant(polynom_T*, uint, real_t, real_t);
return_code control_synthesis_create_discrete_polynom_from_multiple_complex_root(polynom_T*, uint, complex_t);

return_code control_synthesis_convolution_matrix(const vector_generic_T*, matrix_T*, uint);
#ifdef USE_GSL
return_code control_synthesis_RST_poleplace(const polynom_T*, const polynom_T*, const polynom_T*, polynom_T*, polynom_T*, polynom_T*);
#endif

/**
 * First order system parameters structure.
 */
typedef struct control_synthesis_first_order_system_params {
	real_t K; ///<system gain
	real_t T; ///<system time constant
} control_synthesis_first_order_system_params_T;

/**
 * Second order closed loop characteristic polynomial parameters structure.
 */
typedef struct control_synthesis_desired_polynom_2nd_order {
	real_t omega_0; ///<own frequency
	real_t b;  ///<damping
} control_synthesis_desired_polynom_2nd_order_T;

/**
 * Third order closed loop characteristic polynomial parameters structure.
 */
typedef struct control_synthesis_desired_polynom_3rd_order {
	control_synthesis_desired_polynom_2nd_order_T; ///< Second order desired polynomial parameters
	real_t k; ///< real pole
} control_synthesis_desired_polynom_3rd_order_T;

return_code control_synthesis_PI_poleplace(const control_synthesis_first_order_system_params_T*, const control_synthesis_desired_polynom_2nd_order_T*, PID_regulator_params_T*);
return_code control_synthesis_PIV_poleplace(const control_synthesis_first_order_system_params_T*, const control_synthesis_desired_polynom_3rd_order_T*, PID_regulator_params_T*, PID_regulator_params_T*);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* INC_CONTROL_SYNTHESIS_H_ */
