/**
 * @file systems_analyze.h
 * */
#ifndef INC_SYSTEMS_ANALYZE_H_
#define INC_SYSTEMS_ANALYZE_H_

#include "common_def.h"
#include "complex.h"
#include "vector.h"
#include "matrix.h"
#include "systems.h"
#include "polynom.h"
#include "signal_process.h"

/** @addtogroup linear_systems Linear systems
 *  @ingroup systems
 *
 * # Transfer function
 * - Poles and zeros analyze -> stability analyze
 * - Static gain analyze and setting functions
 * - Conversions to state space and zeros-poles-gain representation
 * - Discretization using Forward Euler, Bilinear transform and Zeros-poles match methods
 * - Serial, parallel and feedback connection resulting transfer function
 *
 *
 * # State space
 * - Static gain analyze and setting functions
 * - Poles and zeros analyze using eigenvalues
 * - Discretization using resolvent matrix (matrix exponential)
 * - Conversion state space <-> transfer function
 * - Conversion to zero-pole-gain
 *
 * # Zeros-poles-gain
 * - Discretization using roots complex exponential transform
 * - Conversion to transfer function
 * - Stability analysis using poles location analysis
 *
 */

/** @addtogroup systems_convert Systems conversions
 *  @ingroup linear_systems
 *  @brief Conversions between various representations of continuous and discrete linear dynamic systems.
 * - transfer function <-> state space <-> zeros poles gain transformations
 * - continuous <-> discrete poles transformations
 * - discretization of systems - zero-pole match and bilinear transform
 *
 * */

/** @addtogroup systems_analyze Systems analyze
 *  @ingroup linear_systems
 *  @brief Dynamic and static behavior analysis of various representations of continuous and discrete linear dynamic systems.
 * - zeros and poles analyze
 * - stability analysis
 * - static gain analysis
 * */

#ifdef __cplusplus
extern "C" {
#endif
/** @addtogroup systems_convert
 *  @{ */
complex_t convert_continuous_root_to_discrete_root(complex_t, real_t);
complex_t convert_discrete_root_to_continuous_root(complex_t, real_t);
/** @} */

/** @addtogroup systems_analyze
 *  @{ */
bool_t analyze_roots_are_stable(const vector_generic_T*, system_time_domain);
bool_t analyze_transfer_function_is_stable(const transfer_function_T*, system_time_domain);
return_code analyze_transfer_function_gain(const transfer_function_T*, real_t*, system_time_domain);
/** @} */

/** @addtogroup systems_convert
 *  @{ */
return_code convert_transfer_function_to_state_space(const transfer_function_T*, state_space_T*, system_time_domain);
return_code convert_transfer_function_set_gain(transfer_function_T*, real_t, system_time_domain);
return_code convert_transfer_function_normalize(transfer_function_T*);
return_code convert_transfer_function_to_zeros_poles_gain(const transfer_function_T*, zeros_poles_gain_T*, system_time_domain);


return_code convert_transfer_function_continuous_to_discrete_forward_Euler(const transfer_function_T*, transfer_function_T*, real_t);
return_code convert_transfer_function_continuous_to_discrete_bilinear(const transfer_function_T*, transfer_function_T*, real_t);
return_code convert_transfer_function_continuous_to_discrete_match_zeros_poles(const transfer_function_T*, transfer_function_T*, real_t);

return_code convert_transfer_function_serial(const transfer_function_T*, const transfer_function_T*, transfer_function_T*);
return_code convert_transfer_function_paralel(const transfer_function_T*, const transfer_function_T*, transfer_function_T*);
return_code convert_transfer_function_feedback(const transfer_function_T*, const transfer_function_T*, transfer_function_T*);

return_code convert_zeros_poles_gain_continuous_to_discrete(const zeros_poles_gain_T*, zeros_poles_gain_T*, real_t);
return_code convert_zero_pole_gain_to_transfer_function(const zeros_poles_gain_T*, transfer_function_T*, system_time_domain);
/** @} */

/** @addtogroup systems_analyze
 *  @{ */
bool_t analyze_zero_pole_is_stable(const zeros_poles_gain_T*, system_time_domain);
/** @} */

/** @addtogroup systems_convert
 *  @{ */
return_code convert_state_space_continuous_to_discrete_Taylor_series(const state_space_T*, state_space_T*, real_t, uint);
#ifdef USE_GSL
return_code convert_state_space_continuous_to_discrete(const state_space_T*, state_space_T*, real_t);
return_code convert_state_space_to_transfer_function(const state_space_T*, transfer_function_T*, system_time_domain);
return_code convert_state_space_to_zeros_poles_gain(const state_space_T*, zeros_poles_gain_T*, system_time_domain);
return_code convert_state_space_set_gain(state_space_T*, real_t, system_time_domain);
/** @} */

/** @addtogroup systems_analyze
 *  @{ */
return_code analyze_state_space_gain(const state_space_T*, real_t*, system_time_domain);
return_code analyze_state_space_poles(const state_space_T*, vector_generic_T*);
return_code analyze_state_space_zeros(const state_space_T*, vector_generic_T*);
bool_t analyze_state_space_is_stable(const state_space_T*, system_time_domain);
/** @} */
#endif

#ifdef __cplusplus
}
#endif

#endif /* INC_SYSTEMS_ANALYZE_H_ */
