/**
 * @file model.h
 */
#ifndef MODEL_H_
#define MODEL_H_

#include "common_def.h"
#include "discrete_systems.h"
#include "continuous_systems.h"
#include "static_systems.h"
#include "signal_sources.h"
#include "signal_sinks.h"


/**
 * @addtogroup model Advanced models
 * @brief Interface for complex discrete and continuous models structures.
 * Systems can be interconnected and merged using @ref systems_interface to get a complex model structure.
 * To evaluate and solve this kind of models, specific interface is provided.
 *
 * # Implemented functionality
 * - states derivatives and output update functions wrappers for continuous models
 * - subsystem output update function
 * - step function for discrete models
 *
 * @{
 */

/**
 * Continuous model structure.
 * Contains linked list of subsystem's continuous systems
 */
typedef struct model_continuous {
	continuous_system_generic_T; ///< inherited continuous system interface
	list_t continuous_systems_list; ///< linked list of subsystem's continuous systems
} model_continuous_T;

/**
 * Discrete model structure.
 */
typedef struct model_discrete {
	discrete_system_generic_T; ///< inherited discrete system interface
	uint tick; ///< samples counter
	real_t sample_time; ///< sample time of model
} model_discrete_T;


#ifdef __cplusplus
extern "C" {
#endif

return_code subsystem_output_update(system_generic_T*, real_t, ODE_solver_step_t);
return_code model_discrete_init(model_discrete_T*, const system_generic_T*, real_t);

return_code model_continuous_init(model_continuous_T*,const system_generic_T*);
return_code model_continuous_deinit(model_continuous_T*);
const vector_generic_T* model_continuous_get_derivatives(model_continuous_T*, real_t, ODE_solver_step_t);
return_code model_continuous_update_outputs(model_continuous_T*, real_t, ODE_solver_step_t);


#ifdef __cplusplus
}
#endif


/** @} */
#endif
