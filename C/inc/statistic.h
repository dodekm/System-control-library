/**
 * @file statistic.h
 */
#ifndef INC_STATISTIC_H_
#define INC_STATISTIC_H_

#include "common_def.h"
#include "vector.h"

/** @addtogroup statistic Statistics
 *  @brief
 *
 * @{
 */


#ifdef __cplusplus
extern "C" {
#endif

return_code vector_real_max(const vector_generic_T*, real_t*, uint*);
return_code vector_real_min(const vector_generic_T*, real_t*, uint*);
return_code vector_real_mean(const vector_generic_T*, real_t*);
return_code vector_real_variance(const vector_generic_T*, real_t*);
return_code vector_real_standard_deviation(const vector_generic_T*, real_t*);
return_code vector_real_mean_std_dev(const vector_generic_T*, real_t*, real_t*);
return_code vector_real_RMS(const vector_generic_T*, real_t*);
return_code vector_real_RMSE(const vector_generic_T*,const vector_generic_T*, real_t*);
return_code vector_real_MAE(const vector_generic_T*,const vector_generic_T*, real_t*);

#ifdef __cplusplus
}
#endif


/** @} */
#endif /* INC_STATISTIC_H_ */
