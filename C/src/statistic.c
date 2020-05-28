
#include "statistic.h"

return_code vector_real_max(const vector_generic_T* vector, real_t* max_value_ptr, uint* max_idx_ptr) {
#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
#endif
	size_t length = vector_length(vector);
	real_t max_value = vector_type_at(vector, real_t, 0);
	uint max_idx = 0;
	for (uint i = 1; i < length; i++) {
		if (vector_type_at(vector,real_t,i) > max_value) {
			max_value = vector_type_at(vector, real_t, i);
			max_idx = i;
		}
	}
	if (max_value_ptr != NULL)
		*max_value_ptr = max_value;
	if (max_idx_ptr != NULL)
		*max_idx_ptr = max_idx;

	return return_OK;

}
return_code vector_real_min(const vector_generic_T* vector, real_t* min_value_ptr, uint* min_idx_ptr) {

#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
#endif
	size_t length = vector_length(vector);
	real_t min_value = vector_type_at(vector, real_t, 0);
	uint min_idx = 0;
	for (uint i = 1; i < length; i++) {
		if (vector_type_at(vector,real_t,i) < min_value) {
			min_value = vector_type_at(vector, real_t, i);
			min_idx = i;
		}
	}
	if (min_value_ptr != NULL)
		*min_value_ptr = min_value;
	if (min_idx_ptr != NULL)
		*min_idx_ptr = min_idx;

	return return_OK;
}
return_code vector_real_mean(const vector_generic_T* vector, real_t* mean_ptr) {
#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
	if (mean_ptr == NULL)
		return return_NULLPTR;
#endif
	real_t sum = 0;
	size_t length = vector_length(vector);
	for (uint i = 0; i < length; i++) {

		sum += vector_type_at(vector, real_t, i);
	}
	real_t mean = sum / length;
	*mean_ptr = mean;
	return return_OK;

}
return_code vector_real_variance(const vector_generic_T* vector, real_t* variance_ptr) {
#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
	if (variance_ptr == NULL)
		return return_NULLPTR;
#endif
	size_t length = vector_length(vector);
	real_t sum = 0;
	real_t sum_of_squares = 0;

	for (uint i = 0; i < length; i++) {

		real_t x = vector_type_at(vector, real_t, i);
		sum += x;
		sum_of_squares += POW2(x);
	}
	real_t variance = (sum_of_squares - POW2(sum) / (real_t) length) / ((real_t) (length - 1));

	*variance_ptr = variance;
	return return_OK;

}

return_code vector_real_standard_deviation(const vector_generic_T* vector, real_t* std_dev_ptr) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (std_dev_ptr == NULL)
		return return_NULLPTR;
#endif
	real_t variance = 0;
	returnval = vector_real_variance(vector, &variance);
	if (returnval != return_OK)
		return returnval;
	real_t std_dev = sqrt(variance);
	*std_dev_ptr = std_dev;
	return return_OK;
}

return_code vector_real_mean_std_dev(const vector_generic_T* vector, real_t* mean_ptr, real_t* std_dev_ptr) {
#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
#endif
	size_t length = vector_length(vector);
	real_t sum = 0;
	real_t sum_of_squares = 0;

	for (uint i = 0; i < length; i++) {

		real_t x = vector_type_at(vector, real_t, i);
		sum += x;
		sum_of_squares += POW2(x);
	}
	real_t mean = sum / (real_t) length;
	real_t variance = (sum_of_squares - POW2(sum) / (real_t) length) / ((real_t) (length - 1));
	real_t std_dev = sqrt(variance);
	if (mean_ptr != NULL)
		*mean_ptr = mean;
	if (std_dev_ptr != NULL)
		*std_dev_ptr = std_dev;
	return return_OK;
}

return_code vector_real_RMS(const vector_generic_T* vector, real_t* RMS_ptr) {
#ifdef ASSERT_NULLPTR
	return_code returnval = vector_assert(vector);
	if (returnval != return_OK)
		return returnval;
	if (RMS_ptr == NULL)
		return return_NULLPTR;
#endif
	size_t length = vector_length(vector);
	real_t sum_of_squares = 0;

	for (uint i = 0; i < length; i++) {
		real_t x = vector_type_at(vector, real_t, i);
		sum_of_squares += POW2(x);
	}
	real_t RMS = sqrt((sum_of_squares) / (real_t) (length));

	*RMS_ptr = RMS;
	return return_OK;
}

return_code vector_real_RMSE(const vector_generic_T* vector_A,const vector_generic_T* vector_B, real_t* RMSE_ptr) {
#ifdef ASSERT_NULLPTR
	return_code returnval = return_OK;
	returnval = vector_assert(vector_A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_B);
	if (returnval != return_OK)
		return returnval;
	if (RMSE_ptr == NULL)
		return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_A) != vector_length(vector_B))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_A);
	real_t sum_of_squared_error = 0;
	for (uint i = 0; i < length; i++) {
		real_t e = vector_type_at(vector_A, real_t, i) - vector_type_at(vector_B, real_t, i);
		sum_of_squared_error += POW2(e);
	}
	real_t RMSE = sqrt((sum_of_squared_error) / (real_t) (length));

	*RMSE_ptr = RMSE;
	return return_OK;

}

return_code vector_real_MAE(const vector_generic_T* vector_A,const vector_generic_T* vector_B, real_t* MAE_ptr) {
#ifdef ASSERT_NULLPTR
	return_code returnval = return_OK;
	returnval = vector_assert(vector_A);
	if (returnval != return_OK)
		return returnval;
	returnval = vector_assert(vector_B);
	if (returnval != return_OK)
		return returnval;
	if (MAE_ptr == NULL)
		return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (vector_length(vector_A) != vector_length(vector_B))
		return return_WRONG_DIMENSIONS;
#endif
	size_t length = vector_length(vector_A);
	real_t sum_of_absolute_error = 0;
	for (uint i = 0; i < length; i++) {
		real_t e = vector_type_at(vector_A, real_t, i) - vector_type_at(vector_B, real_t, i);
		sum_of_absolute_error += fabs(e);
	}
	real_t MAE = sum_of_absolute_error / (real_t) (length);

	*MAE_ptr = MAE;
	return return_OK;
}
