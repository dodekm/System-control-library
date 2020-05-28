#include "matrix.h"
#include "gsl_wrapper.h"

/**
 * Asserts/verifies matrix structure integrity.
 * Checks:
 * - data pointer
 * - allocation info
 * - zero dimensions
 * @param mat matrix to assert
 * @return system control library error code
 * - @ref return_NULLPTR if matrix struct or data pointer is NULL
 * - @ref return_ERROR if allocation type is unallocated.
 * - @ref return_WRONG_DIMENSIONS if n_rows or n_cols is 0
 * - else @ref return_OK
 */

return_code matrix_assert(const matrix_T* mat) {
	if (mat == NULL)
		return return_NULLPTR;
	if (mat->data_ptr == NULL)
		return return_NULLPTR;
	if (mat->allocation_info == allocation_type_unallocated)
		return return_ERROR;
	if (mat->n_cols == 0 || mat->n_rows == 0)
		return return_WRONG_DIMENSIONS;
	return return_OK;
}

/**
 * Gets matrix indexed element value using safety restrictions.
 * Asserts matrix and checks indexes according to dimensions
 *
 * @param mat matrix pointer
 * @param row element row index
 * @param col element column index
 * @param value value to set
 * @return return_INDEX_OUT_OF_RANGE if indexes are out of matrix dimensions
 */

return_code matrix_set_value_safe(matrix_T* mat, uint row, uint col, real_t value) {

	return_code returnval = matrix_assert(mat);
	if (returnval != return_OK)
		return returnval;
	if (row >= mat->n_rows || col >= mat->n_cols)
		return return_INDEX_OUT_OF_RANGE;
	matrix_at(mat,row,col)=value;
	return return_OK;
}

/**
 * Gets matrix indexed element value using safety restrictions.
 * Asserts matrix and checks indexes according to dimensions
 *
 * @param mat matrix pointer
 * @param row element row index
 * @param col element column index
 * @param value_ptr value destination pointer
 * @return return_INDEX_OUT_OF_RANGE if indexes are out of matrix dimensions
 */
return_code matrix_get_value_safe(const matrix_T* mat, uint row, uint col, real_t* value_ptr) {
	if (value_ptr == NULL)
		return return_NULLPTR;
	return_code returnval = matrix_assert(mat);
	if (returnval != return_OK)
		return returnval;
	if (row >= mat->n_rows || col >= mat->n_cols)
		return return_INDEX_OUT_OF_RANGE;
	*value_ptr = matrix_at(mat, row, col);
	return return_OK;
}

/**
 * Initializes matrix structure with static or dynamic data.
 * Sets matrix dimensions and pointer to data
 * @param mat matrix to initialize
 * @param n_rows number of matrix rows
 * @param n_cols number of matrix columns
 * @param data_ptr pointer to matrix data if static allocation or NULL if dynamic allocation
 * @return system control library error code
 */
return_code matrix_init(matrix_T* mat, size_t n_rows, size_t n_cols, real_t* data_ptr) {

#ifdef ASSERT_NULLPTR
	if (mat == NULL)
		return return_NULLPTR;
#endif

#ifdef ASSERT_DIMENSIONS
	if (n_rows == 0 || n_cols == 0)
		return return_WRONG_DIMENSIONS;
#endif

	if (mat->allocation_info != allocation_type_unallocated)
		return return_ERROR;
	if (data_ptr == NULL) {
		data_ptr = (real_t*) malloc(n_rows * n_cols * sizeof(real_t));
		if (data_ptr == NULL)
			return return_NULLPTR;
		memset(data_ptr, 0, n_rows * n_cols * sizeof(real_t));
		mat->allocation_info = allocation_type_dynamic;
	} else {
		mat->allocation_info = allocation_type_static;
	}
	mat->data_ptr = data_ptr;
	mat->n_rows = n_rows;
	mat->n_cols = n_cols;
	mat->n_cols_mem = n_cols;
	return return_OK;

}
/**
 * De-initializes matrix structure.
 * Asserts matrix first.
 * If dynamic allocated - frees memory.
 * @param mat matrix to de-initialize
 * @return system control library error code
 */

return_code matrix_deinit(matrix_T* mat) {
#ifdef ASSERT_NULLPTR
	return_code returnval = matrix_assert(mat);
	if (returnval != return_OK)
		return returnval;
#endif
	if (mat->allocation_info == allocation_type_dynamic) {
		free(mat->data_ptr);
	}
	mat->data_ptr = NULL;
	mat->allocation_info = allocation_type_unallocated;
	mat->n_rows = 0;
	mat->n_cols = 0;
	mat->n_cols_mem = 0;
	return return_OK;

}
/**
 * Deep matrix copy.
 * Copy matrix dimensions and loads data.
 * If not equal dimensions - performs de-initialization and dynamic initialization
 * @param matDst destination matrix
 * @param matSrc source matrix
 * @return system control library error code
 */

return_code matrix_copy(matrix_T* matDst, const matrix_T* matSrc) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = matrix_assert(matSrc);
	if (returnval != return_OK)
		return returnval;
#endif
	if (matrix_assert(matDst) != return_OK || matSrc->n_cols != matDst->n_cols || matSrc->n_rows != matDst->n_rows) {
		matrix_deinit(matDst);
		returnval = matrix_init(matDst, matSrc->n_rows, matSrc->n_cols, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (matSrc->n_cols != matDst->n_cols || matSrc->n_rows != matDst->n_rows)
		return return_WRONG_DIMENSIONS;
#endif
	return matrix_load(matDst, matSrc->data_ptr);

}
/**
 * Sets all matrix elements to certain value.
 * @param mat destination matrix
 * @param value real number value
 * @return system control library error code
 */
return_code matrix_set_all_elements(matrix_T* mat, real_t value) {

#ifdef ASSERT_NULLPTR
	return_code returnval = matrix_assert(mat);
	if (returnval != return_OK)
		return returnval;
#endif

	for (uint i = 0; i < mat->n_rows; i++) {
		for (uint j = 0; j < mat->n_cols; j++) {
			matrix_at(mat, i, j)= value;
		}
	}
	return return_OK;

}
/**
 * Loads matrix elements data according to dimensions.
 * @param matDst destination matrix
 * @param data_ptr source data pointer
 * @return system control library error code
 */

return_code matrix_load(matrix_T* matDst, const real_t* data_ptr) {

#ifdef ASSERT_NULLPTR
	return_code returnval = matrix_assert(matDst);
	if (returnval != return_OK)
		return returnval;
	if (data_ptr == NULL)
		return return_NULLPTR;
#endif

	memcpy((void*) matDst->data_ptr, (const void*) data_ptr, matDst->n_rows * matDst->n_cols * sizeof(real_t));
	return return_OK;

}
/**
 * Selects sumbatrix from matrix.
 * Submatrix is set as static allocated - does not own data.
 * @param matDst submatrix pointer
 * @param matSrc source matrix pointer
 * @param n_rows submatrix number of rows
 * @param n_cols submatrix number of columns
 * @param row_offset submatrix row offset/start index
 * @param col_offset submatrix column offset/start index
 * @return system control library error code
 */

return_code matrix_submatrix(matrix_T* matDst, const matrix_T* matSrc, size_t n_rows, size_t n_cols, size_t row_offset, size_t col_offset) {
#ifdef ASSERT_NULLPTR
	return_code returnval = matrix_assert(matSrc);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (n_rows + row_offset > matSrc->n_rows)
		return return_WRONG_DIMENSIONS;
	if (n_cols + col_offset > matSrc->n_cols)
		return return_WRONG_DIMENSIONS;
#endif

	matrix_init(matDst, n_rows, n_cols, matrix_ptr_at(matSrc, row_offset, col_offset));
	matDst->n_cols_mem = matSrc->n_cols_mem;

	return return_OK;
}
/**
 * Sets matrix as identity/eye matrix.
 * Writes 1 to diagonal and 0 elsewhere.
 * Matrix must be square
 * @param mat matrix pointer
 * @return system control library error code
 */

return_code matrix_set_identity(matrix_T* mat) {
#ifdef ASSERT_NULLPTR
	return_code returnval = matrix_assert(mat);
	if (returnval != return_OK)
		return returnval;
#endif
#ifdef ASSERT_DIMENSIONS
	if (!matrix_is_square(mat))
		return return_WRONG_DIMENSIONS;
#endif
	for (uint i = 0; i < mat->n_rows; i++) {
		for (uint j = 0; j < mat->n_cols; j++) {
			if (i == j)
				matrix_at(mat, i, j)=1;
				else
				matrix_at(mat, i, j)=0;
			}
		}
	return return_OK;
}
/**
 * Multiplies two matrices.
 * Standard matrix multiplication C=A*B.
 * Asserts all matrices and dimensions first.
 * Matrix dimensions conditions are following:
 * 	- A->n cols == B->n rows
 *  - C->n_rows == A->n_rows
 *  - C->n_cols == B->n_cols
 * Cannot multiply matrices in place! Destination matrix C must be different from A and B.
 * Function has GSL/BLAS support enabled by @ref USE_GSL .
 * @param matA matrix A pointer
 * @param matB matrix B pointer
 * @param matDst matrix C pointer
 * @return system control library error code
 */
return_code matrix_multiply(const matrix_T* matA, const matrix_T* matB, matrix_T* matDst) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = matrix_assert(matA);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(matB);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(matDst);
	if (returnval != return_OK) {
		returnval = matrix_init(matDst, matA->n_rows, matB->n_cols, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#endif
#ifdef ASSERT_DIMENSIONS
	if (matA->n_cols != matB->n_rows)
		return return_WRONG_DIMENSIONS;
	if (matDst->n_rows != matA->n_rows || matDst->n_cols != matB->n_cols)
		return return_WRONG_DIMENSIONS;
#endif

	if (matA == matDst || matB == matDst)
		return return_ERROR;

#ifdef USE_GSL
	gsl_matrix A_gsl = matrix_to_gsl_matrix(matA);
	gsl_matrix B_gsl = matrix_to_gsl_matrix(matB);
	gsl_matrix C_gsl = matrix_to_gsl_matrix(matDst);
	return gsl_error_code_to_return_code(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &A_gsl, &B_gsl, 0.0, &C_gsl));

#else
	for (uint i = 0; i < matDst->n_rows; i++) {
		for (uint j = 0; j < matDst->n_cols; j++) {
			real_t sum = 0;
			for (uint n = 0; n < matA->n_cols; n++) {
				sum += (matrix_at(matA, i, n)) * (matrix_at(matB, n, j));
			}
			matrix_at(matDst, i, j)= sum;
		}
	}
	return return_OK;
#endif

}
/**
 * Function performs matrix-matrix element wise operation.
 * Element of destination matrix C equals:  C(i,j)=f(A(i,j),B(i,j)).
 * Function can perform operations in place - destination matrix C can be same as A or B matrix.
 * All matrices must have equal dimensions.
 * @param matA matrix A pointer
 * @param matB matrix B pointer
 * @param matDst matrix C pointer
 * @param operator  binary function/operator pointer f()
 * @return system control library error code
 */
return_code matrix_element_wise_operation(const matrix_T* matA, const matrix_T* matB, matrix_T* matDst, real_binary_operator_T operator) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (operator == NULL)
		return return_NULLPTR;
	returnval = matrix_assert(matA);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(matB);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(matDst);
	if (returnval != return_OK) {
		returnval = matrix_init(matDst, matA->n_rows, matA->n_cols, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#endif
#ifdef ASSERT_DIMENSIONS
	if (!matrix_equal_dimensions(matA, matDst) || !matrix_equal_dimensions(matB, matDst))
		return return_WRONG_DIMENSIONS;
#endif

	for (uint i = 0; i < matDst->n_rows; i++) {
		for (uint j = 0; j < matDst->n_cols; j++) {
			matrix_at(matDst, i, j)= operator(matrix_at(matA, i, j), matrix_at(matB, i, j));
		}
	}
	return return_OK;
}

/**
 * Function performs matrix-scalar element wise operation.
 * Element of destination matrix C equals:   C(i,j)=f(A(i,j),S).
 * Function can perform operations in place - destination matrix C can be same as A or B matrix.
 * Source and destination matrix must have equal dimensions.
 * @param matSrc matrix A pointer
 * @param matDst matrix C pointer
 * @param scalar_value scalar value argument S
 * @param operator binary function/operator pointer f()
 * @return system control library error code
 */

return_code matrix_scalar_operation(const matrix_T* matSrc, matrix_T* matDst, real_t scalar_value, real_binary_operator_T operator) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (operator == NULL)
		return return_NULLPTR;
	returnval = matrix_assert(matSrc);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(matDst);
	if (returnval != return_OK) {
		returnval = matrix_init(matDst, matSrc->n_rows, matSrc->n_cols, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#endif

#ifdef ASSERT_DIMENSIONS
	if (!matrix_equal_dimensions(matSrc, matDst))
		return return_WRONG_DIMENSIONS;
#endif
	for (uint i = 0; i < matDst->n_rows; i++) {
		for (uint j = 0; j < matDst->n_cols; j++) {
			matrix_at(matDst, i, j)= operator(matrix_at(matSrc, i, j), scalar_value);
		}
	}
	return return_OK;
}

/**
 * Function performs matrix-matrix-scalar element wise operation.
 * Element of destination matrix C equals:  C(i,j)=f(B(i,j),g(A(i,j),S)).
 * All matrices must have equal dimensions.
 * Function can perform operations in place - destination matrix C can be same as A or B matrix
 * @param matA matrix A pointer
 * @param matB matrix B pointer
 * @param matDst matrix C pointer
 * @param scalar_value scalar value argument S
 * @param operator_element_wise  binary function/operator pointer f()
 * @param operator_scalar  binary function/operator pointer g()
 * @return system control library error code
 */

return_code matrix_element_wise_operation_and_scalar_operation(const matrix_T* matA, const matrix_T* matB, matrix_T* matDst, real_t scalar_value, real_binary_operator_T operator_element_wise, real_binary_operator_T operator_scalar) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = matrix_assert(matA);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(matB);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(matDst);
	if (returnval != return_OK) {
		returnval = matrix_init(matDst, matA->n_rows, matA->n_cols, NULL);
		if (returnval != return_OK)
			return returnval;
	}
	if (operator_element_wise == NULL)
		return return_NULLPTR;
	if (operator_scalar == NULL)
		return return_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (!matrix_equal_dimensions(matA, matDst) || !matrix_equal_dimensions(matB, matDst))
		return return_WRONG_DIMENSIONS;
#endif

	for (uint i = 0; i < matDst->n_rows; i++) {
		for (uint j = 0; j < matDst->n_cols; j++) {
			matrix_at(matDst, i, j)= operator_element_wise(matrix_at(matB, i, j),operator_scalar(matrix_at(matA, i, j), scalar_value));
		}
	}
	return return_OK;

}
/**
 * Performs scalar operation on matrix diagonal - indexes (i,i).
 * Non-diagonal matrix elements are untouched.
 * Matrix does not have to be square.
 * Source and destination matrix must have equal dimensions.
 * Function can perform operations in place - destination matrix can be same as source matrix.
 * @param matSrc source matrix pointer
 * @param matDst destination matrix pointer
 * @param scalar_value scalar value to process with operator function f()
 * @param operator binary function/operator pointer f()
 * @return system control library error code
 */

return_code matrix_diagonal_operation(const matrix_T* matSrc, matrix_T* matDst, real_t scalar_value, real_binary_operator_T operator) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	if (operator == NULL)
		return return_NULLPTR;
	returnval = matrix_assert(matSrc);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(matDst);
	if (returnval != return_OK) {
		returnval = matrix_init(matDst, matSrc->n_rows, matSrc->n_cols, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#endif

#ifdef ASSERT_DIMENSIONS
	if (!matrix_equal_dimensions(matSrc, matDst))
		return return_WRONG_DIMENSIONS;
#endif

	uint min_dim = MIN(matSrc->n_rows, matSrc->n_cols);
	for (uint i = 0; i < min_dim; i++) {
		matrix_at(matDst, i, i)= operator(matrix_at(matSrc, i, i), scalar_value);
	}
	return return_OK;

}
/**
 * Transposes mXn to nXm matrix in place.
 * Performs temporal matrix dynamic copy an transposition
 * @param mat matrix to transpose
 * @return system control library error code
 */

return_code matrix_transpose(matrix_T* mat) {
	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = matrix_assert(mat);
	if (returnval != return_OK)
		return returnval;
#endif

	matrix_T mat_orig = { 0 };
	returnval = matrix_copy(&mat_orig, mat);
	if (returnval != return_OK)
		return returnval;
	mat->n_rows = mat->n_cols;
	mat->n_cols = mat->n_rows;

	returnval = matrix_transpose_copy(&mat_orig, mat);
	matrix_deinit(&mat_orig);
	return returnval;
}
/**
 * Transposes mXn matrix into other nXm matrix.
 * Function can not transpose in place - matSrc and matDst must not be same matrices
 * If destination matrix is un-initialized function performs dynamic  initialization
 * if is already initialized, function asserts destination matrix dimensions
 * @param matSrc matrix to transpose
 * @param matDst transposed matrix - can be preallocated
 * @return system control library error code
 */

return_code matrix_transpose_copy(const matrix_T* matSrc, matrix_T* matDst) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = matrix_assert(matSrc);
	if (returnval != return_OK)
		return returnval;
#endif
	if (matrix_assert(matDst) != return_OK) {
		returnval = matrix_init(matDst, matSrc->n_cols, matSrc->n_rows, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#ifdef ASSERT_DIMENSIONS
	if (matDst->n_rows != matSrc->n_cols || matDst->n_cols != matSrc->n_rows)
		return return_WRONG_DIMENSIONS;
#endif

	for (uint i = 0; i < matDst->n_rows; i++) {
		for (uint j = 0; j < matDst->n_cols; j++) {
			matrix_at(matDst, i, j)= matrix_at(matSrc, j, i);
		}
	}
	return return_OK;
}

/**
 * Power of matrix.
 * Power of square matrix performed by matrix multiplication
 * Function internally uses temporal dynamically allocated matix
 * @param mat_base  source - base matrix
 * @param mat_power destination - matrix power
 * @param power degree of power
 * @return system control library error code
 */
return_code matrix_power(const matrix_T* mat_base, matrix_T* mat_power, uint power) {

	return_code returnval = return_OK;
#ifdef ASSERT_NULLPTR
	returnval = matrix_assert(mat_base);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(mat_power);
	if (returnval != return_OK) {
		returnval = matrix_init(mat_power, mat_base->n_rows, mat_base->n_cols, NULL);
		if (returnval != return_OK)
			return returnval;
	}

#endif
#ifdef ASSERT_DIMENSIONS
	if (!matrix_equal_dimensions(mat_base, mat_power))
		return return_WRONG_DIMENSIONS;
	if (!matrix_is_square(mat_base))
		return return_WRONG_DIMENSIONS;
#endif
	matrix_T mat_power_i = { 0 };

	returnval = matrix_init(&mat_power_i, mat_base->n_rows, mat_base->n_cols, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_set_identity(&mat_power_i);
	if (returnval != return_OK)
		goto ret;
	returnval = matrix_set_identity(mat_power);
	if (returnval != return_OK)
		goto ret;

	for (uint i = 0; i < power; i++) {
		returnval = matrix_multiply(&mat_power_i, mat_base, mat_power);
		if (returnval != return_OK)
			goto ret;
		returnval = matrix_copy(&mat_power_i, mat_power);
		if (returnval != return_OK)
			goto ret;
	}
	ret: matrix_deinit(&mat_power_i);
	return returnval;
}

/**
 * Matrix exponential.
 * Uses Taylor series expansion of exponential function
 * Input base matrix and exponential matrix must be square
 * Function internally uses temporal dynamically allocated matrices
 * @param mat_base source - base matrix
 * @param mat_exp destination - matrix exponential
 * @param series_order Taylor series order
 * @return system control library error code
 */

return_code matrix_exp(const matrix_T* mat_base, matrix_T* mat_exp, uint series_order) {
	return_code returnval = return_OK;
	if (series_order > 20)
		return return_ERROR;
#ifdef ASSERT_NULLPTR
	returnval = matrix_assert(mat_base);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_assert(mat_exp);
	if (returnval != return_OK) {
		returnval = matrix_init(mat_exp, mat_base->n_rows, mat_base->n_cols, NULL);
		if (returnval != return_OK)
			return returnval;
	}
#endif
#ifdef ASSERT_DIMENSIONS
	if (!matrix_equal_dimensions(mat_base, mat_exp))
		return return_WRONG_DIMENSIONS;
	if (!matrix_is_square(mat_base))
		return return_WRONG_DIMENSIONS;
#endif

	matrix_T mat_power_i_0 = { 0 };
	matrix_T mat_power_i_1 = { 0 };
	uint64_t i_factorial = 1;
	returnval = matrix_init(&mat_power_i_0, mat_base->n_rows, mat_base->n_cols, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_init(&mat_power_i_1, mat_base->n_rows, mat_base->n_cols, NULL);
	if (returnval != return_OK)
		return returnval;
	returnval = matrix_set_identity(&mat_power_i_0);
	if (returnval != return_OK)
		returnval = matrix_set_identity(&mat_power_i_1);
	if (returnval != return_OK)
		goto ret;
	returnval = matrix_set_all_elements(mat_exp, 0);
	if (returnval != return_OK)
		goto ret;

	for (uint i = 0; i < series_order; i++) {
		returnval = matrix_multiply(&mat_power_i_0, mat_base, &mat_power_i_1);
		if (returnval != return_OK)
			goto ret;
		returnval = matrix_copy(&mat_power_i_0, &mat_power_i_1);
		if (returnval != return_OK)
			goto ret;
		returnval = matrix_element_wise_operation_and_scalar_operation(&mat_power_i_1, mat_exp, mat_exp, (real_t) i_factorial, real_sum, real_div);
		if (returnval != return_OK)
			goto ret;
		i_factorial *= (i + 1);
	}
	ret: matrix_deinit(&mat_power_i_0);
	matrix_deinit(&mat_power_i_1);
	return returnval;

}

