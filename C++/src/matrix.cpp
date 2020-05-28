#include "matrix.h"
#include "wrappers.h"

using namespace SystemControl;
using namespace Convert;

void Matrix::assert() const throw (exception_code) {
#ifdef ASSERT_NULLPTR
	if (data_ptr == NULL)
		throw exception_NULLPTR;
#endif
#ifdef ASSERT_DIMENSIONS
	if (n_cols == 0 || n_rows == 0)
		throw exception_WRONG_DIMENSIONS;
#endif
	if (allocation_info == allocation_type_unallocated)
		throw exception_ERROR;

}

void Matrix::init(size_t n_rows, size_t n_cols, real_t* data_ptr) throw (exception_code) {

	if (n_cols == 0 || n_rows == 0)
		throw exception_WRONG_DIMENSIONS;
	if (allocation_info != allocation_type_unallocated)
		throw exception_ERROR;
	if (data_ptr == NULL) {
		data_ptr = (real_t*) malloc(n_rows * n_cols * sizeof(real_t));
		if (data_ptr == NULL)
			throw exception_NULLPTR;
		memset(data_ptr, 0, n_rows * n_cols * sizeof(real_t));
		allocation_info = allocation_type_dynamic;
	} else {
		allocation_info = allocation_type_static;
	}
	this->data_ptr = data_ptr;
	this->n_rows = n_rows;
	this->n_cols = n_cols;
	this->n_cols_mem = n_cols;

}

void Matrix::deinit() {

	try {
		assert();
	} catch (...) {
		return;
	}
	if (allocation_info == allocation_type_dynamic) {
		free(data_ptr);
	}
	data_ptr = NULL;
	allocation_info = allocation_type_unallocated;
	n_rows = 0;
	n_cols = 0;
	n_cols_mem = 0;

}

Matrix::Matrix(size_t n_rows, size_t n_cols, real_t* data_ptr) throw (exception_code) {
	init(n_rows, n_cols, data_ptr);

}

Matrix::Matrix(size_t n_rows, size_t n_cols, const real_t* data_ptr) throw (exception_code) :
		Matrix(n_rows, n_cols, const_cast<real_t*>(data_ptr)) {
}

Matrix::~Matrix() {
	deinit();
}

Matrix::Matrix(const Matrix& matSrc, bool copy_data) throw (exception_code) :
		Matrix(matSrc.n_rows, matSrc.n_cols) {
	if (copy_data)
		load_data(matSrc.data_ptr);
}
#ifdef USE_GSL
Matrix::Matrix(const gsl_matrix& gsl_mat) throw (exception_code) {
	n_rows = gsl_mat.size1;
	n_cols = gsl_mat.size2;
	n_cols_mem = gsl_mat.tda;
	allocation_info = allocation_type_static;
	data_ptr = gsl_mat.data;
	assert();
}
#endif

Matrix Matrix::submatrix(size_t n_rows, size_t n_cols, size_t row_offset, size_t col_offset) throw (exception_code) {
	assert();
#ifdef ASSERT_DIMENSIONS
	if (n_rows + row_offset > this->n_rows)
		throw exception_WRONG_DIMENSIONS;
	if (n_cols + col_offset > this->n_cols)
		throw exception_WRONG_DIMENSIONS;
#endif
	Matrix mat(n_rows, n_cols, ptr_at(row_offset, col_offset));
	mat.n_cols_mem = n_cols_mem;
	return mat;

}
Matrix Matrix::row(size_t row_number) throw (exception_code) {
	return submatrix(1, n_cols, row_number, 0);
}
Matrix Matrix::column(size_t column_number) throw (exception_code) {
	return submatrix(n_rows, 1, 0, column_number);
}

Matrix Matrix::diagonal() throw (exception_code) {
	assert();
	Matrix mat(MIN(n_rows, n_cols), 1, data_ptr);
	mat.n_cols_mem = n_cols_mem + 1;
	return mat;
}

#ifdef USE_GSL
gsl_matrix Matrix::to_gsl_matrix() throw (exception_code) {
	assert();
	return gsl_matrix_view_array_with_tda((double*) data_ptr, n_rows, n_cols, n_cols_mem).matrix;
}
const gsl_matrix Matrix::to_gsl_matrix() const throw (exception_code) {
	assert();
	return gsl_matrix_const_view_array_with_tda((const double*) data_ptr, n_rows, n_cols, n_cols_mem).matrix;
}

gsl_matrix* Matrix::to_gsl_matrix_dynamic_copy() const throw (exception_code) {
	const gsl_matrix gsl_mat_src = to_gsl_matrix();
	gsl_matrix* gsl_mat_dst = gsl_matrix_alloc(n_rows, n_cols);
	if (gsl_mat_dst == NULL)
	throw exception_NULLPTR;
	gsl_matrix_memcpy(gsl_mat_dst, &gsl_mat_src);
	return gsl_mat_dst;
}

#endif

real_t& Matrix::at_safe(uint row, uint col) throw (exception_code) {
	assert();
	if (row >= n_rows || col >= n_cols)
		throw exception_INDEX_OUT_OF_RANGE;
	return this->at(row, col);

}

real_t Matrix::at_safe(uint row, uint col) const throw (exception_code) {
	assert();
	if (row >= n_rows || col >= n_cols)
		throw exception_INDEX_OUT_OF_RANGE;
	return this->at(row, col);

}

template<typename F>
void Matrix::for_each(F lambda) throw (exception_code) {
	assert();
	for (uint i = 0; i < n_rows; i++) {
		for (uint j = 0; j < n_cols; j++) {
			if (!lambda(at(i, j), i, j))
				return;
		}
	}

}

template<typename F>
void Matrix::for_each(F lambda) const throw (exception_code) {
	Matrix& mat = const_cast<Matrix&>(*this);
	mat.for_each(lambda);
}

template<typename F>
void Matrix::for_each(const Matrix& matSrc, F lambda) throw (exception_code) {

	Matrix& matDst = *this;
	matSrc.assert();
	matDst.assert();

#ifdef ASSERT_DIMENSIONS
	if (!matDst.equal_dimensions(matSrc))
		throw exception_WRONG_DIMENSIONS;
#endif
	for (uint i = 0; i < n_rows; i++) {
		for (uint j = 0; j < n_cols; j++) {
			if (!lambda(matDst.at(i, j), matSrc.at(i, j), i, j))
				return;
		}
	}

}

template<typename F>
void Matrix::for_each(const Matrix& matSrcB, F lambda) const throw (exception_code) {
	Matrix& matSrcA = const_cast<Matrix&>(*this);
	matSrcA.for_each(matSrcB, lambda);
}

template<typename F>
void Matrix::for_diagonal(const Matrix& matSrc, F lambda) throw (exception_code) {
	Matrix& matDst = *this;
	matDst.assert();
	matSrc.assert();

#ifdef ASSERT_DIMENSIONS
	if (!matDst.equal_dimensions(matSrc))
		throw exception_WRONG_DIMENSIONS;
#endif

	uint min_dim = MIN(n_rows, n_cols);
	for (uint i = 0; i < min_dim; i++) {
		if (!lambda(matDst.at(i, i), matSrc.at(i, i)))
			return;
	}
}
template<typename F>
void Matrix::for_each(const Matrix& matSrcA, const Matrix& matSrcB, F lambda) throw (exception_code) {
	Matrix& matDst = *this;

	matSrcA.assert();
	matSrcB.assert();
	matDst.assert();

#ifdef ASSERT_DIMENSIONS
	if (!matDst.equal_dimensions(matSrcA) || !matDst.equal_dimensions(matSrcB))
		throw exception_WRONG_DIMENSIONS;
#endif

	for (uint i = 0; i < n_rows; i++) {
		for (uint j = 0; j < n_cols; j++) {
			if (!lambda(matSrcA.at(i, j), matSrcB.at(i, j), matDst.at(i, j), i, j))
				return;
		}
	}
}

void Matrix::set_all_elements(real_t value) throw (exception_code) {
	auto lambda = [value](auto& M_i,auto i,auto j)->auto {M_i=value; return true;};
	for_each(lambda);

}

void Matrix::set_identity() throw (exception_code) {

	auto lambda = [](auto& M_i,auto i,auto j)->auto
	{
		if (i == j)
		M_i = 1.0;
		else
		M_i = 0;
		return true;
	};
	for_each(lambda);
}

void Matrix::load_data(const real_t* data_ptr) throw (exception_code) {
	assert();
#ifdef ASSERT_NULLPTR
	if (data_ptr == NULL)
		throw exception_NULLPTR;
#endif
	memcpy((void*) this->data_ptr, (const void*) data_ptr, n_rows * n_cols * sizeof(real_t));

}

Matrix Matrix::transpose()const throw (exception_code) {

	assert();
	Matrix mat_T(n_cols, n_rows);
	for (uint i = 0; i < mat_T.n_rows; i++) {
		for (uint j = 0; j < mat_T.n_cols; j++) {
			mat_T.at(i, j) = at(j, i);
		}
	}
	return mat_T;
}

void Matrix::transpose()throw (exception_code)
{
	assert();
	const Matrix mat=*this;
	Matrix mat_T=mat.transpose();
	*this=mat_T;
}

Matrix& Matrix::multiply(const Matrix& matA, const Matrix& matB) throw (exception_code) {

	if (this == &matA || this == &matB)
		throw exception_ERROR;

	matA.assert();
	matB.assert();

	try {
		assert();
	} catch (...) {
		*this = Matrix(matA.n_rows, matB.n_cols);
	}

	Matrix& matDst = *this;

#ifdef ASSERT_DIMENSIONS
	if (matA.n_cols != matB.n_rows)
		throw exception_WRONG_DIMENSIONS;
	if (matDst.n_rows != matA.n_rows || matDst.n_cols != matB.n_cols)
		throw exception_WRONG_DIMENSIONS;
#endif

#ifdef USE_GSL
	gsl_matrix A_gsl = matA.to_gsl_matrix();
	gsl_matrix B_gsl = matB.to_gsl_matrix();
	gsl_matrix C_gsl = matDst.to_gsl_matrix();
	exception_code returnval = gsl_error_code_to_return_code(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &A_gsl, &B_gsl, 0.0, &C_gsl));
	if (returnval != exception_OK)
	throw returnval;
#else
	for (uint i = 0; i < n_rows; i++) {
		for (uint j = 0; j < n_cols; j++) {
			real_t sum = 0;
			for (uint n = 0; n < matA.n_cols; n++) {
				sum += matA.at(i, n) * matB.at(n, j);
			}
			matDst.at(i, j) = sum;
		}
	}

#endif

	return *this;
}

Matrix Matrix::multiply(const Matrix& matB) const throw (exception_code) {

	const Matrix& matA = *this;
	Matrix matDst(matA.n_rows, matB.n_cols);
	matDst.multiply(matA, matB);
	return matDst;
}

Matrix& Matrix::multiply_by_scalar_and_accumulate(const Matrix& matSrc, real_t scalar_value) throw (exception_code) {

	Matrix& matDst = *this;
	auto lambda = [scalar_value](auto& Dst_i,auto Src_i,auto i,auto j)->auto {Dst_i+=(Src_i*scalar_value);return true;};
	matDst.for_each(matSrc, lambda);
	return *this;
}

Matrix& Matrix::operator=(const Matrix& matSrc) throw (exception_code) {
	matSrc.assert();
	if (!equal_dimensions(matSrc)) {
		deinit();
		init(matSrc.n_rows, matSrc.n_cols);
	}
	load_data(matSrc.data_ptr);
	return *this;
}

Matrix& Matrix::operator=(real_t scalar_value) throw (exception_code) {

	set_all_elements(scalar_value);
	return *this;
}

Matrix& Matrix::element_add(const Matrix& A, const Matrix& B) throw (exception_code) {

	auto lambda = [](auto A_i,auto B_i,auto& C_i,auto i,auto j)->auto {C_i=A_i+B_i; return true;};
	for_each(A, B, lambda);
	return *this;
}
Matrix& Matrix::element_add(const Matrix& B, real_t value) throw (exception_code) {

	auto lambda = [value](auto& A_i,auto B_i,auto i,auto j)->auto {A_i=B_i+value;return true;};
	for_each(B, lambda);
	return *this;
}

Matrix& Matrix::element_sub(const Matrix& A, const Matrix& B) throw (exception_code) {

	auto lambda = [](auto A_i,auto B_i,auto& C_i,auto i,auto j)->auto {C_i=A_i-B_i; return true;};
	for_each(A, B, lambda);
	return *this;
}
Matrix& Matrix::element_sub(const Matrix& B, real_t value) throw (exception_code) {

	auto lambda = [value](auto& A_i,auto B_i,auto i,auto j)->auto {A_i=B_i-value;return true;};
	for_each(B, lambda);
	return *this;
}

Matrix& Matrix::element_mul(const Matrix& A, const Matrix& B) throw (exception_code) {

	auto lambda = [](auto A_i,auto B_i,auto& C_i,auto i,auto j)->auto {C_i=A_i*B_i; return true;};
	for_each(A, B, lambda);
	return *this;
}
Matrix& Matrix::element_mul(const Matrix& B, real_t value) throw (exception_code) {

	auto lambda = [value](auto& A_i,auto B_i,auto i,auto j)->auto {A_i=B_i*value;return true;};
	for_each(B, lambda);
	return *this;
}

Matrix& Matrix::element_div(const Matrix& A, const Matrix& B) throw (exception_code) {

	auto lambda = [](auto A_i,auto B_i,auto& C_i,auto i,auto j)->auto {C_i=A_i/B_i; return true;};
	for_each(A, B, lambda);
	return *this;
}
Matrix& Matrix::element_div(const Matrix& B, real_t value) throw (exception_code) {

	auto lambda = [value](auto& A_i,auto B_i,auto i,auto j)->auto {A_i=B_i/value;return true;};
	for_each(B, lambda);
	return *this;
}

Matrix Matrix::operator+(const Matrix& matB) const throw (exception_code) {
	const Matrix& matA = *this;
	Matrix matDst(matA.n_rows, matA.n_cols);
	matDst.element_add(matDst, matB);
	return matDst;
}

Matrix Matrix::operator-(const Matrix& matB) const throw (exception_code) {
	const Matrix& matA = *this;
	Matrix matDst(matA.n_rows, matA.n_cols);
	matDst.element_sub(matDst, matB);
	return matDst;
}

Matrix Matrix::operator*(const Matrix& matB) const throw (exception_code) {
	return multiply(matB);
}

Matrix Matrix::operator+(real_t scalar_value) const throw (exception_code) {
	const Matrix& matA = *this;
	Matrix matDst(matA.n_rows, matA.n_cols);
	matDst.element_add(matDst, scalar_value);
	return matDst;
}

Matrix Matrix::operator-(real_t scalar_value) const throw (exception_code) {
	const Matrix& matA = *this;
	Matrix matDst(matA.n_rows, matA.n_cols);
	matDst.element_sub(matDst, scalar_value);
	return matDst;
}
Matrix Matrix::operator*(real_t scalar_value) const throw (exception_code) {
	const Matrix& matA = *this;
	Matrix matDst(matA.n_rows, matA.n_cols);
	matDst.element_mul(matDst, scalar_value);
	return matDst;
}

Matrix& Matrix::operator+=(real_t scalar_value) throw (exception_code) {

	element_add(*this, scalar_value);
	return *this;
}

Matrix& Matrix::operator-=(real_t scalar_value) throw (exception_code) {

	element_sub(*this, scalar_value);
	return *this;
}

Matrix& Matrix::operator*=(real_t scalar_value) throw (exception_code) {

	element_mul(*this, scalar_value);
	return *this;
}

Matrix& Matrix::operator/=(real_t scalar_value) throw (exception_code) {

	element_div(*this, scalar_value);
	return *this;
}

Matrix& Matrix::operator+=(const Matrix& matB) throw (exception_code) {

	element_add(*this, matB);
	return *this;
}

Matrix& Matrix::operator-=(const Matrix& matB) throw (exception_code) {

	element_sub(*this, matB);
	return *this;
}

Matrix& Matrix::operator*=(const Matrix& matB) throw (exception_code) {

	element_mul(*this, matB);
	return *this;
}

Matrix& Matrix::operator/=(const Matrix& matB) throw (exception_code) {

	element_div(*this, matB);
	return *this;
}

bool Matrix::operator==(const Matrix& matB) const throw (exception_code) {
	bool equal = true;
	auto lambda = [&equal](auto A_i,auto B_i,auto i,auto j)->auto {equal=(A_i==B_i);return equal;};
	for_each(matB, lambda);
	return equal;
}

Matrix& Matrix::unary_operation(const Matrix& mSrc, Operators::unary_operator<real_t> function) throw (exception_code) {

	auto lambda = [function](auto& Dst_i,auto Src_i,auto i,auto j)->auto {Dst_i=function(Src_i);return true;};
	for_each(mSrc, lambda);
	return *this;
}

Matrix& Matrix::binary_operation(const Matrix& mSrcA, const Matrix& mSrcB, Operators::binary_operator<real_t> function) throw (exception_code) {

	auto lambda = [function](auto SrcA_i,auto SrcB_i,auto& Dst_i,auto i,auto j)->auto {Dst_i=function(SrcA_i,SrcB_i);return true;};
	for_each(mSrcA, mSrcB, lambda);
	return *this;
}

Matrix& Matrix::exp(const Matrix& mat_base, uint series_order) throw (exception_code) {

	mat_base.assert();
	if(series_order>20)
		throw exception_ERROR;

	try {
		assert();
	} catch (...) {
		*this = Matrix(mat_base, false);
	}
	Matrix& mat_exp = *this;

#ifdef ASSERT_DIMENSIONS
	if (!mat_base.is_square())
		throw exception_WRONG_DIMENSIONS;
	if (!mat_exp.is_square())
		throw exception_WRONG_DIMENSIONS;
	if (!mat_exp.equal_dimensions(mat_base))
		throw exception_WRONG_DIMENSIONS;
#endif
	mat_exp=0;
	Matrix mat_power_i_0(mat_base, false);
	Matrix mat_power_i_1(mat_base, false);
	mat_power_i_0.set_identity();
	mat_power_i_1.set_identity();
	uint64_t i_factorial = 1;

	for (uint i = 0; i < series_order; i++) {
		auto lambda = [i_factorial](auto& exp_i,auto power_i,auto i,auto j)->auto {exp_i+=(power_i/(double)i_factorial);return true;};
		mat_exp.for_each(mat_power_i_1, lambda);
		mat_power_i_1.multiply(mat_power_i_0, mat_base);
		mat_power_i_0 = mat_power_i_1;
		i_factorial *= (i + 1);
	}
	return mat_exp;
}

Matrix& Matrix::power(const Matrix& mat_base, uint power) throw (exception_code) {

	mat_base.assert();
	try {
		assert();
	} catch (...) {
		*this = Matrix(mat_base, false);
	}
	Matrix& mat_pow = *this;

#ifdef ASSERT_DIMENSIONS
	if (!mat_base.is_square())
		throw exception_WRONG_DIMENSIONS;
	if (!mat_pow.is_square())
		throw exception_WRONG_DIMENSIONS;
	if (!mat_pow.equal_dimensions(mat_base))
		throw exception_WRONG_DIMENSIONS;
#endif

	Matrix mat_power_i0(mat_base, false);
	mat_pow.set_identity();

	for (uint i = 0; i < power; i++) {
		mat_power_i0 = mat_pow;
		mat_pow.multiply(mat_power_i0, mat_base);
	}
	return mat_pow;
}

