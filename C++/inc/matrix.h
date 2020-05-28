#ifndef MATRIX_CPP_H_
#define MATRIX_CPP_H_

#include "common_def.h"

namespace SystemControl {

class Matrix {
public:

	~Matrix();
	Matrix() {
	}
	Matrix(size_t, size_t, real_t* = NULL) throw (exception_code);
	Matrix(size_t, size_t, const real_t*) throw (exception_code);
	Matrix(const Matrix&, bool = true) throw (exception_code);
	void assert() const throw (exception_code);

#ifdef USE_GSL
	Matrix(const gsl_matrix&) throw (exception_code);
	gsl_matrix to_gsl_matrix() throw (exception_code);
	const gsl_matrix to_gsl_matrix() const throw (exception_code);
	gsl_matrix* to_gsl_matrix_dynamic_copy() const throw (exception_code);
#endif

	inline real_t& at(uint row, uint col) {
		return data_ptr[col + row * n_cols_mem];
	}
	inline real_t at(uint row, uint col) const {
		return data_ptr[col + row * n_cols_mem];
	}

	inline real_t* ptr_at(uint row, uint col) {
		return &at(row, col);
	}

	real_t& at_safe(uint row, uint col) throw (exception_code);
	real_t at_safe(uint row, uint col) const throw (exception_code);

	inline real_t& operator()(uint row, uint col) {
		return at(row, col);
	}
	inline real_t operator()(uint row, uint col) const {
		return at(row, col);
	}
	inline void resize(size_t n_rows, size_t n_cols) throw (exception_code) {
		if (n_rows != this->n_rows || n_cols != this->n_cols) {
			deinit();
			init(n_rows, n_cols);
		}
	}

	inline size_t get_n_rows() const {
		return n_rows;
	}
	inline size_t get_n_cols() const {
		return n_cols;
	}

	inline real_t* get_data_ptr() {
		return data_ptr;
	}

	inline const real_t* get_data_ptr() const {
		return data_ptr;
	}

	inline bool is_scalar() const {
		return n_rows == 1 && n_cols == 1;
	}

	inline bool is_square() const {
		return n_rows == n_cols;
	}

	inline bool is_col_vector() const {
		return n_rows > 1 && n_cols == 1;
	}

	inline bool is_row_vector() const {
		return n_rows == 1 && n_cols > 1;
	}

	inline bool equal_dimensions(const Matrix& matB) const {
		return (this->n_rows == matB.n_rows && this->n_cols == matB.n_cols);
	}

	void set_all_elements(real_t = 0) throw (exception_code);
	void set_identity() throw (exception_code);
	void load_data(const real_t*) throw (exception_code);

	Matrix& multiply(const Matrix&, const Matrix&) throw (exception_code);
	Matrix multiply(const Matrix&) const throw (exception_code);
	Matrix& multiply_by_scalar_and_accumulate(const Matrix&, real_t) throw (exception_code);

	Matrix transpose()const throw (exception_code);
	void transpose() throw (exception_code);
	Matrix& exp(const Matrix&, uint=10) throw (exception_code);
	Matrix& power(const Matrix&, uint) throw (exception_code);

	Matrix submatrix(size_t, size_t, size_t = 0, size_t = 0) throw (exception_code);
	Matrix row(size_t) throw (exception_code);
	Matrix column(size_t) throw (exception_code);
	Matrix diagonal() throw (exception_code);

	Matrix& operator=(const Matrix&) throw (exception_code);
	Matrix& operator=(real_t) throw (exception_code);

	Matrix& element_add(const Matrix&, const Matrix&) throw (exception_code);
	Matrix& element_add(const Matrix&, real_t) throw (exception_code);
	Matrix& element_sub(const Matrix&, const Matrix&) throw (exception_code);
	Matrix& element_sub(const Matrix&, real_t) throw (exception_code);
	Matrix& element_mul(const Matrix&, const Matrix&) throw (exception_code);
	Matrix& element_mul(const Matrix&, real_t) throw (exception_code);
	Matrix& element_div(const Matrix&, const Matrix&) throw (exception_code);
	Matrix& element_div(const Matrix&, real_t) throw (exception_code);

	Matrix operator+(const Matrix&) const throw (exception_code);
	Matrix operator-(const Matrix&) const throw (exception_code);
	Matrix operator*(const Matrix&) const throw (exception_code);
	Matrix operator+(real_t) const throw (exception_code);
	Matrix operator-(real_t) const throw (exception_code);
	Matrix operator*(real_t) const throw (exception_code);
	Matrix& operator+=(const Matrix&) throw (exception_code);
	Matrix& operator-=(const Matrix&) throw (exception_code);
	Matrix& operator*=(const Matrix&) throw (exception_code);
	Matrix& operator/=(const Matrix&) throw (exception_code);
	Matrix& operator+=(real_t) throw (exception_code);
	Matrix& operator-=(real_t) throw (exception_code);
	Matrix& operator*=(real_t) throw (exception_code);
	Matrix& operator/=(real_t) throw (exception_code);

	bool operator==(const Matrix&) const throw (exception_code);

	Matrix& unary_operation(const Matrix&, Operators::unary_operator<real_t>) throw (exception_code);
	Matrix& binary_operation(const Matrix&, const Matrix&, Operators::binary_operator<real_t>) throw (exception_code);

	template<typename F>
	void for_each(F) throw (exception_code);
	template<typename F>
	void for_each(F) const throw (exception_code);
	template<typename F>
	void for_each(const Matrix&, F) throw (exception_code);
	template<typename F>
	void for_each(const Matrix&, F) const throw (exception_code);
	template<typename F>
	void for_each(const Matrix&, const Matrix&, F) throw (exception_code);
	template<typename F>
	void for_diagonal(const Matrix&, F) throw (exception_code);

protected:
	void init(size_t, size_t, real_t* = NULL) throw (exception_code);
	void deinit();

private:
	size_t n_rows = 0;
	size_t n_cols = 0;
	size_t n_cols_mem = 0;
	allocation_type_enum allocation_info = allocation_type_unallocated;
	real_t* data_ptr = NULL;

};

}
#endif /* MATRIX_CPP_H_ */
