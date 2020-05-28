#include "wrappers.h"

using namespace SystemControl;

#ifdef USE_GSL

exception_code Convert::gsl_error_code_to_return_code(int error) {
	switch (error) {

	case GSL_SUCCESS:
		return exception_OK;
	case GSL_ENOMEM:
		return exception_NULLPTR;
	case GSL_EFAULT:
		return exception_NULLPTR;
	case GSL_EBADLEN:
		return exception_WRONG_DIMENSIONS;
	case GSL_ENOTSQR:
		return exception_WRONG_DIMENSIONS;
	default:
		return exception_ERROR;

	}
}

int Convert::return_code_to_gsl_error_code(exception_code error) {
	switch (error) {
	case exception_OK:
		return GSL_SUCCESS;
	case exception_ERROR:
		return GSL_FAILURE;
	case exception_NULLPTR:
		return GSL_EFAULT;
	case exception_WRONG_DIMENSIONS:
		return GSL_EBADLEN;
	default:
		return GSL_FAILURE;

	}
}
#endif


VectorReal Convert::Matrix2VectorReal(Matrix& M)
{
	return VectorReal(M.get_n_rows()*M.get_n_cols(),M.get_data_ptr());
}

const VectorReal Convert::Matrix2VectorReal(const Matrix& M)
{
	return VectorReal(M.get_n_rows()*M.get_n_cols(),M.get_data_ptr());
}

Matrix Convert::VectorReal2rowMatrix(VectorReal& V)
{
	return Matrix(1,V.get_length(), V.get_data_ptr());
}

Matrix Convert::VectorReal2colMatrix(VectorReal& V)
{
	return Matrix(V.get_length(), 1, V.get_data_ptr());
}


const Matrix Convert::VectorReal2rowMatrix(const VectorReal& V)
{
	return Matrix(1,V.get_length(), V.get_data_ptr());
}

const Matrix Convert::VectorReal2colMatrix(const VectorReal& V)
{
	return Matrix(V.get_length(), 1, V.get_data_ptr());
}




