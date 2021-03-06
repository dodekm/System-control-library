#include "system_identification.h"

using namespace SystemControl;

real_t SystemIdentification::linear_regression(const Matrix& H, const VectorReal& y_vector, VectorReal& theta) {

	y_vector.assert();
	theta.assert();
	H.assert();

#ifdef ASSERT_DIMENSIONS
	if (y_vector.get_length() != H.get_n_rows())
	throw exception_WRONG_DIMENSIONS;
	if (theta.get_length() != H.get_n_cols())
	throw exception_WRONG_DIMENSIONS;
#endif

	size_t n_params = theta.get_length();
	size_t n_observ = y_vector.get_length();

#ifdef USE_GSL

	gsl_vector c = theta.to_gsl_vector();
	const gsl_vector y = y_vector.to_gsl_vector();
	const gsl_matrix H_mat_gsl = H.to_gsl_matrix();

	gsl_matrix* cov = NULL;
	gsl_multifit_linear_workspace * work = NULL;

	auto dealloc = [&]() {
		gsl_multifit_linear_free(work);
		gsl_matrix_free(cov);
	};

	cov = gsl_matrix_alloc(n_params, n_params);
	if (cov == NULL) {
		throw exception_NULLPTR;
	}
	work = gsl_multifit_linear_alloc(n_observ, n_params);
	if (work == NULL) {
		dealloc();
		throw exception_NULLPTR;

	}
	real_t chisq;
	exception_code returnval = exception_OK;
	returnval = exception_code(gsl_multifit_linear(&H_mat_gsl, &y, &c, cov, &chisq, work));
	dealloc();
	if (returnval != exception_OK)
	throw returnval;
	return chisq;
#else

	const MatrixTranspose HT = MatrixTranspose(H);
	const VectorReal HTy = HT.multiply(y_vector);
	Matrix HTH = HT.multiply(H);
	HTH.solve(HTy, theta);

	return 0;
#endif
}

real_t SystemIdentification::polynomial_fit(const VectorReal& x_vector, const VectorReal& y_vector, Polynom& polynom, size_t degree) {

	x_vector.assert();
	y_vector.assert();
	polynom.assert();

#ifdef ASSERT_DIMENSIONS
	if (x_vector.get_length() != y_vector.get_length())
	throw exception_WRONG_DIMENSIONS;
	if (polynom.get_length() != degree + 1)
	throw exception_WRONG_DIMENSIONS;
#endif

	size_t n_params = degree + 1;
	size_t n_observ = y_vector.get_length();

	Matrix X_mat(n_observ, n_params);

	for (uint i = 0; i < n_observ; i++) {
		real_t x_pow = 1;
		real_t x_i = x_vector[i];
		for (uint j = 0; j < n_params; j++) {
			X_mat(i, j) = x_pow;
			x_pow *= x_i;
		}
	}
	return linear_regression(X_mat, y_vector, polynom);
}

real_t SystemIdentification::estimate_discrete_transfer_function(const VectorReal& u_vector, const VectorReal& y_vector, VectorReal& numerator, VectorReal& denominator, size_t nb, size_t na,real_t u_past,real_t y_past) {

	u_vector.assert();
	y_vector.assert();
	try {
		numerator.assert();
	} catch (...) {
		numerator = Polynom(nb + 1);
	}
	try {
		denominator.assert();
	} catch (...) {
		denominator = Polynom(na + 1);
	}

#ifdef ASSERT_DIMENSIONS
	if (u_vector.get_length() != y_vector.get_length())
	throw exception_WRONG_DIMENSIONS;
	if (numerator.get_length() != nb + 1)
	throw exception_WRONG_DIMENSIONS;
	if (denominator.get_length() != na + 1)
	throw exception_WRONG_DIMENSIONS;
#endif

	size_t n_params = nb + na;
	size_t n_observ = y_vector.get_length();

	Matrix X_mat(n_observ, n_params);

	for (int i = 1; i < (int) n_observ; i++) {
		for (int j = 0; j < (int) nb; j++) {
			if (i - (int) nb + j < 0)
				X_mat(i, j) = u_past;
			else
				X_mat(i, j) = u_vector[i - (int) nb + j];
		}
		for (int j = 0; j < (int) na; j++) {
			if (i - (int) na + j < 0)
				X_mat(i, j + (int) nb) = -y_past;
			else
				X_mat(i, j + (int) nb) = -y_vector[i - (int) na + j];
		}
	}

	VectorDiscreteParameters theta(numerator, denominator);
	numerator[nb] = 0;
	denominator[na] = 1;

	return linear_regression(Matrix(X_mat,n_observ-1,n_params,1,0),VectorReal(y_vector, n_observ - 1, 1), theta);


}

SystemIdentification::RecursiveLeastSquares::RecursiveLeastSquares(size_t n_params, real_t P0_initial_values) :
		Pk_0(n_params, n_params), Pk_1(n_params, n_params), YhT(n_params, n_params), theta(n_params), Y(n_params) {
	reset(P0_initial_values);
}

void SystemIdentification::RecursiveLeastSquares::reset(real_t P0_initial_values) {

	Pk_0.set_all_elements(0);
	Pk_1.set_all_elements(0);
	Pk_0.diagonal() = P0_initial_values;
	Pk_1.diagonal() = P0_initial_values;

	theta.set_all(0);
	Y.set_all(0);
	YhT.set_all_elements(0);
	error = 0;
}

real_t SystemIdentification::RecursiveLeastSquares::estimate(const VectorReal& hk_1, real_t yk_1) {

	hk_1.assert();
#ifdef  ASSERT_DIMENSIONS
	if (hk_1.get_length() != theta.get_length())
	throw exception_WRONG_DIMENSIONS;
#endif
#ifdef USE_GSL
	const gsl_vector hk_1_gsl = hk_1.to_gsl_vector();
	gsl_vector theta = this->theta.to_gsl_vector();
	gsl_vector Y = this->Y.to_gsl_vector();
	gsl_matrix Pk_0 = this->Pk_0.to_gsl_matrix();
	gsl_matrix Pk_1 = this->Pk_1.to_gsl_matrix();
	gsl_matrix YhT = this->YhT.to_gsl_matrix();

	exception_code returnval = exception_OK;
	returnval = exception_code(gsl_matrix_memcpy(&Pk_0, &Pk_1)); //Pk_0=Pk_1
	if (returnval != exception_OK)
	throw returnval;

	real_t y_hat = 0;
	returnval = exception_code(gsl_blas_ddot(&hk_1_gsl, &theta, &y_hat));//y_estimated_k_1=hk_1_T*theta
	if (returnval != exception_OK)
	throw returnval;

	error = yk_1 - y_hat;

	returnval = exception_code(gsl_blas_dgemv(CblasNoTrans, 1, &Pk_0, &hk_1_gsl, 0, &Y));//Pk_0*hk_1
	if (returnval != exception_OK)
	throw returnval;
	real_t rho = 0;

	returnval = exception_code(gsl_blas_ddot(&hk_1_gsl, &Y, &rho));//rho=hk_1_T*Pk_0*hk_1
	if (returnval != exception_OK)
	throw returnval;
	rho += lambda;
	returnval = exception_code(gsl_vector_scale(&Y, 1.0 / rho));
	if (returnval != exception_OK)
	throw returnval;

	gsl_matrix Yk_1_mat = gsl_matrix_view_vector(&Y, Y.size, 1).matrix;
	const gsl_matrix hk_1_T_mat = gsl_matrix_const_view_vector(&hk_1_gsl, 1, hk_1_gsl.size).matrix;
	returnval = exception_code(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0 / lambda, &Yk_1_mat, &hk_1_T_mat, 0, &YhT));//YhT=Yk_1*hk_1_T
	if (returnval != exception_OK)
	throw returnval;

	returnval = exception_code(gsl_matrix_add_diagonal(&YhT, 1.0 / lambda));
	if (returnval != exception_OK)
	throw returnval;
	returnval = exception_code(gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, &YhT, &Pk_0, 0, &Pk_1));
	if (returnval != exception_OK)
	throw returnval;

	returnval = exception_code(gsl_blas_daxpy(error, &Y, &theta));//theta_k1=theta_k0+e*Y;
	if (returnval != exception_OK)
	throw returnval;

#else

	Pk_0 = Pk_1;
	error = yk_1 - hk_1.dot_product(theta);
	Pk_0.multiply(hk_1, Y);
	Y/=(hk_1.dot_product(Y) + lambda);
	YhT.multiplyTvector(Y, hk_1);
	YhT.diagonal() -= 1.0;
	YhT/=-lambda;
	Pk_1.multiply(YhT, Pk_0);
	Y*=error;
	theta+=Y;
#endif
	return error;

}

