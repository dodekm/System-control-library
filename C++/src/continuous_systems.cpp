#include "continuous_systems.h"

using namespace SystemControl;


ContinuousSystemInterface::ContinuousSystemInterface(size_t order, real_t* dX_data_ptr, real_t* X_data_ptr) :
		X(order, X_data_ptr), dX(order, dX_data_ptr) {
	X.set_all(0);
	dX.set_all(0);
}

ContinuousSystem::ContinuousSystem(size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr, size_t order, real_t* dX_data_ptr, real_t* X_data_ptr) :
		System(system_type_dynamic_system, system_time_domain_continuous, input_port_width, input_port_ptr, output_port_width, output_port_ptr),ContinuousSystemInterface(order, dX_data_ptr, X_data_ptr) {
}

ContinuousSystemSISO::ContinuousSystemSISO(size_t order, real_t* dX_data_ptr, real_t* X_data_ptr) :
		ContinuousSystem(1, &input, 1, &output, order, dX_data_ptr, X_data_ptr) {
	input = 0;
	output = 0;

}

ContinuousSystemMIMO::ContinuousSystemMIMO(size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr, size_t order, real_t* dX_data_ptr, real_t* X_data_ptr) :
		ContinuousSystem(input_port_width, input_port_ptr, output_port_width, output_port_ptr, order, dX_data_ptr, X_data_ptr) {
	if (input_port_ptr != NULL)
		memset(input_port_ptr, 0, sizeof(real_t) * input_port_width);
	if (output_port_ptr != NULL)
		memset(output_port_ptr, 0, sizeof(real_t) * output_port_width);
}

ContinuousStateSpace::ContinuousStateSpace(size_t order, real_t* A_data_ptr, real_t* B_data_ptr, real_t* C_data_ptr, real_t* dX_data_ptr, real_t* X_data_ptr) :
		StateSpace(order, System::system_time_domain_continuous, A_data_ptr, B_data_ptr, C_data_ptr), ContinuousSystemSISO(order, dX_data_ptr, X_data_ptr) {

}

void ContinuousStateSpace::update_derivatives_fcn(real_t time, step_type step_type) {

	(void) time;
	real_t u = input_port_real_value(0);
#ifdef USE_GSL

	const gsl_matrix A_gsl = A.to_gsl_matrix();
	const gsl_vector B_gsl = B.to_gsl_vector();
	const gsl_vector X_gsl = X.to_gsl_vector();
	gsl_vector dX_gsl = dX.to_gsl_vector();
	gsl_blas_dgemv(CblasNoTrans, 1.0, &A_gsl, &X_gsl, 0.0, &dX_gsl);
	gsl_blas_daxpy(u, &B_gsl, &dX_gsl);
#else
	A.multiply(X,dX);
	dX.multiply_by_scalar_and_accumulate(B,u);
#endif

}
void ContinuousStateSpace::update_output_fcn(real_t time, step_type step_type) {

#ifdef USE_GSL
	const gsl_vector C_gsl = C.to_gsl_vector();
	const gsl_vector X_gsl = X.to_gsl_vector();

	real_t y = 0;
	gsl_blas_ddot(&C_gsl, &X_gsl, &y);
	output_port_real_value(0) = y;
#else
	real_t y = C.dot_product(X);
	output_port_real_value(0) = y;
#endif

}

ContinuousTransferFuction::ContinuousTransferFuction(size_t numerator_order, size_t denominator_order, real_t* numerator_coeffs_ptr, real_t* denominator_coeffs_ptr, real_t* dX_data_ptr, real_t* X_data_ptr) :
		TransferFunction(numerator_order, denominator_order, System::system_time_domain_continuous, numerator_coeffs_ptr, denominator_coeffs_ptr), ContinuousSystemSISO(denominator_order, dX_data_ptr, X_data_ptr) {
#ifdef ASSERT_DIMENSIONS
	if (numerator_order > denominator_order)
		throw exception_WRONG_DIMENSIONS;
#endif

}

void ContinuousTransferFuction::update_derivatives_fcn(real_t time, step_type step_type) {

	(void) time;
	(void) step_type;
	size_t system_order = ContinuousSystem::get_order();
	dX[system_order - 1] = 0;

	for (uint i = 0; i < system_order; i++) {
		if (i < system_order - 1) {
			dX[i] = X[i + 1];
		}
		dX[system_order - 1] += X[i] * (-denominator_coeffs[i]);
	}
	dX[system_order - 1] += input_port_real_value(0);
	dX[system_order - 1] /= denominator_coeffs[system_order];

}

void ContinuousTransferFuction::update_output_fcn(real_t time, step_type step_type) {

	(void) time;
	(void) step_type;
	real_t y = 0;
	size_t system_order = ContinuousSystem::get_order();
	for (uint i = 0; i < numerator_coeffs.get_length(); i++) {
		if (i < system_order)
			y += numerator_coeffs[i] * X[i];
		else if (i == system_order)
			y += numerator_coeffs[i] * dX[i - 1];
	}
	output_port_real_value(0) = y;

}

ContinuousTransferFuctionFirstOrder::ContinuousTransferFuctionFirstOrder(System1stOrderParams& params) :
		ContinuousTransferFuction(0, 1, numerator_data, denominator_data, dX_data, X_data) {
	numerator_data[0] = params.K;
	denominator_data[0] = 1;
	denominator_data[1] = params.T;
}

ContinuousTransferFuctionSecondOrder::ContinuousTransferFuctionSecondOrder(System2ndOrderParams& params) :
		ContinuousTransferFuction(0, 2, numerator_data, denominator_data, dX_data, X_data) {

	numerator_data[0] = params.K * POW2(params.omega_0);
	denominator_data[0] = POW2(params.omega_0);
	denominator_data[1] = (real_t)2.0 * params.b * params.omega_0;
	denominator_data[2] = 1;
}

ContinuousIntegrator::ContinuousIntegrator() :
		ContinuousSystemSISO(1, dX_data, X_data) {

}
void ContinuousIntegrator::update_derivatives_fcn(real_t time, step_type step_type) {
	(void) time;
	(void) step_type;
	dX[0] = input_port_real_value(0);

}

void ContinuousIntegrator::update_output_fcn(real_t time, step_type step_type) {

	(void) time;
	(void) step_type;
	output_port_real_value(0) = X[0];
}

ContinuousPID_regulator::ContinuousPID_regulator(real_t P_gain, real_t I_gain, real_t D_gain, real_t N_gain) :
		ContinuousSystemSISO(2, dX_data, X_data) {
	this->P_gain = P_gain;
	this->I_gain = I_gain;
	this->D_gain = D_gain;
	this->N_gain = N_gain;

}





void ContinuousPID_regulator::update_derivatives_fcn(real_t time, step_type step_type) {
	(void) time;
	(void) step_type;
	dX[0] = input_port_real_value(0);
	dX[1] = N_gain *(input_port_real_value(0) - X[1]);

}



void ContinuousPID_regulator::update_output_fcn(real_t time, step_type step_type) {

	(void) time;
	(void) step_type;
	real_t y, y_P, y_I, y_D;
	y_P = P_gain * input_port_real_value(0);
	y_I = I_gain * X[0];
	y_D = D_gain * N_gain * (input_port_real_value(0) - X[1]);
	y = y_P + y_I + y_D;
	output_port_real_value(0) = y;

}

ContinuousNonlinearSystem::ContinuousNonlinearSystem(derrivatives_fcn custom_derrivatives_fcn, output_fcn custom_update_output_fcn, size_t order, size_t n_params, real_t* params_data_ptr) :
		ContinuousSystemSISO(order, NULL, NULL), params(n_params) {
	params.load_data(params_data_ptr);
	this->custom_derrivatives_fcn = custom_derrivatives_fcn;
	this->custom_update_output_fcn = custom_update_output_fcn;

}

void ContinuousNonlinearSystem::update_derivatives_fcn(real_t time, step_type step_type) {
	(void) time;
	(void) step_type;

	custom_derrivatives_fcn(dX.get_data_ptr(), X.get_data_ptr(), params.get_data_ptr(), input_port_real_value(0), time);

}
void ContinuousNonlinearSystem::update_output_fcn(real_t time, step_type step_type) {
	(void) time;
	(void) step_type;
	custom_update_output_fcn(X.get_data_ptr(), params.get_data_ptr(), input_port_real_value(0), output_port_struct(0).ptr);

}

