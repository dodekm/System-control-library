#include "ode_solver.h"
#include "continuous_systems.h"

using namespace SystemControl;

ODE_solver_RK::ODE_solver_RK(size_t order, const real_t* A_data, const real_t* B_data, const real_t* C_data) :
		A(order, order, const_cast<real_t*>(A_data)), B(order, const_cast<real_t*>(B_data)), C(order, const_cast<real_t*>(C_data)) {

}

void ODE_solver_RK::solve(real_t step_size, ContinuousSystemInterface& system) throw (exception_code) {

	VectorReal& X = system.get_states();
	const VectorReal& dX = system.get_derivatives();
	X.assert();
	dX.assert();

#ifdef ASSERT_DIMENSIONS
	if (X.get_length() != dX.get_length())
		throw exception_WRONG_DIMENSIONS;
#endif

	uint system_order = system.get_order();
	uint solver_order = this->get_order();

	if (k.get_n_rows() != solver_order || k.get_n_cols() != system_order) {
		k = Matrix(solver_order, system_order);
	}
	Xn = X;
	for (uint i = 0; i < solver_order; i++) {
		if (i > 0) {
			for (uint n = 0; n < system_order; n++) {
				real_t Xn_minor = Xn[n];
				for (uint j = 0; j < i; j++) {
					Xn_minor += A(i, j) * k(j, n);
				}
				X[n] = Xn_minor;
			}
		}
		real_t time_minor = time + step_size * C[i];
		system.update_output_fcn(time_minor, step_minor);
		system.update_derivatives_fcn(time_minor, step_minor);

		for (uint n = 0; n < system_order; n++) {
			k(i, n) = step_size * dX[n];
		}
	}
	for (uint n = 0; n < system_order; n++) {
		X[n] = Xn[n];
		for (uint i = 0; i < solver_order; i++) {
			X[n] += B[i] * k(i, n);
		}
	}
	time += step_size;
	system.update_output_fcn(time, step_major);
	system.update_derivatives_fcn(time, step_major);
}

const real_t ODE_solver_RK_ForwardEuler::A_values[1][1] = { { 0 } };
const real_t ODE_solver_RK_ForwardEuler::B_values[1] = { 1 };
const real_t ODE_solver_RK_ForwardEuler::C_values[1] = { 0 };

const real_t ODE_solver_RK_midpoint::A_values[2][2] = { { 0, 0 }, { 0, 0.5 } };
const real_t ODE_solver_RK_midpoint::B_values[2] = { 0, 1 };
const real_t ODE_solver_RK_midpoint::C_values[2] = { 0, 0.5 };

const real_t ODE_solver_RK_Heun::A_values[2][2] = { { 0, 0 }, { 1, 0 } };
const real_t ODE_solver_RK_Heun::B_values[2] = { 0.5, 0.5 };
const real_t ODE_solver_RK_Heun::C_values[2] = { 0, 1 };

const real_t ODE_solver_RK_Ralston::A_values[3][3] = { { 0, 0, 0 }, { 0.5, 0, 0 }, { 0, 3.0 / 4.0, 0 } };
const real_t ODE_solver_RK_Ralston::B_values[3] = { 0, 0.5, 3.0 / 4.0 };
const real_t ODE_solver_RK_Ralston::C_values[3] = { 2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0 };

const real_t ODE_solver_RK_Kutta4::A_values[4][4] = { { 0, 0, 0, 0 }, { 0.5, 0, 0, 0 }, { 0, 0.5, 0, 0 }, { 0, 0, 1, 0 } };
const real_t ODE_solver_RK_Kutta4::B_values[4] = { 1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0 };
const real_t ODE_solver_RK_Kutta4::C_values[4] = { 0, 0.5, 0.5, 1 };

