#ifndef ODE_SOLVER_H_
#define ODE_SOLVER_H_


#include "common_def.h"
#include "vector_numeric.h"
#include "matrix.h"

namespace SystemControl
{

class ContinuousSystemInterface;

class ODE_solver_RK {

public:

	ODE_solver_RK(size_t, const real_t*, const real_t*, const real_t*);
	void solve(real_t, ContinuousSystemInterface&)throw(exception_code);

	inline real_t get_time() const{
		return time;
	}
	inline size_t get_order()const
	{
		return  A.get_n_rows();
	}

private:
	const Matrix A;
	const VectorReal B;
	const VectorReal C;
	Matrix k;
	VectorReal Xn;
	real_t time = 0;
};

class ODE_solver_RK_ForwardEuler: public ODE_solver_RK {

public:
	ODE_solver_RK_ForwardEuler() :
			ODE_solver_RK(1, *A_values, B_values, C_values) {
	}
private:

	static const real_t A_values[1][1];
	static const real_t B_values[1];
	static const real_t C_values[1];
};


class ODE_solver_RK_midpoint: public ODE_solver_RK {

public:
	ODE_solver_RK_midpoint() :
			ODE_solver_RK(2, *A_values, B_values, C_values) {
	}
private:
	static const real_t A_values[2][2];
	static const real_t B_values[2];
	static const real_t C_values[2];

};

class ODE_solver_RK_Heun: public ODE_solver_RK {

public:
	ODE_solver_RK_Heun() :
			ODE_solver_RK(2, *A_values, B_values, C_values) {
	}
private:

	static const real_t A_values[2][2];
	static const real_t B_values[2];
	static const real_t C_values[2] ;

};

class ODE_solver_RK_Ralston: public ODE_solver_RK {

public:
	ODE_solver_RK_Ralston() :
			ODE_solver_RK(3, *A_values, B_values, C_values) {
	}
private:
	static const real_t A_values[3][3];
	static const real_t B_values[3];
	static const real_t C_values[3];

};

class ODE_solver_RK_Kutta4: public ODE_solver_RK {

public:
	ODE_solver_RK_Kutta4() :
			ODE_solver_RK(4, *A_values, B_values, C_values) {
	}
private:
	static const real_t A_values[4][4];
	static const real_t B_values[4];
	static const real_t C_values[4];
};
}

#endif /* ODE_SOLVER_H_ */
