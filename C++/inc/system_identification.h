#ifndef SYSTEM_IDENTIFICATION_H_
#define SYSTEM_IDENTIFICATION_H_

#include "common_def.h"
#include "vector.h"
#include "polynom.h"
#include "matrix.h"
#include "discrete_systems.h"

namespace SystemControl {

void theta_vector_extract_num_den(const VectorReal&,VectorReal&,VectorReal&);

namespace SystemIdentification {

real_t linear_regression(const Matrix&, const VectorReal&, VectorReal&);
real_t polynomial_fit(const VectorReal&, const VectorReal&, Polynom&, size_t);
real_t estimate_discrete_transfer_function(const VectorReal&, const VectorReal&, Polynom&, Polynom&, size_t, size_t);

class RecursiveLeastSquares {
public:
	RecursiveLeastSquares(size_t, real_t);

	inline real_t get_error()const {
		return error;
	}
	inline const VectorReal& get_theta()const {
		return theta;
	}

	inline const Matrix& get_P()const {
		return Pk_1;
	}
	inline void set_lambda(real_t lambda) {
		this->lambda = lambda;
	}

	void reset(real_t);
	real_t estimate(const VectorReal&, real_t);
	real_t estimate_discrete_transfer_function(const VectorReal&, real_t, Polynom&, Polynom&);

private:
	Matrix Pk_0;
	Matrix Pk_1;
	Matrix YhT;
	VectorReal theta;
	VectorReal Y;
	real_t lambda = 1.0;
	real_t error = 0;
};


}


}

#endif /* SYSTEM_IDENTIFICATION_H_ */
