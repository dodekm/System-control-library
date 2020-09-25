#ifndef INC_PREDICTION_H_
#define INC_PREDICTION_H_

#include "common_def.h"
#include "vector.h"
#include "discrete_systems.h"


namespace SystemControl {

namespace Prediction {

class DiscretePredictor {
public:

	DiscretePredictor(uint n, const VectorReal& theta) :
			h(n), theta(theta) {
		theta.assert();
		if (theta.get_length() != n)
			throw exception_code(exception_WRONG_DIMENSIONS);
	}

	inline const VectorReal& get_regressor() const {
		return h;
	}
	inline VectorReal& get_regressor(){
		return h;
	}
	real_t predict() {
		return h.dot_product(theta);
	}

protected:
	VectorReal h;
	const VectorReal& theta;

};

class DiscretePredictor1input: public DiscretePredictor {

public:
	DiscretePredictor1input(uint nb, uint na, const VectorReal& theta) :
			DiscretePredictor(nb + na, theta), nb(nb), na(na) {
	}

	inline void update(real_t u, real_t y) {
		update(u, y, h);
	}
	void predict(const VectorReal&, VectorReal&);

private:

	inline void update(real_t u, real_t y, VectorReal& h) {
		discrete_transfer_function_vector_h_update(h, u, y, nb, na);
	}

	uint nb;
	uint na;

};

class DiscretePredictor2input: public DiscretePredictor {

public:
	DiscretePredictor2input(uint nb_1, uint nb_2, uint na, const VectorReal& theta) :
			DiscretePredictor(nb_1 + nb_2 + na, theta), nb_1(nb_1), nb_2(nb_2), na(na) {

	}

	inline void update(real_t u_1, real_t u_2, real_t y) {
		update(u_1, u_2, y, h);
	}

	void predict(const VectorReal&, const VectorReal&, VectorReal&);

private:

	void update(real_t, real_t, real_t, VectorReal&);

	uint nb_1;
	uint nb_2;
	uint na;

};

}
}

#endif /* INC_PREDICTION_H_ */
