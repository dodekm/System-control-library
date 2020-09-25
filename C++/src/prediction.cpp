#include "prediction.h"

using namespace SystemControl;
using namespace Prediction;

void DiscretePredictor1input::predict(const VectorReal& u_vector, VectorReal& y_vector) {
	if (u_vector.get_length() != y_vector.get_length())
		throw exception_code(exception_WRONG_DIMENSIONS);
	VectorReal h(this->h);
	for (uint i = 0; i < y_vector.get_length(); i++) {
		real_t y = h.dot_product(theta);
		y_vector[i] = y;
		update(u_vector[i], y, h);
	}
}

void DiscretePredictor2input::predict(const VectorReal& u_1_vector, const VectorReal& u_2_vector, VectorReal& y_vector) {
	if (u_1_vector.get_length() != y_vector.get_length() || u_2_vector.get_length() != y_vector.get_length())
		throw exception_code(exception_WRONG_DIMENSIONS);
	VectorReal h(this->h);
	for (uint i = 0; i < y_vector.get_length(); i++) {
		real_t y = h.dot_product(theta);
		y_vector[i] = y;
		update(u_1_vector[i], u_2_vector[i], y, h);
	}
}

void DiscretePredictor2input::update(real_t u_1, real_t u_2, real_t y, VectorReal& h) {

	VectorReal h_1(h,nb_1, 0);
	VectorReal h_2(h,nb_2, nb_1);
	VectorReal h_3(h,na, nb_1 + nb_2);
	discrete_transfer_function_vector_h_update(h_1, u_1, 0, nb_1, 0);
	discrete_transfer_function_vector_h_update(h_2, u_2, 0, nb_2, 0);
	discrete_transfer_function_vector_h_update(h_3, 0, y, 0, na);

}

