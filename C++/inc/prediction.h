#ifndef INC_PREDICTION_H_
#define INC_PREDICTION_H_

#include "common_def.h"
#include "vector.h"
#include "discrete_systems.h"

namespace SystemControl {

namespace Prediction {

class GPC: public Saturation {

public:
	GPC(uint n_e1, uint n_e2, uint n_u, const VectorReal& lambda_u, const VectorReal& lambda_y, const VectorReal& B, const VectorReal& B_dist, const VectorReal& A, const VectorReal& T) :
			n_e1(n_e1), n_e2(n_e2), n_u(n_u), B(B), B_dist(B_dist), A(A), T(T), H(n_e2, n_e2), HT_lambda_y(n_u, n_e2 - n_e1), invHTH(n_u, n_u), u_p(B.get_length() - 1), y_p(A.get_length() - 1), vm_p(B_dist.get_length() - 1), vu_p(T.get_length() - 1), e(n_e2 - n_e1), HTe(n_u), du_f(n_u), y_free(n_e2), y_forced(n_e2), u_f(n_e2) {

#ifdef ASSERT_DIMENSIONS
		if (lambda_u.get_length() != n_u)
		throw exception_WRONG_DIMENSIONS;
		if (lambda_y.get_length() != n_e2 - n_e1)
		throw exception_WRONG_DIMENSIONS;
#endif

		fill_H_matrix(H, B, A);
		Matrix H(this->H, n_e2 - n_e1, n_u, n_e1, 0);
		const MatrixTranspose HT(H);
		Matrix lambda_y_mat(n_e2 - n_e1, n_e2 - n_e1);
		lambda_y_mat.diagonal() = Matrix::VectorReal2colMatrix(lambda_y);
		HT_lambda_y.multiply(HT, lambda_y_mat);
		Matrix HTH = HT_lambda_y.multiply(H);
		HTH.diagonal() += Matrix::VectorReal2colMatrix(lambda_u);
		MatrixPermRow HTHP(HTH);
		HTHP.LU_decompose();
		HTHP.LU_inv(invHTH);

	}
	real_t operator()(real_t, const VectorReal&, const VectorReal&);

	real_t get_v_uk() const {
		return v_uk;
	}
	real_t get_u_k() const {
		return u_k;
	}

	void get_yf(VectorReal& y_f) const {
		y_f.add(y_free, y_forced);
	}

	static void predict(VectorReal& y_f, const VectorReal& u_f, const VectorReal& y_p, const VectorReal& u_p, const VectorReal& B, const VectorReal& A);
	static void predict(VectorReal& y_f, const VectorReal& u_f, const VectorReal& vm_f, real_t vu_k, const VectorReal& y_p, const VectorReal& u_p, const VectorReal& vm_p, const VectorReal& vu_p, const VectorReal& B, const VectorReal& B_dist, const VectorReal& A, const VectorReal& T);
private:

	static void fill_M_yf(Matrix& M_yf, const VectorReal& A);
	static void fill_M_uf(Matrix& M_uf, const VectorReal& B);

	static void fill_M_yp(Matrix& M_yp, const VectorReal& A);
	static void fill_M_up(Matrix& M_up, const VectorReal& B);
	static void fill_H_matrix(Matrix& H, const VectorReal& B, const VectorReal& A);

private:

	const uint n_e1;
	const uint n_e2;
	const uint n_u;

	const VectorReal& B;
	const VectorReal& B_dist;
	const VectorReal& A;
	const VectorReal& T;

	Matrix H;
	Matrix HT_lambda_y;
	Matrix invHTH;

	VectorStates u_p;
	VectorStates y_p;
	VectorStates vm_p;
	VectorStates vu_p;

	VectorReal e;
	VectorReal HTe;
	VectorReal du_f;
	VectorReal y_free;
	VectorReal y_forced;

	VectorReal u_f;

	real_t u_k = 0;
	real_t v_uk = 0;

};

class DiscretePredictor1input {

public:
	DiscretePredictor1input(uint nb, uint na, const VectorReal& theta) :
			h(nb, na), theta(theta) {
	}

	inline void update(real_t u, real_t y) {
		h.update(u, y);
	}
	void predict(const VectorReal&, VectorReal&);
	real_t predict() {
		return h.dot_product(theta);
	}

	VectorDiscreteRegressor h;
private:
	const VectorReal& theta;
};

class DiscretePredictor2input {

public:
	DiscretePredictor2input(uint nb_1, uint nb_2, uint na, const VectorReal& theta) :
			h(nb_1, nb_2, na), theta(theta) {
	}

	inline void update(real_t u_1, real_t u_2, real_t y) {
		h.update(u_1, u_2, y);
	}

	void predict(const VectorReal&, const VectorReal&, VectorReal&);
	real_t predict() {
		return h.dot_product(theta);
	}
	VectorDiscreteRegressor2Input h;
private:
	const VectorReal& theta;

};

}
}

#endif /* INC_PREDICTION_H_ */
