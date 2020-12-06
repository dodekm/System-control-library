#include "prediction.h"

using namespace SystemControl;
using namespace Prediction;

void GPC::fill_M_yf(Matrix& M_yf, const VectorReal& A) {

	M_yf.assert();
	A.assert();
#ifdef ASSERT_DIMENSIONS
	if (!M_yf.is_square())
	throw exception_WRONG_DIMENSIONS;
#endif

	const VectorRealReverse A_rev(A, A.get_length(), 0);
	int n = M_yf.get_n_rows();
	for (int i = 0; i < n; i++) {
		int j_end = MIN(i + 1, (int )A_rev.get_length());
		for (int j = 0; j < j_end; j++) {
			M_yf(i, i - j) = A_rev[j];
		}
	}

}
void GPC::fill_M_uf(Matrix& M_uf, const VectorReal& B) {

	M_uf.assert();
	B.assert();
#ifdef ASSERT_DIMENSIONS
	if (!M_uf.is_square())
	throw exception_WRONG_DIMENSIONS;
#endif

	const VectorRealReverse B_rev(B, B.get_length(), 0);
	int n = M_uf.get_n_rows();
	for (int i = 0; i < n; i++) {
		int j_end = MIN(i + 1, (int )B_rev.get_length() - 1);
		for (int j = 0; j < j_end; j++) {
			M_uf(i, i - j) = B_rev[j + 1];
		}
	}

}
void GPC::fill_M_yp(Matrix& M_yp, const VectorReal& A) {

	M_yp.assert();
	A.assert();

	const VectorRealReverse A_rev(A, A.get_length(), 0);
	int n = (int) M_yp.get_n_rows();
	int m = (int) M_yp.get_n_cols();

	for (int i = 0; i < n; i++) {
		int j_end = MIN(m, (int )A_rev.get_length() - i - 1);
		for (int j = 0; j < j_end; j++) {
			M_yp(i, m - 1 - j) = A_rev[i + j + 1];
		}
	}

}
void GPC::fill_M_up(Matrix& M_up, const VectorReal& B) {

	M_up.assert();
	B.assert();

	const VectorRealReverse B_rev(B, B.get_length(), 0);
	int n = (int) M_up.get_n_rows();
	int m = (int) M_up.get_n_cols();

	for (int i = 0; i < n; i++) {
		int j_end = MIN(m, (int )B_rev.get_length() - i - 2);
		for (int j = 0; j < j_end; j++) {
			M_up(i, m - 1 - j) = B_rev[i + j +  2];
		}
	}

}

void GPC::fill_H_matrix(Matrix& H, const VectorReal& B, const VectorReal& A) {

	H.assert();
	B.assert();
	A.assert();

	uint n_e2 = H.get_n_rows();
	Matrix M_yf(n_e2, n_e2);
	MatrixPermRow M_yf_LU(M_yf);
	fill_M_yf(M_yf, A);
	M_yf_LU.LU_decompose();
	Matrix inv_M_yf(n_e2, n_e2);
	M_yf_LU.LU_inv(inv_M_yf);
	Matrix M_uf(n_e2, n_e2);
	fill_M_uf(M_uf, B);
	H.multiply(inv_M_yf, M_uf);

	for (uint i = 1; i < n_e2; i++) {
		H.row(i) += H.row(i - 1);
	}

}

void GPC::predict(VectorReal& y_f, const VectorReal& u_f, const VectorReal& y_p, const VectorReal& u_p, const VectorReal& B, const VectorReal& A) {

#ifdef ASSERT_DIMENSIONS
	if (u_f.get_length() != y_f.get_length())
	throw exception_WRONG_DIMENSIONS;
#endif

	int n = u_f.get_length();

	Matrix M_yf(n, n);
	Matrix M_uf(n, n);
	Matrix M_yp(n, y_p.get_length());
	Matrix M_up(n, u_p.get_length());

	fill_M_yf(M_yf, A);
	fill_M_uf(M_uf, B);
	fill_M_yp(M_yp, A);
	fill_M_up(M_up, B);

	VectorReal R(n);

	M_up.multiply(u_p, R, 1);
	M_uf.multiply(u_f, R, 1);
	M_yp.multiply(y_p, R, -1);
	R *= -1.0;
	M_yf.solve(R, y_f);

}

void GPC::predict(VectorReal& y_f, const VectorReal& u_f, const VectorReal& vm_f, real_t vu_k, const VectorReal& y_p, const VectorReal& u_p, const VectorReal& vm_p, const VectorReal& vu_p, const VectorReal& B, const VectorReal& B_dist, const VectorReal& A, const VectorReal& T) {

#ifdef ASSERT_DIMENSIONS
	if (u_f.get_length() != vm_f.get_length())
	throw exception_WRONG_DIMENSIONS;
	if (u_f.get_length() != y_f.get_length())
	throw exception_WRONG_DIMENSIONS;
#endif

	int n = u_f.get_length();

	Matrix M_yf(n, n);
	Matrix M_uf(n, n);
	Matrix M_vmf(n, n);
	Matrix M_vuf(n, n);
	Matrix M_yp(n, y_p.get_length());
	Matrix M_up(n, u_p.get_length());
	Matrix M_vmp(n, vm_p.get_length());
	Matrix M_vup(n, vu_p.get_length());

	fill_M_yf(M_yf, A);
	fill_M_uf(M_uf, B);
	fill_M_uf(M_vmf, B_dist);
	fill_M_uf(M_vuf, T);
	fill_M_yp(M_yp, A);
	fill_M_up(M_up, B);
	fill_M_up(M_vmp, B_dist);
	fill_M_up(M_vup, T);

	VectorReal vu_f(n);
	vu_f.set_all(vu_k);

	VectorReal R(n);
	M_up.multiply(u_p, R, 1);
	M_vmp.multiply(vm_p, R, 1);
	M_vup.multiply(vu_p, R, 1);
	M_uf.multiply(u_f, R, 1);
	M_vmf.multiply(vm_f, R, 1);
	M_vuf.multiply(vu_f, R, 1);
	M_yp.multiply(y_p, R, -1);
	R *= -1.0;
	R += vu_k;
	M_yf.solve(R, y_f);

}

real_t GPC::operator()(real_t y_k, const VectorReal& y_ref, const VectorReal& vm_f) {

	real_t y_k_hat = (VectorReal(B, B.get_length() - 1, 0).dot_product(u_p) + VectorReal(B_dist, B_dist.get_length() - 1, 0).dot_product(vm_p) + VectorReal(T, T.get_length() - 1, 0).dot_product(vu_p) - VectorReal(A, A.get_length() - 1, 0).dot_product(y_p));
	v_uk = y_k - y_k_hat;

	++y_p;
	y_p.at() = y_k;
	u_f.set_all(u_k);

	predict(y_free, u_f, vm_f, v_uk, y_p, u_p, vm_p, vu_p, B, B_dist, A, T);
	const VectorReal y_free_cut(y_free, n_e2 - n_e1, n_e1);
	const VectorReal y_ref_cut(y_ref, n_e2 - n_e1, n_e1);

	e.sub(y_ref_cut, y_free_cut);

	HT_lambda_y.multiply(e, HTe);
	invHTH.multiply(HTe, du_f);

	Matrix(H, H.get_n_rows(), du_f.get_length(), 0, 0).multiply(du_f, y_forced);

	u_k += du_f[0];
	u_k = modify_output(u_k);

	++u_p;
	u_p.at() = u_k;

	++vm_p;
	vm_p.at() = vm_f[0];

	++vu_p;
	vu_p.at() = v_uk;

	return u_k;
}

void DiscretePredictor1input::predict(const VectorReal& u_vector, VectorReal& y_vector) {
#ifdef ASSERT_DIMENSIONS
	if (u_vector.get_length() != y_vector.get_length())
	throw exception_code(exception_WRONG_DIMENSIONS);
#endif

	VectorDiscreteRegressor h(h);
	for (uint i = 0; i < y_vector.get_length(); i++) {
		real_t y = h.dot_product(theta);
		y_vector[i] = y;
		h.update(u_vector[i], y);
	}
}

void DiscretePredictor2input::predict(const VectorReal& u_1_vector, const VectorReal& u_2_vector, VectorReal& y_vector) {
#ifdef ASSERT_DIMENSIONS
	if (u_1_vector.get_length() != y_vector.get_length() || u_2_vector.get_length() != y_vector.get_length())
	throw exception_code(exception_WRONG_DIMENSIONS);
#endif

	VectorDiscreteRegressor2Input h(h);
	for (uint i = 0; i < y_vector.get_length(); i++) {
		real_t y = h.dot_product(theta);
		y_vector[i] = y;
		h.update(u_1_vector[i], u_2_vector[i], y);
	}
}

