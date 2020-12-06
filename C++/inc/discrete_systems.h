#ifndef DISCRETE_SYSTEMS_H_
#define DISCRETE_SYSTEMS_H_

#include "systems.h"

namespace SystemControl {

class Saturation {
public:

	void set_saturation(real_t saturation_high, real_t saturation_low) {
		if (saturation_high < saturation_low)
			throw exception_ERROR;
		if (saturation_high == saturation_low)
			throw exception_ERROR;
		this->saturation_high = saturation_high;
		this->saturation_low = saturation_low;
	}
protected:
	real_t saturate(real_t y) {
		y = CLIP_TOP(y, saturation_high);
		y = CLIP_BOTTOM(y, saturation_low);
		return y;
	}
	real_t modify_output(real_t y) {
		return saturate(y);
	}

	real_t saturation_high = INFINITY;
	real_t saturation_low = -INFINITY;

};

class DiscreteAntiWindup: public Saturation {

public:
	inline void set_AW_gain(real_t AW_gain) {
		this->AW_gain = AW_gain;
	}

protected:
	real_t modify_output(real_t u_original) {
		real_t u_saturated = saturate(u_original);
		u_cut = u_saturated - u_original;
		return u_saturated;
	}
	real_t get_AW_correction() {
		return u_cut * AW_gain;
	}
private:
	real_t AW_gain = 0;
	real_t u_cut = 0;

};
class DiscreteSystem: public System {

public:

	DiscreteSystem(size_t, real_t*, size_t, real_t*);

	inline void step(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
		return step_function(input_signals, output_signals);
	}
	inline void step() {
		return step_function(input_port, output_port);
	}
	real_t step(real_t);
	real_t step(real_t, real_t);

	inline void update(real_t time, step_type step_type) {
		if (step_type == step_major)
			step();
	}

protected:
	virtual void step_function(const Vector<Signal>&, Vector<Signal>&)=0;
	virtual real_t modify_output(real_t y) {
		return y;
	}
};

class DiscreteSystemSISO: public DiscreteSystem {
public:
	DiscreteSystemSISO();
private:
	real_t input_data;
	real_t output_data;

};
class DiscreteSystemMIMO: public DiscreteSystem {
public:
	DiscreteSystemMIMO(size_t, real_t*, size_t, real_t*);
};

class VectorStates: public VectorReal {
public:
	using VectorReal::VectorReal;

	VectorStates(const VectorStates& other) :
			VectorReal(other), idx(other.idx) {
	}

	VectorStates& operator=(const VectorStates& vectorSrc) throw (exception_code) {
		idx = vectorSrc.idx;
		(VectorReal &) *this = (const VectorReal&) vectorSrc;
		return *this;
	}

	inline void operator++() {
		idx++;
		idx %= get_length();
	}
	inline real_t& at(uint n) {
		return Vector::at((n + idx + 1) % length);
	}
	inline real_t& at() {
		return Vector::at(idx);
	}
	inline real_t at() const {
		return Vector::at(idx);
	}
	inline void reset() {
		idx = length - 1;
	}
private:
	uint idx = length - 1;
};

class DiscreteTransferFunction: public DiscreteSystemSISO, public TransferFunction {

public:
	DiscreteTransferFunction(size_t, size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*, const real_t*);
	void states_set(real_t = 0, real_t = 0);
private:

	VectorStates input_states;
	VectorStates output_states;

	void step_function(const Vector<Signal>&, Vector<Signal>&);

};

class DiscreteStateSpace: public StateSpace, public DiscreteSystemSISO {

public:
	DiscreteStateSpace(size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*);
	void states_set(real_t = 0);
private:

	VectorReal Xk_0;
	VectorReal Xk_1;

	void step_function(const Vector<Signal>&, Vector<Signal>&);

};

class DiscreteLuenbergObserver: public StateSpace, public DiscreteSystemMIMO {

public:
	DiscreteLuenbergObserver(size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*);
	void states_set(real_t = 0);
	const VectorReal& get_estimate() const {
		return Xk_1;
	}
	real_t get_error() const {
		return e;
	}
private:
	VectorReal L;
	VectorReal Xk_0;
	VectorReal Xk_1;
	real_t e = 0;
	real_t input_signals_data[2];
	real_t output_signals_data[1];

	void step_function(const Vector<Signal>&, Vector<Signal>&);

};

class Discrete_RST_Controller: public DiscreteSystemMIMO, public Saturation {

public:
	Discrete_RST_Controller(size_t, size_t, size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*, const real_t*, const real_t*);
	void states_set(real_t = 0, real_t = 0, real_t = 0);
	void coeffs_set(const real_t*, const real_t*, const real_t*);
	SubSystem create_control_loop(DiscreteSystemSISO&);

	VectorReal R_coeffs;
	VectorReal S_coeffs;
	VectorReal T_coeffs;

private:

	VectorStates u_states;
	VectorStates y_states;
	VectorStates w_states;
	real_t input_signals_data[2];
	real_t output_signals_data[1];

	void step_function(const Vector<Signal>&, Vector<Signal>&);
	real_t modify_output(real_t y) {
		return Saturation::modify_output(y);
	}
};

class DiscreteIIRfilterDFII: public TransferFunction, public DiscreteSystemSISO, public Saturation {

public:
	DiscreteIIRfilterDFII(size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*);
	void states_set(real_t = 0);
private:

	VectorStates states;
	void step_function(const Vector<Signal>&, Vector<Signal>&);
	real_t modify_output(real_t y) {
		return Saturation::modify_output(y);
	}
};

class DiscreteFIRfilter: public DiscreteSystemSISO {

public:
	DiscreteFIRfilter(size_t, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*);
	void states_set(real_t = 0);
	void coeffs_set(const real_t*);
private:

	VectorReal coeffs;
	VectorStates states;
	void step_function(const Vector<Signal>&, Vector<Signal>&);
};

class DiscreteDelay: public DiscreteSystemSISO {

public:
	DiscreteDelay(size_t, real_t* = NULL);
private:
	VectorStates states;
	void step_function(const Vector<Signal>&, Vector<Signal>&);

};

typedef struct discrete_biquad_section_coeffs {
	real_t b_coeffs[3];
	real_t a_coeffs[3];
} discrete_biquad_section_coeffs_T;

typedef struct discrete_biquad_section_states_DF_I {
	real_t input_states[2];
	real_t output_states[2];
} discrete_biquad_section_states_DF_I_T;

typedef struct discrete_biquad_section_states_DF_II {
	real_t v_states[2];
} discrete_biquad_section_states_DF_II_T;

class DiscreteBiquadSOSfilterDFI: public DiscreteSystemSISO {

public:
	DiscreteBiquadSOSfilterDFI(size_t, discrete_biquad_section_coeffs_T* = NULL, discrete_biquad_section_states_DF_I_T* = NULL);
private:
	Vector<discrete_biquad_section_coeffs> coeffs;
	Vector<discrete_biquad_section_states_DF_I_T> states;
	void step_function(const Vector<Signal>&, Vector<Signal>&);

};

class DiscreteBiquadSOSfilterDFII: public DiscreteSystemSISO {

public:
	DiscreteBiquadSOSfilterDFII(size_t, discrete_biquad_section_coeffs_T* = NULL, discrete_biquad_section_states_DF_II_T* = NULL);
private:
	Vector<discrete_biquad_section_coeffs> coeffs;
	Vector<discrete_biquad_section_states_DF_II_T> states;
	void step_function(const Vector<Signal>&, Vector<Signal>&);

};

class DiscreteTransferFunctionFirstOrder: public DiscreteTransferFunction {
	using DiscreteTransferFunction::DiscreteTransferFunction;
public:
	DiscreteTransferFunctionFirstOrder() :
			DiscreteTransferFunction(1, 1, numerator_coeffs_data, denominator_coeffs_data, input_states_data, output_states_data) {
	}
protected:
	real_t input_states_data[2];
	real_t output_states_data[2];
	real_t numerator_coeffs_data[2];
	real_t denominator_coeffs_data[2];
private:
};

class DiscreteSumator: public DiscreteTransferFunctionFirstOrder {
public:
	DiscreteSumator(real_t = 0);
};

class DiscreteDiference: public DiscreteTransferFunctionFirstOrder {
public:
	DiscreteDiference(real_t = 0);
};

class DiscretePSDregulator: public DiscreteSystemSISO, public PID_regulator_params, public DiscreteAntiWindup {

public:
	DiscretePSDregulator(real_t, real_t, real_t);
	DiscreteSumator sumator;
	DiscreteDiference diference;
private:
#define S_gain I_gain

	void step_function(const Vector<Signal>&, Vector<Signal>&);
};

class DiscreteIntegrator: public DiscreteTransferFunctionFirstOrder {
public:
	DiscreteIntegrator(approximation_method, real_t, real_t = 0);
};

class DiscreteIntegratorLimited: public DiscreteIntegrator, public Saturation {
	using DiscreteIntegrator::DiscreteIntegrator;
private:
	real_t modify_output(real_t y) {
		return Saturation::modify_output(y);
	}

};

class DiscreteDerivative: public DiscreteTransferFunctionFirstOrder {
public:
	DiscreteDerivative(approximation_method, real_t, real_t = 10.0, real_t = 0);
};

class DiscretePIDregulator: public DiscreteSystemSISO, public PID_regulator_params, public DiscreteAntiWindup {

public:
	DiscretePIDregulator(real_t, real_t, real_t, real_t, real_t, approximation_method);
	DiscreteIntegrator integrator;
	DiscreteDerivative derivator;
private:

	void step_function(const Vector<Signal>&, Vector<Signal>&);
};

class DiscretePIregulator: public DiscreteSystemSISO, public PID_regulator_params, public DiscreteAntiWindup {

public:
	DiscretePIregulator(real_t, real_t, real_t, approximation_method);
	DiscreteIntegrator integrator;
private:

	void step_function(const Vector<Signal>&, Vector<Signal>&);
};

class DiscreteIPregulator: public DiscreteSystemMIMO, public PID_regulator_params, public DiscreteAntiWindup {

public:
	DiscreteIPregulator(real_t, real_t, real_t, approximation_method);
	DiscreteIntegrator integrator;
private:

	real_t input_signals_data[2];
	real_t output_signals_data[1];

	void step_function(const Vector<Signal>&, Vector<Signal>&);
};

class VectorDiscreteRegressor: public VectorReal {

public:
	VectorDiscreteRegressor(uint nb, uint na) :
			VectorReal(nb + na), u_states(nb, data_ptr), y_states(na, data_ptr + nb) {
	}
	VectorDiscreteRegressor(const VectorDiscreteRegressor& other) :
			VectorDiscreteRegressor(other.u_states.get_length(), other.y_states.get_length()) {
		u_states = other.u_states;
		y_states = other.y_states;
	}
	void update(real_t, real_t);
	real_t& at(uint);

	inline size_t get_length() const {
		return u_states.get_length() + y_states.get_length();
	}
private:
	VectorStates u_states;
	VectorStates y_states;

};

class VectorDiscreteRegressor2Input: public VectorReal {

public:
	VectorDiscreteRegressor2Input(uint nb_1, uint nb_2, uint na) :
			VectorReal(nb_1 + nb_2 + na), u1_states(nb_1, data_ptr), u2_states(nb_2, data_ptr + nb_1), y_states(na, data_ptr + nb_1 + nb_2) {
	}
	VectorDiscreteRegressor2Input(const VectorDiscreteRegressor2Input& other) :
			VectorDiscreteRegressor2Input(other.u1_states.get_length(), other.u2_states.get_length(), other.y_states.get_length()) {
		u1_states = other.u1_states;
		u2_states = other.u2_states;
		y_states = other.y_states;
	}

	void update(real_t, real_t, real_t);
	real_t& at(uint);

private:
	VectorStates u1_states;
	VectorStates u2_states;
	VectorStates y_states;

};

class VectorDiscreteParameters: public VectorReal {

public:
	VectorDiscreteParameters(VectorReal& B, VectorReal& A) :
			VectorReal(B.get_length() - 1 + A.get_length() - 1, B.get_data_ptr()), B(B), A(A) {
	}

	real_t& at(uint);
private:
	VectorReal& B;
	VectorReal& A;

};

class VectorDiscreteParameters2Input: public VectorReal {

public:
	VectorDiscreteParameters2Input(VectorReal& B_1, VectorReal& B_2, VectorReal& A) :
			VectorReal(B_1.get_length() - 1 + B_2.get_length() - 1 + A.get_length() - 1, B_1.get_data_ptr()), B_1(B_1), B_2(B_2), A(A) {
	}

	real_t& at(uint);

private:
	VectorReal& B_1;
	VectorReal& B_2;
	VectorReal& A;

};

}

#endif /* DISCRETE_SYSTEMS_H_ */
