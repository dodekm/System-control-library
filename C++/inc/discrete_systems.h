#ifndef DISCRETE_SYSTEMS_H_
#define DISCRETE_SYSTEMS_H_

#include "systems.h"

namespace SystemControl {

class Saturation {
public:
	inline real_t saturate(real_t y) {
		y = CLIP_TOP(y, saturation_high);
		y = CLIP_BOTTOM(y, saturation_low);
		return y;
	}
	inline void set_saturation(real_t saturation_high, real_t saturation_low) {
		if (saturation_high < saturation_low)
			throw exception_ERROR;
		if (saturation_high == 0.0 || saturation_low == 0.0)
			throw exception_ERROR;
		this->saturation_high = saturation_high;
		this->saturation_low = saturation_low;
	}
protected:
	real_t saturation_high = INFINITY;
	real_t saturation_low = -INFINITY;

};

class DiscreteAntiWindup: public Saturation {

public:
	inline void set_AW_gain(real_t AW_gain) {
		this->AW_gain = AW_gain;
	}
	inline real_t apply_AW_saturation(real_t u_original) {
		real_t u_saturated = saturate(u_original);
		u_cut = u_saturated - u_original;
		return u_saturated;
	}
	inline real_t get_AW_correction() {
		return u_cut * AW_gain;
	}
private:
	real_t AW_gain = 0;
	real_t u_cut = 0;

};
class DiscreteSystem: public System {

public:

	DiscreteSystem(size_t, real_t*, size_t, real_t*);

	inline void step(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {
		return step_function(input_signals, output_signals);
	}
	inline void step() {
		return step_function(input_port, output_port);
	}
	real_t step(real_t);
	inline void update(real_t time, step_type step_type) {
		if (step_type == step_major)
			step();
	}

protected:
	virtual void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&)=0;
};

class DiscreteSystemSISO: public DiscreteSystem {
public:
	DiscreteSystemSISO();
private:
	real_t input;
	real_t output;

};
class DiscreteSystemMIMO: public DiscreteSystem {
public:
	DiscreteSystemMIMO(size_t, real_t*, size_t, real_t*);
};

class StateBuffer: public Vector<real_t> {
public:
	StateBuffer(size_t length, real_t* data_ptr = NULL) :
			Vector<real_t>(length, data_ptr) {

	}
	inline void shift() {
		shift_idx++;
		shift_idx %= get_length();
	}
	inline void sample(real_t value) {
		Vector::at(shift_idx) = value;
	}
	inline real_t& at(uint n) {
		return Vector::at((n + shift_idx + 1) % get_length());
	}
	inline real_t& operator[](uint n) {
		return at(n);
	}
	inline void reset() {
		shift_idx = 0;
	}
private:
	uint shift_idx = 0;
};

class DiscreteTransferFunction: public DiscreteSystemSISO, public TransferFunction, public Saturation {

public:
	DiscreteTransferFunction(size_t, size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*, const real_t*);
	void states_set(real_t = 0, real_t = 0);
private:

	StateBuffer input_states;
	StateBuffer output_states;

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteStateSpace: public StateSpace, public DiscreteSystemSISO, public Saturation {

public:
	DiscreteStateSpace(size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*);
	void states_set(real_t = 0);
private:

	VectorReal Xk_0;
	VectorReal Xk_1;

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class Discrete_RST_Regulator: public DiscreteSystemMIMO, public Saturation {

public:
	Discrete_RST_Regulator(size_t, size_t, size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*, const real_t*, const real_t*);
	void states_set(real_t = 0, real_t = 0, real_t = 0);
	void coeffs_set(const real_t*, const real_t*, const real_t*);
	SubSystem create_control_loop(DiscreteSystemSISO&);
private:
	Vector<real_t> R_coeffs;
	Vector<real_t> S_coeffs;
	Vector<real_t> T_coeffs;

	StateBuffer u_states;
	StateBuffer y_states;
	StateBuffer w_states;
	real_t input_signals_data[2];
	real_t output_signals_data[1];

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteIIRfilterDFII: public TransferFunction, public DiscreteSystemSISO, public Saturation {

public:
	DiscreteIIRfilterDFII(size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*);
	void states_set(real_t = 0);
private:

	StateBuffer states;
	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteFIRfilter: public DiscreteSystemSISO, public Saturation {

public:
	DiscreteFIRfilter(size_t, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*);
	void states_set(real_t = 0);
	void coeffs_set(const real_t*);
private:

	Vector<real_t> coeffs;
	StateBuffer states;
	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteDelay: public DiscreteSystemSISO {

public:
	DiscreteDelay(size_t, real_t* = NULL);
private:
	StateBuffer states;
	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
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
	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
public:
	DiscreteBiquadSOSfilterDFI(size_t, discrete_biquad_section_coeffs_T* = NULL, discrete_biquad_section_states_DF_I_T* = NULL);
private:
	Vector<discrete_biquad_section_coeffs> coeffs;
	Vector<discrete_biquad_section_states_DF_I_T> states;

};

class DiscreteBiquadSOSfilterDFII: public DiscreteSystemSISO {

public:
	DiscreteBiquadSOSfilterDFII(size_t, discrete_biquad_section_coeffs_T* = NULL, discrete_biquad_section_states_DF_II_T* = NULL);
private:
	Vector<discrete_biquad_section_coeffs> coeffs;
	Vector<discrete_biquad_section_states_DF_II_T> states;
	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteTransferFunctionFirstOrder: public DiscreteTransferFunction {
	using DiscreteTransferFunction::DiscreteTransferFunction;
public:

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

private:
#define S_gain I_gain
	DiscreteSumator sumator;
	DiscreteDiference diference;

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteIntegrator: public DiscreteTransferFunctionFirstOrder {
public:
	DiscreteIntegrator(approximation_method, real_t, real_t = 0);
};

class DiscreteDerivative: public DiscreteTransferFunctionFirstOrder {
public:
	DiscreteDerivative(approximation_method, real_t, real_t, real_t = 0);
};

class DiscretePIDregulator: public DiscreteSystemSISO, public PID_regulator_params, public DiscreteAntiWindup {

public:
	DiscretePIDregulator(real_t, real_t, real_t, real_t, real_t, approximation_method);
private:
	DiscreteIntegrator integrator;
	DiscreteDerivative derivator;

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteIPregulator: public DiscreteSystemMIMO, public PID_regulator_params, public DiscreteAntiWindup {

public:
	DiscreteIPregulator(real_t, real_t, real_t, approximation_method);

private:
	DiscreteIntegrator integrator;
	real_t input_signals_data[2];
	real_t output_signals_data[1];

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};
}

#endif /* DISCRETE_SYSTEMS_H_ */
