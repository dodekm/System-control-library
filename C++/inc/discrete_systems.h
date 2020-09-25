#ifndef DISCRETE_SYSTEMS_H_
#define DISCRETE_SYSTEMS_H_

#include "systems.h"

namespace SystemControl {

class Saturation {
public:

	void set_saturation(real_t saturation_high, real_t saturation_low) {
		if (saturation_high < saturation_low)
			throw exception_ERROR;
		if (saturation_high == 0.0 || saturation_low == 0.0)
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
	virtual real_t modify_output(real_t y) {
		return y;
	}
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

class VectorStates: public VectorReal {
public:
	using VectorReal::VectorReal;
	inline void operator++() {
		idx++;
		idx %= get_length();
	}
	inline real_t& at(uint n) {
		return Vector::at((n + idx + 1) % get_length());
	}
	inline real_t& at() {
		return Vector::at(idx);
	}
	inline real_t at(uint n) const {
		return Vector::at((n + idx + 1) % get_length());
	}
	inline real_t at() const {
		return Vector::at(idx);
	}
	inline real_t& operator[](uint n) {
		return at(n);
	}

	inline real_t operator[](uint n) const {
		return at(n);
	}
	inline void reset() {
		idx = 0;
	}
private:
	uint idx = 0;
};

class DiscreteTransferFunction: public DiscreteSystemSISO, public TransferFunction {

public:
	DiscreteTransferFunction(size_t, size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*, const real_t*);
	void states_set(real_t = 0, real_t = 0);
private:

	VectorStates input_states;
	VectorStates output_states;

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);

};

class DiscreteStateSpace: public  StateSpace, public DiscreteSystemSISO {

public:
	DiscreteStateSpace(size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);
	void states_set(const real_t*);
	void states_set(real_t = 0);
private:

	VectorReal Xk_0;
	VectorReal Xk_1;

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);

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

	VectorStates u_states;
	VectorStates y_states;
	VectorStates w_states;
	real_t input_signals_data[2];
	real_t output_signals_data[1];

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
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
	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
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

	Vector<real_t> coeffs;
	VectorStates states;
	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteDelay: public DiscreteSystemSISO {

public:
	DiscreteDelay(size_t, real_t* = NULL);
private:
	VectorStates states;
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

public:
	DiscreteBiquadSOSfilterDFI(size_t, discrete_biquad_section_coeffs_T* = NULL, discrete_biquad_section_states_DF_I_T* = NULL);
private:
	Vector<discrete_biquad_section_coeffs> coeffs;
	Vector<discrete_biquad_section_states_DF_I_T> states;
	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);

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

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
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

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscretePIregulator: public DiscreteSystemSISO, public PID_regulator_params, public DiscreteAntiWindup {

public:
	DiscretePIregulator(real_t, real_t, real_t, approximation_method);
	DiscreteIntegrator integrator;
private:

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteIPregulator: public DiscreteSystemMIMO, public PID_regulator_params, public DiscreteAntiWindup {

public:
	DiscreteIPregulator(real_t, real_t, real_t, approximation_method);
	DiscreteIntegrator integrator;
private:

	real_t input_signals_data[2];
	real_t output_signals_data[1];

	void step_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class DiscreteTransferFunctionRegressor {

	DiscreteTransferFunctionRegressor(uint nb,uint na):u_states(nb),y_states(na)
	{

	}

public:
	void iterate(real_t u, real_t y)
	{
		u_states.at()=u;
		++u_states;
		y_states.at()=-y;
		++y_states;
	}
	real_t& at(uint n) {
		uint nb=u_states.get_length();
		uint na=y_states.get_length();
		if(n<nb)
			return u_states[n];
		else if(n<na+nb)
			return y_states[n-nb];
		else
			throw exception_code(exception_INDEX_OUT_OF_RANGE);
	}
	real_t& operator[](uint n){
		return at(n);
	}
	inline size_t get_length() const {
		return u_states.get_length()+y_states.get_length();
	}

private:
	VectorStates u_states;
	VectorStates y_states;

};

void discrete_transfer_function_vector_h_update(VectorReal&, real_t, real_t, size_t, size_t);

}

#endif /* DISCRETE_SYSTEMS_H_ */
