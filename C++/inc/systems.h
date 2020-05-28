#ifndef SYSTEMS_H_
#define SYSTEMS_H_

#include "common_def.h"
#include "adt.h"
#include "vector.h"
#include "matrix.h"
#include "polynom.h"
#include "complex.h"

namespace SystemControl {

typedef struct signal_realtime {
	real_t* ptr;
	class System* owner;
} signal_realtime_T;

/**
 * Common control loop signals structure.
 */
typedef struct {
	uint64_t tick; ///<sample counter
	real_t w; ///<control setpoint
	real_t y; ///<system output
	real_t u; ///<control output
} control_loop_signals_T;

class System {

	friend class ContinuousSubsystem;
	friend class SubSystem;

public:

	inline signal_realtime_T& input_port_struct(uint idx) {
		return input_port.at(idx);
	}
	inline signal_realtime_T& output_port_struct(uint idx) {
		return output_port.at(idx);
	}
	inline real_t& input_port_real_value(uint idx) {
		return *input_port_struct(idx).ptr;
	}
	inline real_t& output_port_real_value(uint idx) {
		return *output_port_struct(idx).ptr;
	}
	inline size_t input_port_size() {
		return input_port.get_length();
	}
	inline size_t output_port_size() {
		return output_port.get_length();
	}
	virtual void update(real_t, step_type)=0;

	typedef enum {
		system_type_uninitialized, system_type_static_system = 1, system_type_dynamic_system, system_type_signal_source, system_type_signal_sink, system_type_subsystem
	} system_type;

	typedef enum {
		system_time_domain_uninitialized, system_time_domain_discrete = 1, system_time_domain_continuous
	} system_time_domain;

protected:
	Vector<signal_realtime_T> input_port;
	Vector<signal_realtime_T> output_port;

	system_type type = system_type_uninitialized;
	system_time_domain time_domain = system_time_domain_uninitialized;

	System(system_type, system_time_domain, size_t, real_t*, size_t, real_t*);
	~System();
private:
	void assign_port_pointers(real_t* = NULL, real_t* = NULL);

};

class SubSystem: public System {

	using System::System;
public:

	static SubSystem connect_serial(Vector<System*>&, Vector<uint>&, Vector<uint>&);
	static SubSystem connect_paralel(Vector<System*>&, System&);
	static SubSystem connect_feedback(System&, System&, System&);

	void update(real_t, step_type);

	List<System*> execution_list;
};

class StateSpace;
class TransferFunction;
class ZeroPoleGain;

class StateSpace {
	friend class TransferFunction;
	friend class ZeroPoleGain;
public:
	StateSpace() {
	}
	StateSpace(size_t order, System::system_time_domain time_domain, real_t* A_data_ptr = NULL, real_t* B_data_ptr = NULL, real_t* C_data_ptr = NULL) :
			A(order, order, A_data_ptr), B(order, B_data_ptr), C(order, C_data_ptr), time_domain(time_domain) {
	}
	inline size_t get_order() const {
		return A.get_n_rows();
	}
	void assert() const;
	void coeffs_set(const real_t*, const real_t*, const real_t*);
#ifdef USE_GSL
	void set_gain(real_t);
	real_t get_gain() const;
	void get_poles(VectorComplex&) const;
	void get_zeros(VectorComplex&) const;
	bool is_stable() const;
#endif

	void to_discrete_Taylor_series(StateSpace&, real_t, uint) const;
#ifdef USE_GSL
	void to_discrete(StateSpace&, real_t) const;
	void to_transfer_function(TransferFunction&) const;
	void to_zero_pole_gain(ZeroPoleGain&) const;
#endif

protected:
	Matrix A;
	VectorReal B;
	VectorReal C;
	System::system_time_domain time_domain;
};

class TransferFunction {
	friend class ZeroPoleGain;
	friend class StateSpace;
public:
	TransferFunction() {
	}
	TransferFunction(size_t numerator_order, size_t denominator_order, System::system_time_domain time_domain, real_t* numerator_coeffs_ptr = NULL, real_t* denominator_coeffs_ptr = NULL) :
			numerator_coeffs(numerator_order + 1, numerator_coeffs_ptr), denominator_coeffs(denominator_order + 1, denominator_coeffs_ptr), time_domain(time_domain) {
	}
	inline size_t get_order() const {
		return denominator_coeffs.get_length() - 1;
	}
	void assert() const;
	void coeffs_set(const real_t*, const real_t*);

	bool is_stable() const;
	real_t get_gain() const;
	void set_gain(real_t);
	void normalize();

	void get_poles(VectorComplex&) const;
	void get_zeros(VectorComplex&) const;

	void to_state_space(StateSpace&) const;
	void to_zero_pole_gain(ZeroPoleGain&) const;

	void to_discrete_forward_Euler(TransferFunction&, real_t) const;
	void to_discrete_bilinear(TransferFunction&, real_t) const;
	void to_discrete_match_zeros_poles(TransferFunction&, real_t) const;

	void serial_transfer_function_(const TransferFunction&, const TransferFunction&);
	void paralel_transfer_function(const TransferFunction&, const TransferFunction&);
	void feedback_transfer_function(const TransferFunction&, const TransferFunction&);

protected:

	Polynom numerator_coeffs;
	Polynom denominator_coeffs;
	System::system_time_domain time_domain;
};

class ZeroPoleGain {
	friend class TransferFunction;
	friend class StateSpace;
public:
	ZeroPoleGain() {
	}
	ZeroPoleGain(size_t n_zeros, size_t n_poles, System::system_time_domain time_domain, real_t gain, Complex* zeros_ptr = NULL, Complex* poles_ptr = NULL) :
			zeros(n_zeros, zeros_ptr), poles(n_poles, poles_ptr), time_domain(time_domain), gain(gain) {

	}
	inline size_t get_order() const {
		return poles.get_length();
	}
	void assert() const;

	void to_discrete(ZeroPoleGain&, real_t) const;
	void to_transfer_function(TransferFunction&) const;
	bool is_stable() const;

protected:
	VectorComplex zeros;
	VectorComplex poles;
	System::system_time_domain time_domain;
	real_t gain = 0;
};

class System1stOrderParams {
public:
	real_t K;
	real_t T;
};

class System2ndOrderParams {
public:
	real_t K;
	real_t omega_0;
	real_t b;
};

class ReferencePolynom2ndOrder {
public:
	real_t omega_0;
	real_t b;
};

class ReferencePolynom3rdOrder: public ReferencePolynom2ndOrder {
public:
	real_t k;
};

class PID_regulator_params {
public:
	real_t P_gain = 0;
	real_t I_gain = 0;
	real_t D_gain = 0;
	PID_regulator_params& get_params() {
		return *this;
	}
};

}

#endif /* SYSTEMS_H_ */
