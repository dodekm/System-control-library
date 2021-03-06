#ifndef SYSTEMS_H_
#define SYSTEMS_H_

#include "common_def.h"
#include "adt.h"
#include "vector.h"
#include "matrix.h"
#include "polynom.h"
#include "complex.h"

namespace SystemControl {

class Signal {

public:
	Signal() {
	}
	Signal(real_t* ptr, class System* owner = NULL) :
			owner(owner), ptr(ptr) {
	}
	Signal(const Signal& other) :
			Signal(other.ptr, other.owner) {
	}
	Signal& operator=(const Signal& other)
	{
		owner=other.owner;
		ptr=other.ptr;
		return *this;
	}
	real_t& operator=(real_t value) {
		*ptr = value;
		return *ptr;
	}
	operator real_t() const {
		return *ptr;
	}
	operator real_t&() {
		return *ptr;
	}
	operator real_t*() {
		return ptr;
	}
	operator const real_t*() const {
		return ptr;
	}

	bool operator==(const Signal& other)const {
		return this->owner == other.owner && this->ptr == other.ptr;
	}

	class System* owner = NULL;
private:
	real_t* ptr = NULL;
};

class System {

	friend class ContinuousSubsystem;
	friend class SubSystem;

public:

	inline Signal& input_port_struct(uint idx) {
		return input_port.at(idx);
	}
	inline Signal& output_port_struct(uint idx) {
		return output_port.at(idx);
	}
	inline real_t& input(uint idx) {
		return input_port.at(idx);
	}
	inline real_t& output(uint idx) {
		return output_port.at(idx);
	}
	inline real_t input(uint idx) const {
		return input_port.at(idx);
	}
	inline real_t output(uint idx) const {
		return output_port.at(idx);
	}

	inline size_t input_port_size() const {
		return input_port.get_length();
	}
	inline size_t output_port_size() const {
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
	Vector<Signal> input_port;
	Vector<Signal> output_port;

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

	void set_gain(real_t);
	real_t get_gain() const;
#ifdef USE_GSL
	void get_poles(VectorComplex&) const;
	void get_zeros(VectorComplex&) const;
	bool is_stable() const;
	void to_transfer_function(TransferFunction&) const;
	void to_zero_pole_gain(ZeroPoleGain&) const;
#endif

	void to_discrete_Taylor_series(StateSpace&, real_t, uint = 10) const;
	void to_discrete(StateSpace&, real_t) const;

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
	System1stOrderParams(real_t K = 0, real_t T = 0) :
			K(K), T(T) {
	}
	real_t K;
	real_t T;
};

class System2ndOrderParams {
public:
	System2ndOrderParams(real_t K = 0, real_t omega_0 = 0, real_t b = 0) :
			K(K), omega_0(omega_0), b(b) {
	}
	real_t K;
	real_t omega_0;
	real_t b;
};

class ReferencePolynom2ndOrder {
public:
	ReferencePolynom2ndOrder(real_t omega_0 = 0, real_t b = 0) :
			omega_0(omega_0), b(b) {
	}
	real_t omega_0;
	real_t b;
};

class ReferencePolynom3rdOrder: public ReferencePolynom2ndOrder {
public:
	real_t k;
};

class PID_regulator_params {
public:
	PID_regulator_params(real_t P_gain = 0, real_t I_gain = 0, real_t D_gain = 0) :
			P_gain(P_gain), I_gain(I_gain), D_gain(D_gain) {
	}

	union {
		struct {
			real_t P_gain;
			real_t I_gain;
			real_t D_gain;
		};
		real_t data[3];
	};
	PID_regulator_params& get_params() {
		return *this;
	}
	const PID_regulator_params& get_params() const {
		return *this;
	}
};

}

#endif /* SYSTEMS_H_ */
