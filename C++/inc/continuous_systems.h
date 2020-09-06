#ifndef CONTINUOUS_SYSTEMS_H_
#define CONTINUOUS_SYSTEMS_H_


#include "systems.h"
#include "vector_numeric.h"
#include "ode_solver.h"

namespace SystemControl {

class ContinuousSystemInterface {
public:

	ContinuousSystemInterface(size_t, real_t*, real_t*);
	inline void set_states(const real_t* X_ptr) {
		X.load_data(X_ptr);
	}
	inline void set_states(real_t X_value) {
		X.set_all(X_value);
	}
	inline size_t get_order() const {
		return X.get_length();
	}
	inline const VectorReal& get_derivatives() const {
		return dX;
	}
	inline VectorReal& get_states() {
		return X;
	}
	virtual void update_derivatives_fcn(real_t, step_type)=0;
	virtual void update_output_fcn(real_t, step_type)=0;

protected:
	VectorReal X;
	VectorReal dX;

};

class ContinuousSystem: public System ,public ContinuousSystemInterface{

public:
	ContinuousSystem(size_t, real_t*, size_t, real_t*, size_t, real_t*, real_t*);
	inline void update(real_t time, step_type step_type) {
		update_output_fcn(time, step_type);
	}
};

class ContinuousSystemSISO: public ContinuousSystem {
public:
	ContinuousSystemSISO(size_t, real_t*, real_t*);
private:
	real_t input;
	real_t output;
};

class ContinuousSystemMIMO: public ContinuousSystem {
public:
	ContinuousSystemMIMO(size_t, real_t*, size_t, real_t*, size_t, real_t*, real_t*);
private:
};

class ContinuousStateSpace: public StateSpace, public ContinuousSystemSISO {

public:
	ContinuousStateSpace(size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);

private:

	void update_derivatives_fcn(real_t, step_type);
	void update_output_fcn(real_t, step_type);
};

class ContinuousTransferFuction: public TransferFunction, public ContinuousSystemSISO {

public:
	ContinuousTransferFuction(size_t, size_t, real_t* = NULL, real_t* = NULL, real_t* = NULL, real_t* = NULL);

private:
	void update_derivatives_fcn(real_t, step_type);
	void update_output_fcn(real_t, step_type);
};

class ContinuousTransferFuctionFirstOrder: public ContinuousTransferFuction {
public:
	ContinuousTransferFuctionFirstOrder(System1stOrderParams&);
private:
	real_t numerator_data[1];
	real_t denominator_data[2];
	real_t dX_data[1];
	real_t X_data[1];
};

class ContinuousTransferFuctionSecondOrder: public ContinuousTransferFuction {
public:
	ContinuousTransferFuctionSecondOrder(System2ndOrderParams&);
private:
	real_t numerator_data[1];
	real_t denominator_data[3];
	real_t dX_data[2];
	real_t X_data[2];
};

class ContinuousIntegrator: public ContinuousSystemSISO {
public:
	ContinuousIntegrator();
private:
	real_t dX_data[1];
	real_t X_data[1];
	void update_derivatives_fcn(real_t, step_type);
	void update_output_fcn(real_t, step_type);
};

class ContinuousPID_regulator: public ContinuousSystemSISO, public PID_regulator_params {

public:
	ContinuousPID_regulator(real_t, real_t, real_t, real_t);

private:
	real_t N_gain;
	real_t dX_data[2];
	real_t X_data[2];
	void update_derivatives_fcn(real_t, step_type);
	void update_output_fcn(real_t, step_type);
};

class ContinuousNonlinearSystem: public ContinuousSystemSISO {

public:
	typedef std::function<void(real_t[], const real_t[], const real_t[], real_t, real_t)> derrivatives_fcn;
	typedef std::function<void(const real_t[], const real_t[], real_t, real_t*)> output_fcn;

	ContinuousNonlinearSystem(derrivatives_fcn, output_fcn, size_t n_states, size_t n_params, real_t* params_data);
private:
	Vector<real_t> params;
	derrivatives_fcn custom_derrivatives_fcn;
	output_fcn custom_update_output_fcn;

	void update_derivatives_fcn(real_t, step_type);
	void update_output_fcn(real_t, step_type);
};

}

#endif /* CONTINUOUS_SYSTEMS_H_ */
