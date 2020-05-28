#ifndef STATIC_SYSTEMS_H_
#define STATIC_SYSTEMS_H_

#include "systems.h"

namespace SystemControl {

class StaticSystem: public System {

public:
	StaticSystem(size_t, real_t*, size_t, real_t*);

	inline void eval(const Vector<signal_realtime_T>& input_signals, Vector<signal_realtime_T>& output_signals) {
		return eval_function(input_signals, output_signals);
	}
	inline void eval() {
		return eval_function(input_port, output_port);
	}
	real_t eval(real_t);

	inline void update(real_t time, step_type step_type) {
		eval();
	}

protected:
	virtual void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&)=0;
};

class StaticSystemSISO: public StaticSystem {
public:
	StaticSystemSISO();

private:
	real_t input;
	real_t output;
};

class StaticSystemMIMO: public StaticSystem {
public:
	StaticSystemMIMO(size_t, real_t*, size_t, real_t*);
};

class StaticSystemSaturation: public StaticSystemSISO {

public:
	StaticSystemSaturation(real_t lower_limit, real_t upper_limit) :
			StaticSystemSISO() {
		this->lower_limit = lower_limit;
		this->upper_limit = upper_limit;
	}

private:
	real_t lower_limit;
	real_t upper_limit;

	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class StaticSystemDeadzone: public StaticSystemSISO {

public:
	StaticSystemDeadzone(real_t treshold_low, real_t treshold_high) :
			StaticSystemSISO() {
		this->treshold_low = treshold_low;
		this->treshold_high = treshold_high;
	}

private:
	real_t treshold_low;
	real_t treshold_high;
	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class StaticSystemGain: public StaticSystemSISO {

public:
	StaticSystemGain(real_t gain) :
			StaticSystemSISO() {
		this->gain = gain;
	}

private:
	real_t gain;
	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class StaticSystemQuantize: public StaticSystemSISO {

public:
	StaticSystemQuantize(real_t step_size) :
			StaticSystemSISO() {
		this->step_size = step_size;
	}

private:
	real_t step_size;
	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class StaticSystemRelay: public StaticSystemSISO {

public:
	StaticSystemRelay(real_t treshold_switch_on, real_t treshold_switch_off, real_t value_on, real_t value_off) :
			StaticSystemSISO() {
		this->treshold_switch_on = treshold_switch_on;
		this->treshold_switch_off = treshold_switch_off;
		this->value_on = value_on;
		this->value_off = value_off;
		this->y_last = 0;
	}

private:
	real_t treshold_switch_on;
	real_t treshold_switch_off;
	real_t value_on;
	real_t value_off;
	real_t y_last;
	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);
};

class StaticSystem2ISO: public StaticSystemMIMO {
public:
	StaticSystem2ISO() :
			StaticSystemMIMO(2, input, 1, output) {
	}

private:
	real_t input[2];
	real_t output[1];

};

class StaticSystemSum: public StaticSystem2ISO {
	using StaticSystem2ISO::StaticSystem2ISO;
private:
	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);

};

class StaticSystemSub: public StaticSystem2ISO {
	using StaticSystem2ISO::StaticSystem2ISO;
private:
	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);

};

class StaticSystemProduct: public StaticSystem2ISO {
	using StaticSystem2ISO::StaticSystem2ISO;
private:
	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);

};

class DynamicSizeInputSignals {
protected:
	DynamicSizeInputSignals(size_t input_port_size) :
			input_signals(input_port_size) {
	}
	Vector<real_t> input_signals;
};

class StaticSystemPortOperator: private DynamicSizeInputSignals, public StaticSystemMIMO {

public:

	StaticSystemPortOperator(size_t input_port_size, Operators::binary_operator<real_t> operators_arr[]) :
			DynamicSizeInputSignals(input_port_size), StaticSystemMIMO(input_port_size, input_signals.get_data_ptr(), 1, &output), operators(input_port_size) {
		operators.load_data(operators_arr);
	}
private:
	real_t output;
	Vector<Operators::binary_operator<real_t>> operators;
	void eval_function(const Vector<signal_realtime_T>&, Vector<signal_realtime_T>&);

};

}

#endif /* STATIC_SYSTEMS_H_ */
