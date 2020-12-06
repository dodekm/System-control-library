#ifndef SIGNAL_SOURCES_H_
#define SIGNAL_SOURCES_H_

#include "systems.h"

namespace SystemControl {

class SignalSource: public System {

public:
	SignalSource();

	void eval(real_t time, Vector<Signal>& output_signals) {
		eval_function(time, output_signals);
	}
	void eval(real_t time) {
		eval_function(time, output_port);
	}
	real_t eval_scalar(real_t);

	inline void update(real_t time, step_type step_type) {
		eval(time);
	}

private:
	virtual void eval_function(real_t, Vector<Signal>&)=0;
	real_t output;
};

class SignalSourceSine: public SignalSource {
public:
	SignalSourceSine(real_t period, real_t amplitude, real_t offset) :
			period(period), amplitude(amplitude), offset(offset) {
	}
private:
	real_t period;
	real_t amplitude;
	real_t offset;
	void eval_function(real_t , Vector<Signal>&);
};

class SignalSourceStep: public SignalSource {

public:
	SignalSourceStep(real_t step_time, real_t initial_state, real_t final_state) :
			step_time(step_time), initial_state(initial_state), final_state(final_state) {
	}
private:
	real_t step_time;
	real_t initial_state;
	real_t final_state;
	void eval_function(real_t, Vector<Signal>& );
};


class SignalSourceTimeseries: public SignalSource, public SignalTimeseries {

public:
	using SignalTimeseries::SignalTimeseries;
private:

	void eval_function(real_t time, Vector<Signal>& output_signals)
	{
		output_signals[0]=read(time);
	}
};

class SignalSourceSampledSignal: public SignalSource, public SignalSampled {

public:
	using SignalSampled::SignalSampled;
private:

	void eval_function(real_t time, Vector<Signal>& output_signals)
	{
		output_signals[0]=at(time);
	}
};


}

#endif /* SIGNAL_SOURCES_H_ */
