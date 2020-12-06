#include "signal_sources.h"

using namespace SystemControl;


SignalSource::SignalSource():System(system_type_signal_source, static_cast<system_time_domain>(system_time_domain_continuous | system_time_domain_discrete), 0, NULL, 1, &output)
{

}

real_t SignalSource::eval_scalar(real_t time) {
	real_t y = 0;
	Signal output_signal(&y);
	Vector<Signal> output_signals(1, &output_signal);
	eval_function(time, output_signals);
	return y;
}



void SignalSourceSine::eval_function(real_t time, Vector<Signal>& output_signals)
{
	output_signals[0] = amplitude * (real_t)(::sin(2.0 * M_PI * (double)(time / period))) + offset;
}




void SignalSourceStep::eval_function(real_t time, Vector<Signal>& output_signals)
{
	output_signals[0]=time < step_time ? initial_state : final_state;
}

