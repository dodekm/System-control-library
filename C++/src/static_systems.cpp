#include "static_systems.h"

using namespace SystemControl;

real_t StaticSystem::eval(real_t u) {

	real_t y=0;
	Signal input_signal(&u);
	Signal output_signal(&y);

	const Vector<Signal> input_signals(1, &input_signal);
	Vector<Signal> output_signals(1, &output_signal);
	eval_function(input_signals, output_signals);
	return y;

}

StaticSystem::StaticSystem(size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr) :
		System(system_type_static_system, static_cast<system_time_domain>(system_time_domain_continuous | system_time_domain_discrete), input_port_width, input_port_ptr, output_port_width, output_port_ptr) {
}

StaticSystemSISO::StaticSystemSISO() :
		StaticSystem(1, &input_data, 1, &output_data) {
	input_data = 0;
	output_data = 0;
}

StaticSystemMIMO::StaticSystemMIMO(size_t input_port_width, real_t* input_port_ptr, size_t output_port_width, real_t* output_port_ptr) :
		StaticSystem(input_port_width, input_port_ptr, output_port_width, output_port_ptr) {

	if (input_port_ptr != NULL)
		memset(input_port_ptr, 0, sizeof(real_t) * input_port_width);
	if (output_port_ptr != NULL)
		memset(output_port_ptr, 0, sizeof(real_t) * output_port_width);
}

void StaticSystemSaturation::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {

#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 1 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t u = input_signals.at(0);
	real_t y = u;
	y = CLIP_TOP(y, upper_limit);
	y = CLIP_BOTTOM(y, lower_limit);
	output_signals.at(0) = y;
}

void StaticSystemDeadzone::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 1 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t u = input_signals.at(0);
	real_t y = 0;
	if (u > treshold_high)
		y = u - treshold_high;
	if (u < treshold_low)
		y = u - treshold_low;
	output_signals.at(0) = y;

}

void StaticSystemGain::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 1 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t u = input_signals.at(0);
	real_t y = u * gain;
	output_signals.at(0) = y;

}

void StaticSystemQuantize::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 1 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t u = input_signals.at(0);
	real_t y = Operators::quantize(u, step_size);
	output_signals.at(0) = y;
}

void StaticSystemRelay::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 1 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif
	const real_t u = input_signals.at(0);
	real_t y = y_last;
	if (y_last == value_on) {
		if (u < treshold_switch_off)
			y = value_off;
	} else {
		if (u > treshold_switch_on)
			y = value_on;
	}
	y_last = y;
	output_signals.at(0) = y;

}

void StaticSystemSum::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 2 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t u_1 = input_signals.at(0);
	const real_t u_2 = input_signals.at(1);
	real_t y = u_1 + u_2;
	output_signals.at(0) = y;
}

void StaticSystemSub::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 2 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t u_1 = input_signals.at(0);
	const real_t u_2 = input_signals.at(1);

	real_t y = u_1 - u_2;
	output_signals.at(0) = y;

}

void StaticSystemProduct::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != 2 || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	const real_t u_1 = input_signals.at(0);
	const real_t u_2 = input_signals.at(1);

	real_t y = u_1 * u_2;
	output_signals.at(0) = y;
}

void StaticSystemPortOperator::eval_function(const Vector<Signal>& input_signals, Vector<Signal>& output_signals) {
#ifdef ASSERT_DIMENSIONS
	if (input_signals.get_length() != input_port.get_length() || output_signals.get_length() != 1)
		throw exception_WRONG_DIMENSIONS;
#endif

	real_t y = 0;
	for (uint i = 0; i < operators.get_length(); i++) {
		real_t u = input_signals[i];
		y = operators[i](y, u);
	}
	output_signals.at(0)= y;


}

