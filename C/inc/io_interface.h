/**
 * @file io_interface.h
 * @brief
 *
 *
 */


#ifndef INC_IO_INTERFACE_H_
#define INC_IO_INTERFACE_H_

#include "common_def.h"

#define io_analog_input_max_channels 16

typedef struct io_digital_input_generic {
	bool_t (*read_fcn)(struct io_digital_input_generic*);
} io_digital_input_generic_t;

typedef struct io_digital_interrupt_input_generic {
	io_digital_input_generic_t;
	bool_t state;
	void (*state_changed_handler)(bool_t);
} io_digital_interrupt_input_generic_t;

typedef struct io_digital_output_generic {
	void (*write_fcn)(struct io_digital_output_generic*, bool_t);
} io_digital_output_generic_t;

typedef struct io_analog_input_generic {
	uint16_t channels_data[io_analog_input_max_channels];
	uint n_channels;
	uint8_t n_bits;
	real_t ref_voltage;
	return_code (*sample_fcn)(struct io_analog_input_generic*);
} io_analog_input_generic_t;

typedef struct io_analog_output_generic {
	return_code (*write_fcn)(struct io_analog_output_generic*, uint32_t);
	uint8_t n_bits;
	real_t ref_voltage;
} io_analog_output_generic_t;

typedef struct io_PWM_output_generic {
	return_code (*write_fcn)(struct io_PWM_output_generic*, uint32_t);
	uint32_t full_scale;
} io_PWM_output_generic_t;

#define TYPE_RANGE(n_bits) ((1<<(n_bits))-1)

#ifdef __cplusplus
extern "C" {
#endif


static inline bool_t io_digital_input_read(io_digital_input_generic_t* DI) {
	return DI->read_fcn(DI);
}

static inline void io_digital_input_interrupt_handler(io_digital_interrupt_input_generic_t* DI) {
	bool_t state_new = DI->read_fcn((io_digital_input_generic_t*) DI);
	if (state_new != DI->state) {
		DI->state = state_new;
		if (DI->state_changed_handler != NULL)
			DI->state_changed_handler(DI->state);
	}
}

static inline void io_digital_output_write(io_digital_output_generic_t* DO, bool_t value) {
#ifdef USE_CLIP
	value = CLIP_TOP(value, 1);
#endif
	return DO->write_fcn(DO, value);
}

static inline return_code io_analog_input_sample(io_analog_input_generic_t* AI) {
	return AI->sample_fcn(AI);
}

static inline uint32_t io_analog_input_read_integer(io_analog_input_generic_t* AI, uint channel) {
	if (channel < io_analog_input_max_channels && channel < AI->n_channels)
		return (uint32_t) AI->channels_data[channel];
	else
		return 0;
}
static inline real_t io_analog_input_read_real(io_analog_input_generic_t* AI, uint channel) {
	return ((real_t) io_analog_input_read_integer(AI, channel)) / TYPE_RANGE(AI->n_bits);
}
static inline real_t io_analog_input_read_voltage(io_analog_input_generic_t* AI, uint channel) {
	return io_analog_input_read_real(AI, channel) * AI->ref_voltage;
}

static inline return_code io_analog_output_write_integer(io_analog_output_generic_t* AO, uint32_t value) {
#ifdef USE_CLIP
	value = CLIP_TOP(value, TYPE_RANGE(AO->n_bits));
#endif
	return AO->write_fcn(AO, value);
}

static inline return_code io_analog_output_write_real(io_analog_output_generic_t* AO, real_t value) {
#ifdef USE_CLIP
	value = CLIP_BOTTOM(value, 0.0);
	value = CLIP_TOP(value, 1.0);
#endif
	return AO->write_fcn(AO, (uint32_t) (value * TYPE_RANGE(AO->n_bits)));
}

static inline return_code io_analog_output_write_voltage(io_analog_output_generic_t* AO, real_t voltage) {
	return io_analog_output_write_real(AO, voltage / AO->ref_voltage);
}

static inline return_code io_PWM_output_write_integer(io_PWM_output_generic_t* PWMO, uint32_t value) {
#ifdef USE_CLIP
	value = CLIP_TOP(value, PWMO->full_scale);
#endif
	return PWMO->write_fcn(PWMO, value);
}
static inline return_code io_PWM_output_write_real(io_PWM_output_generic_t* PWMO, real_t duty) {
#ifdef USE_CLIP
	duty = CLIP_BOTTOM(duty, 0.0);
	duty = CLIP_TOP(duty, 1.0);
#endif
	return PWMO->write_fcn(PWMO, (uint32_t) (duty * PWMO->full_scale));
}


#ifdef __cplusplus
}
#endif

#endif /* INC_IO_INTERFACE_H_ */
