/**
 * @file rotary_encoder.h
 * @brief
 */


#ifndef INC_ROTARY_ENCODER_H_
#define INC_ROTARY_ENCODER_H_

#include "common_def.h"
#include "io_interface.h"

typedef enum {
	rotary_encoder_NONE = 0, rotary_encoder_DEC = -1, rotary_encoder_INC = 1,
} rotary_encoder_flag;

typedef enum {
	rotary_encoder_polarity_non_inverted = 0, rotary_encoder_polarity_inverted = 1,
} rotary_encoder_polarity;

typedef struct rotary_encoder {
	io_digital_interrupt_input_generic_t* A_input;
	io_digital_interrupt_input_generic_t* B_input;
	rotary_encoder_polarity input_polarity;
	bool_t A_state;
	bool_t B_state;
	int8_t signal_phase;
	int32_t position;
	float phi_per_increment;
	rotary_encoder_flag flag;
	void (*position_change_handler)(int32_t, rotary_encoder_flag);

} rotary_encoder_t;

#ifdef __cplusplus
extern "C" {
#endif


return_code rotary_encoder_init(rotary_encoder_t*, io_digital_interrupt_input_generic_t*, io_digital_interrupt_input_generic_t*, rotary_encoder_polarity, float, void (*)(int32_t, rotary_encoder_flag));
void rotary_encoder_interrupt_handler(rotary_encoder_t*);

static inline int32_t rotary_encoder_get_position_whole(rotary_encoder_t* encoder) {
	return encoder->position;
}

static inline int32_t rotary_encoder_get_position_increments(rotary_encoder_t* encoder) {
	return (4 * encoder->position + (int32_t) encoder->signal_phase);
}

static inline float rotary_encoder_get_position_angle(rotary_encoder_t* encoder) {
	return (float) rotary_encoder_get_position_increments(encoder) * encoder->phi_per_increment;
}

static inline rotary_encoder_flag rotary_encoder_get_flag(rotary_encoder_t* encoder) {
	rotary_encoder_flag flag = encoder->flag;
	encoder->flag = 0;
	return flag;
}


#ifdef __cplusplus
}
#endif

#endif /* INC_ROTARY_ENCODER_H_ */
