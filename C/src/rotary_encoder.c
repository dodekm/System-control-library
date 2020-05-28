
#include "rotary_encoder.h"

return_code rotary_encoder_init(rotary_encoder_t* encoder, io_digital_interrupt_input_generic_t* A_input, io_digital_interrupt_input_generic_t* B_input, rotary_encoder_polarity input_polarity, float phi_per_increment, void (*position_change_handler)(int32_t, rotary_encoder_flag)) {

#ifdef ASSERT_NULLPTR
	if (encoder == NULL)
		return return_NULLPTR;
	if (A_input == NULL)
		return return_NULLPTR;
	if (B_input == NULL)
		return return_NULLPTR;
#endif
	encoder->input_polarity = input_polarity;
	encoder->phi_per_increment = phi_per_increment;
	encoder->A_input = A_input;
	encoder->B_input = B_input;
	encoder->A_state = bool_false;
	encoder->B_state = bool_false;
	encoder->signal_phase = 0;
	encoder->position = 0;
	encoder->flag = 0;

	encoder->position_change_handler = position_change_handler;

	return return_OK;
}
void rotary_encoder_interrupt_handler(rotary_encoder_t* encoder) {

	bool_t A_state_new = io_digital_input_read((io_digital_input_generic_t*) encoder->A_input) ^ encoder->input_polarity;
	bool_t B_state_new = io_digital_input_read((io_digital_input_generic_t*) encoder->B_input) ^ encoder->input_polarity;

	if (A_state_new != encoder->A_state) {
		encoder->A_state = A_state_new;
		if (A_state_new) {
			if (encoder->signal_phase == 0)
				encoder->signal_phase = 1;
			else if (encoder->signal_phase == -1)
				encoder->signal_phase = -2;
			else
				encoder->signal_phase = 0;
		} else {
			if (encoder->signal_phase == 2)
				encoder->signal_phase = 3;
			else if (encoder->signal_phase == -3)
				encoder->signal_phase = -4;
			else
				encoder->signal_phase = 0;
		}
	} else if (B_state_new != encoder->B_state) {
		encoder->B_state = B_state_new;
		if (B_state_new) {
			if (encoder->signal_phase == 1)
				encoder->signal_phase = 2;
			else if (encoder->signal_phase == 0)
				encoder->signal_phase = -1;
			else
				encoder->signal_phase = 0;
		} else {
			if (encoder->signal_phase == 3)
				encoder->signal_phase = 4;
			else if (encoder->signal_phase == -2)
				encoder->signal_phase = -3;
			else
				encoder->signal_phase = 0;
		}
	} else
		return;

	if (encoder->signal_phase == 4 || encoder->signal_phase == -4) {
		int flag = encoder->signal_phase / 4;
		encoder->position += flag;
		encoder->flag = flag;
		encoder->signal_phase = 0;
		if (encoder->position_change_handler != NULL) {
			encoder->position_change_handler(encoder->position, encoder->flag);
		}
	}
}

