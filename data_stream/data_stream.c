#include "data_stream.h"
#include "string.h"

static const data_stream_header_t data_stream_header_template = { [0 ... sizeof(data_stream_header_t) - 1] ='$' };
static const data_stream_tail_t data_stream_tail_template = { [0 ... sizeof(data_stream_tail_t) - 1] ='*' };


static data_stream_return_t data_stream_message_validate(data_stream_message_struct*);
static uint8_t data_stream_message_CRC(data_stream_message_struct*);
static data_stream_return_t data_stream_message_init(data_stream_message_struct*, data_stream_operation_t, data_stream_id_t, data_stream_data_type_t);
static data_stream_return_t data_stream_compare_stream_metadata(data_stream_metadata_t, data_stream_metadata_t);
static uint8_t CRC_xor(uint8_t*, size_t);
static uint8_t CRC_sum(uint8_t*, size_t);

static data_stream_return_t data_stream_file_stream_generic_write(data_stream_file_stream_generic* file_handle, uint8_t* buffer, uint32_t btw, uint32_t* bw, uint32_t timeout) {
	size_t bytes_written = file_handle->file_write_callback(file_handle->handle, (const void*) buffer, (size_t) btw, timeout);
	*bw = (uint32_t) bytes_written;
	if (bytes_written == 0)
		return data_stream_file_error;
	return data_stream_ok;
}
static data_stream_return_t data_stream_file_stream_generic_read(data_stream_file_stream_generic* file_handle, uint8_t* buffer, uint32_t btr, uint32_t*br, uint32_t timeout) {
	size_t bytes_readed = file_handle->file_read_callback(file_handle->handle, (void*) buffer, (size_t) btr, timeout);
	*br = (uint32_t) bytes_readed;
	if (bytes_readed == 0)
		return data_stream_file_error;
	return data_stream_ok;
}

static data_stream_return_t data_stream_message_validate(data_stream_message_struct* message) {
	if (memcmp(message->header, data_stream_header_template, sizeof(data_stream_header_t)) != 0)
		return data_stream_error_invalid_message;
	if (memcmp(message->tail, data_stream_tail_template, sizeof(data_stream_tail_t)) != 0)
		return data_stream_error_invalid_message;
	if (message->metadata.data_type != data_stream_data_type_integer && message->metadata.data_type != data_stream_data_type_floating)
		return data_stream_error_invalid_message;
	if (message->metadata.operation != data_stream_operation_read && message->metadata.operation != data_stream_operation_write && message->metadata.operation != data_stream_operation_response)
		return data_stream_error_invalid_message;
#ifdef data_stream_use_checksum
	if (data_stream_message_CRC(message) != message->checksum)
		return data_stream_error;
#endif
	return data_stream_ok;
}
static data_stream_return_t data_stream_message_init(data_stream_message_struct* message, data_stream_operation_t operation, data_stream_id_t stream_id, data_stream_data_type_t data_type) {

	memset((void*) message, 0, sizeof(data_stream_message_struct));
	memcpy(message->header, data_stream_header_template, sizeof(data_stream_header_t));
	memcpy(message->tail, data_stream_tail_template, sizeof(data_stream_tail_t));
	message->metadata.operation = operation;
	message->metadata.data_type = data_type;
	message->metadata.stream_id = stream_id;
	message->data_union.integer_signed = 0;
	return data_stream_ok;
}

static data_stream_return_t data_stream_compare_stream_metadata(data_stream_metadata_t metadata_1, data_stream_metadata_t metadata_2) {
	if (metadata_1.data_type == metadata_2.data_type && metadata_1.stream_id == metadata_2.stream_id)
		return data_stream_ok;
	else
		return data_stream_wrong_stream_params;
}
/**
 *
 * @param stream data stream instance structure pointer
 * @param data_read_callback application data read callback function to register (required for server), can be NULL
 * @param data_write_callback application data write callback function to register (required for server), can be NULL
 * @param file_read_callback file read callback function to register
 * @param file_write_callback file write callback function to register
 * @param file_handle pointer to file handle, can be NULL
 * @param get_timestamp_fcn timestamp callback function, can be NULL
 * @param timestamp_source timestamp source handle pointer, can be NULL
 * @param user_data user data pointer, can be NULL
 * @return data stream return code
 */
data_stream_return_t data_stream_init(data_stream_instance* stream, data_stream_data_read_callback_t data_read_callback, data_stream_data_write_callback_t data_write_callback, file_stream_read_callback_t file_read_callback, file_stream_write_callback_t file_write_callback, void* file_handle, get_timestamp_callback_t get_timestamp_fcn, void* timestamp_source, void* user_data) {

	if (stream == NULL)
		return data_stream_error;
	stream->application_data_read_callback = data_read_callback;
	stream->application_data_write_callback = data_write_callback;

	if (file_read_callback == NULL)
		return data_stream_error;
	stream->file_stream.file_read_callback = file_read_callback;
	if (file_write_callback == NULL)
		return data_stream_error;
	stream->file_stream.file_write_callback = file_write_callback;
	stream->file_stream.handle = file_handle;

	stream->get_timestamp_fcn = get_timestamp_fcn;
	stream->timestamp_source = timestamp_source;
	stream->user_data = user_data;

	memset(&stream->message_received, 0, sizeof(stream->message_received));
	memset(&stream->message_to_send, 0, sizeof(stream->message_to_send));

	stream->state = data_stream_state_ready;

	return data_stream_ok;
}

/**
 * Writes data stream as client.
 * @param stream data stream instance structure pointer
 * @param stream_id stream id number
 * @param data_type transferred data type
 * @param data_ptr pointer to transferred data, if NULL - function uses data read callback
 * @return data stream return code
 */
data_stream_return_t data_stream_client_write_stream(data_stream_instance* stream, data_stream_id_t stream_id, data_stream_data_type_t data_type,const data_stream_data_union_t* data_ptr) {
	data_stream_return_t returnval = data_stream_ok;
	if (stream->state == data_stream_state_busy)
		return data_stream_error_busy;
	stream->state = data_stream_state_busy;
	data_stream_message_init(&stream->message_to_send, data_stream_operation_write, stream_id, data_type);

	if (data_ptr != NULL) {
		stream->message_to_send.data_union = *data_ptr;
	} else if (stream->application_data_read_callback != NULL) {
		returnval = stream->application_data_read_callback(stream_id, data_type, &stream->message_to_send.data_union, stream->user_data);
		if (returnval != data_stream_ok) {
			goto terminate;
		}
	} else {
		returnval = data_stream_error;
		goto terminate;
	}

#ifdef data_stream_use_timestamp
	if (stream->get_timestamp_fcn != NULL)
		stream->message_to_send.timestamp = stream->get_timestamp_fcn(stream->timestamp_source);
	else
		stream->message_to_send.timestamp = 0;
#endif

#ifdef data_stream_use_checksum
	stream->message_to_send.checksum = data_stream_message_CRC(&stream->message_to_send);
#endif

	uint32_t bytes_written = 0;
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (uint8_t*) (&stream->message_to_send), sizeof(data_stream_message_struct), &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}
	uint32_t bytes_readed = 0;
	returnval = data_stream_file_stream_generic_read(&stream->file_stream, (uint8_t*) (&stream->message_received), sizeof(data_stream_message_struct), &bytes_readed, data_stream_read_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_readed != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}
	returnval = data_stream_message_validate(&stream->message_received);
	if (returnval != data_stream_ok) {
		goto terminate;
	}
	if (stream->message_received.metadata.operation != data_stream_operation_response) {
		returnval = data_stream_error_invalid_message;
		goto terminate;
	}
	data_stream_metadata_t metadata_ref = { 0 };
	metadata_ref.operation = 0;
	metadata_ref.stream_id = stream_id;
	metadata_ref.data_type = data_type;

	returnval = data_stream_compare_stream_metadata(metadata_ref, stream->message_received.metadata);
	if (returnval != data_stream_ok) {
		goto terminate;
	}

	terminate: stream->state = data_stream_state_ready;
	return returnval;

}
/**
 * Reads data stream as client.
 * @param stream data stream instance structure pointer
 * @param stream_id stream id number
 * @param data_type transferred data type
 * @param data_ptr pointer to received data, if NULL - function uses data write callback
 * @return data stream return code
 */
data_stream_return_t data_stream_client_read_stream(data_stream_instance* stream, data_stream_id_t stream_id, data_stream_data_type_t data_type, data_stream_data_union_t* data_ptr) {

	data_stream_return_t returnval = data_stream_ok;
	if (stream->state == data_stream_state_busy)
		return data_stream_error_busy;
	stream->state = data_stream_state_busy;
	data_stream_message_init(&stream->message_to_send, data_stream_operation_read, stream_id, data_type);

#ifdef data_stream_use_timestamp
	if (stream->get_timestamp_fcn != NULL)
		stream->message_to_send.timestamp = stream->get_timestamp_fcn(stream->timestamp_source);
	else
		stream->message_to_send.timestamp = 0;
#endif

#ifdef data_stream_use_checksum
	stream->message_to_send.checksum = data_stream_message_CRC(&stream->message_to_send);
#endif

	uint32_t bytes_written = 0;
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (uint8_t*) (&stream->message_to_send), sizeof(data_stream_message_struct), &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}

	uint32_t bytes_readed = 0;
	returnval = data_stream_file_stream_generic_read(&stream->file_stream, (uint8_t*) (&stream->message_received), sizeof(data_stream_message_struct), &bytes_readed, data_stream_read_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_readed != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}
	returnval = data_stream_message_validate(&stream->message_received);
	if (returnval != data_stream_ok) {
		goto terminate;
	}
	if (stream->message_received.metadata.operation != data_stream_operation_response) {
		returnval = data_stream_error_invalid_message;
		goto terminate;
	}
	data_stream_metadata_t metadata_ref = { 0 };
	metadata_ref.operation = 0;
	metadata_ref.stream_id = stream_id;
	metadata_ref.data_type = data_type;
	returnval = data_stream_compare_stream_metadata(metadata_ref, stream->message_received.metadata);
	if (returnval != data_stream_ok) {
		goto terminate;
	}
	if (data_ptr != NULL) {
		*data_ptr = stream->message_received.data_union;
	} else if (stream->application_data_write_callback != NULL) {
		returnval = stream->application_data_write_callback(stream_id, data_type, &stream->message_received.data_union, stream->user_data);
		if (returnval != data_stream_ok) {
			goto terminate;
		}
	} else {
		returnval = data_stream_error;
		goto terminate;
	}
	terminate: stream->state = data_stream_state_ready;
	return returnval;

}

/**
 * Runs data stream server instance.
 * Serves client requests - sends requested data
 * @param stream data stream instance structure pointer
 * @return data stream return code
 */
data_stream_return_t data_stream_server(data_stream_instance* stream) {

	data_stream_return_t returnval = data_stream_ok;
	if (stream->state == data_stream_state_busy)
		return data_stream_error_busy;
	stream->state = data_stream_state_busy;

	uint32_t bytes_readed = 0;
	returnval = data_stream_file_stream_generic_read(&stream->file_stream, (uint8_t*) (&stream->message_received), sizeof(data_stream_message_struct), &bytes_readed, 0xFFFFFFFF);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_readed != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}
	returnval = data_stream_message_validate(&stream->message_received);
	if (returnval != data_stream_ok)
		goto terminate;

	data_stream_message_init(&stream->message_to_send, data_stream_operation_response, stream->message_received.metadata.stream_id, stream->message_received.metadata.data_type);
	if (stream->message_received.metadata.operation == data_stream_operation_read) {
		returnval = stream->application_data_read_callback(stream->message_received.metadata.stream_id, stream->message_received.metadata.data_type, &stream->message_to_send.data_union, stream->user_data);
		if (returnval != data_stream_ok) {
			goto terminate;
		}
	} else if (stream->message_received.metadata.operation == data_stream_operation_write) {
		returnval = stream->application_data_write_callback(stream->message_received.metadata.stream_id, stream->message_received.metadata.data_type, &stream->message_received.data_union, stream->user_data);
		if (returnval != data_stream_ok) {
			goto terminate;
		}
	}

#ifdef data_stream_use_timestamp
	if (stream->get_timestamp_fcn != NULL)
		stream->message_to_send.timestamp = stream->get_timestamp_fcn(stream->timestamp_source);
	else
		stream->message_to_send.timestamp = 0;
#endif

#ifdef data_stream_use_checksum
	stream->message_to_send.checksum = data_stream_message_CRC(&stream->message_to_send);
#endif
	uint32_t bytes_written = 0;
	returnval = data_stream_file_stream_generic_write(&stream->file_stream, (uint8_t*) (&stream->message_to_send), sizeof(data_stream_message_struct), &bytes_written, data_stream_write_timeout);
	if (returnval != data_stream_ok) {
		goto terminate;
	} else if (bytes_written != sizeof(data_stream_message_struct)) {
		returnval = data_stream_file_error;
		goto terminate;
	}
	terminate: stream->state = data_stream_state_ready;
	return returnval;
}

#ifdef  data_stream_use_checksum
static uint8_t data_stream_message_CRC(data_stream_message_struct* message) {
	void* start_ptr = (void*) (&message->metadata);
	void* end_ptr = (void*) (&message->checksum);
#ifdef data_stream_checksum_XOR
	return CRC_xor((uint8_t*) start_ptr, (size_t) (end_ptr - start_ptr));
#endif

#ifdef data_stream_checksum_SUM
	return CRC_sum((uint8_t*) start_ptr, (size_t) (end_ptr - start_ptr));
#endif

}

static uint8_t CRC_xor(uint8_t* ptr, size_t length) {
	uint8_t CRC = 0;
	for (size_t i = 0; i < length; i++) {
		CRC ^= ptr[i];
	}
	return CRC;
}

static uint8_t CRC_sum(uint8_t* ptr, size_t length) {
	uint8_t CRC = 0;
	for (size_t i = 0; i < length; i++) {
		CRC += ptr[i];
	}
	return CRC;
}

#endif
