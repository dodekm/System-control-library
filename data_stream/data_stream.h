/**
 * @file data_stream.h
 */
#ifndef DATA_STREAM_H_
#define DATA_STREAM_H_

#include <stdint.h>
#include <stddef.h>

/** @addtogroup data_stream Communication interface
 *  @brief Data streaming communication interface.
 *
 *  Suitable for serial (UART) and socket  (UDP) data communication
 *  # Implemented functionality
 *  - constant message length
 *  - transfer medium independence
 *  - data read/write callback prototypes
 *  - file abstraction - read/write callback prototypes
 *  - data stream indexing support (256 streams)
 *  - functions for client stream read and write operations
 *  - function for data stream server
 *  - integer and floating point numbers support
 *  - timestamp information support
 *  - message checksum support
 *
 * @{
 */

#define data_stream_use_timestamp ///<configuration switch - adds timestamp information to message
#define data_stream_use_checksum  ///<configuration switch - adds checksum information to message
#define data_stream_checksum_XOR  ///<configuration switch - message checksum using xor (^) bytewise operator
//#define data_stream_checksum_SUM ///<configuration switch -message checksum using add (+) bytewise operator

#define data_stream_header_length 2 ///<length of message header (bytes)

#ifdef data_stream_use_checksum
#define data_stream_tail_length 1 ///<length of message tail (bytes)
#else
#define data_stream_tail_length 2
#endif

#define data_stream_read_timeout 1000 ///<file stream read timeout
#define data_stream_write_timeout 1000 ///<file stream write timeout

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Data stream return code enum.
 * Return codes for data stream functions
 */
typedef enum {
	data_stream_ok = 0,//!< data_stream_ok
	data_stream_error, ///<general error, checksum error
	data_stream_error_invalid_message, ///<message verification failed
	data_stream_file_error, ///<file transfer (read/write) failed
	data_stream_error_busy, ///<data stream is busy - operation pending
	data_stream_wrong_stream_params ///< messages metadata not match, stream id does not exist
} data_stream_return_t;


/**
 * Data stream locking state enum.
 * Provides mutual exclusion access to data stream structure
 */
typedef enum {
	data_stream_state_ready = 0, ///< data stream ready for API calls
	data_stream_state_busy = 1 ///< data stream operation pending (busy), e.g. waiting for file transfer complete
} data_stream_status_t;


typedef uint8_t data_stream_header_t[data_stream_header_length]; ///< message header byte array typedef
typedef uint8_t data_stream_tail_t[data_stream_tail_length]; ///< message tail byte array typedef

typedef uint8_t data_stream_id_t; ///<stream index number typedef (byte) (0-255)
typedef uint8_t data_stream_operation_t; ///<message operation type typedef (byte)
typedef uint8_t data_stream_data_type_t;  ///<message transferred data type typedef (byte)
typedef float data_stream_timestamp_t; ///<message transferred timestamp type (float)

/**
 * Union for transferred data.
 * 4 bytes size.
 * Floating point numbers and integer numbers support.
 */
typedef union {
	float floating; ///<floating point real number
	int32_t integer_signed; ///<integer number
} data_stream_data_union_t;

/**
 * Operation type enum.
 * Message structure contains this information.
 */
enum {
	data_stream_operation_unknown = 0, ///<not assigned operation type (default)
	data_stream_operation_read, ///<data read operation message
	data_stream_operation_write, ///<data write operation message
	data_stream_operation_response ///<response (answer) operation message
};

/**
 * Transferred data type information enum.
 * Message structure contains this information
 */
enum {
	data_stream_data_type_unknown = 0, ///<not assigned data type (default)
	data_stream_data_type_floating, ///<floating point data type
	data_stream_data_type_integer ///<integer data type
};


/**
 * Data stream message metadata structure.
 * Contains stream id, operation and data type information
 * Structure is compressed bitfield
 */
typedef struct {
	data_stream_id_t stream_id :8; ///<message stream index number(0-255)
	data_stream_data_type_t data_type :4; ///<message data type information
	data_stream_operation_t operation :4; ///<message operation type information
} data_stream_metadata_t;

/**
 * Data stream message structure.
 * Timestamp and checksum members are optional (depending on corresponding switches)
 */
typedef struct {
	data_stream_header_t header; ///<Message header data - leading constant byte array
	data_stream_metadata_t metadata; ///<Message metadata information - bitfield structure
#ifdef data_stream_use_timestamp
	data_stream_timestamp_t timestamp; ///<Message timestamp information (optional)
#endif
	data_stream_data_union_t data_union; ///<Message transferred data union
#ifdef data_stream_use_checksum
	uint8_t checksum; ///<Message checksum information (optional)
#endif
	data_stream_tail_t tail; ///<Message tail data - trailing constant byte array
}__attribute__((packed)) data_stream_message_struct;


/**
 * Data write callback function prototype.
 * Application specific data write implementation function
 * @param - stream id
 * @param - data type
 * @param - pointer to data union
 * @param - user data pointer
 * @return data stream return code
 */
typedef data_stream_return_t (*data_stream_data_write_callback_t)(data_stream_id_t, data_stream_data_type_t,const data_stream_data_union_t*,void*);

/**
 * Data read callback function prototype.
 * Application specific data read implementation function
 * @param - stream id
 * @param - data type
 * @param - pointer to data union
 * @param - user data pointer
 * @return - data stream return code
 */
typedef data_stream_return_t (*data_stream_data_read_callback_t)(data_stream_id_t, data_stream_data_type_t, data_stream_data_union_t*,void*);
/**
 * Timestamp callback function prototype.
 * Application specific timestamp read function
 * @param timestamp source handle (optional)
 * @return current system time
 */
typedef data_stream_timestamp_t (*get_timestamp_callback_t)(void*);


/**
 * File stream write callback function prototype.
 * Medium specific file write function
 * @param - file handle pointer
 * @param - data buffer
 * @param - bytes to read
 * @param - timeout
 * @return - bytes readed
 */
typedef size_t (*file_stream_write_callback_t)(void*, const void*, size_t, uint32_t);

/**
 * File stream read callback function prototype.
 * Medium specific file read function
 * @param - file handle pointer
 * @param - data buffer
 * @param - bytes to write
 * @param - timeout
 * @return - bytes written
 */
typedef size_t (*file_stream_read_callback_t)(void*, void*, size_t, uint32_t);


/**
 * File stream abstraction structure.
 * Contains file specific read and write function pointers and file handle pointer
 */
typedef struct {
	void* handle; ///<pointer to file handle
	file_stream_write_callback_t file_write_callback; ///< data write function pointer
	file_stream_read_callback_t file_read_callback; ///< data read function pointer
} data_stream_file_stream_generic;

/**
 * Data stream instance structure.
 */
typedef struct {
	data_stream_message_struct message_to_send;  ///<Outgoing (sending) message structure
	data_stream_message_struct message_received; ///<Incoming (receiving) message structure
	data_stream_status_t state;	///<Instance locking state
	data_stream_data_read_callback_t application_data_read_callback; ///<Registered data read callback function(required for server)
	data_stream_data_write_callback_t application_data_write_callback; ///<Registered data write callback function(required for server)
	data_stream_file_stream_generic file_stream; ///<Transfer medium file stream handle
	void* timestamp_source; ///<Timestamp source handle (optional)
	get_timestamp_callback_t get_timestamp_fcn; ///<Registered timestamp callback function
	void* user_data; ///<Pointer to user data (optional)
} data_stream_instance;

data_stream_return_t data_stream_init(data_stream_instance*, data_stream_data_read_callback_t, data_stream_data_write_callback_t, file_stream_read_callback_t, file_stream_write_callback_t, void*, get_timestamp_callback_t, void*,void*);
data_stream_return_t data_stream_client_write_stream(data_stream_instance*, data_stream_id_t, data_stream_data_type_t,const data_stream_data_union_t*);
data_stream_return_t data_stream_client_read_stream(data_stream_instance*, data_stream_id_t, data_stream_data_type_t, data_stream_data_union_t*);
data_stream_return_t data_stream_server(data_stream_instance*);

#ifdef data_stream_use_timestamp
static inline data_stream_timestamp_t data_stream_get_received_timestamp(data_stream_instance* stream) {
	return stream->message_received.timestamp;
}
#endif

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* DATA_STREAM_H_ */
