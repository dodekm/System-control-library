/**
 * @file adt.h
 */

#ifndef ADT_H_
#define ADT_H_

#include "common_def.h"

/** @addtogroup adt Abstract data types
 * @brief Abstract data types.
 *
 * Contains interface and functions for linked list data type and derived types such as stack and queue
 * # Implemented functionality
 * ## Linked list:
 *  - assert function
 *	- push front/back
 *	- pop front/back
 * 	- indexed element access
 * 	- peek functions
 * 	- clearing function
 * ## Stack
 * 	- push and pop
 * ## Queue
 * 	- enqueue and dequeue
 * @{
 */

/**
 * Linked list node/item structure.
 * Contains pointers to previous and next item
 */
typedef struct list_item {
	struct list_item* prev; ///<pointer to previous list item
	struct list_item* next; ///<pointer to next list item
	void* object;			///<pointer to item object
} list_item_t;

/**
 * Linked list generic structure.
 * Contains pointers to front and back linked list items
 */
typedef struct list {
	list_item_t* front; ///<list front item
	list_item_t* back;  ///<list back item
} list_t;


typedef list_t queue_t;///< queue is specific linked list type
typedef list_t stack_t;///< stack is specific linked list type

#ifdef __cplusplus
extern "C" {
#endif

/**
 * If linked list is empty returns 1.
 * List is empty if both front and back item pointers are NULL
 * @param list linked list to check
 * @return true(1) or false(0)
 */
static inline bool_t list_is_empty(const list_t* list) {
	return (list->front == NULL && list->back == NULL);
}
/**
 * If linked list has only one item returns 1.
 * List is one item if both front and back item pointers are equal and non NULL
 * @param list linked list to check
 * @return true(1) or false(0)
 */
static inline bool_t list_has_one_item(const list_t* list) {
	return (!list_is_empty(list)&& list->front == list->back);
}

/**
 * Returns linked list front item object without removing it.
 * @param list linked list to peek from
 * @return list front item object
 */
static inline void* list_peek_front(const list_t* list) {
	return list->front->object;
}
/**
 * Returns linked list back item object without removing it.
 * @param list linked list to peek from
 * @return list back item object
 */
static inline void* list_peek_back(const list_t* list) {
	return list->back->object;
}

return_code list_assert(const list_t*);
return_code list_push_front(list_t*, void*);
return_code list_push_back(list_t*, void*);
void* list_pop_front(list_t*);
void* list_pop_back(list_t*);

void* list_get_nth_element(const list_t*, uint);
return_code list_set_nth_element(list_t*, uint, void*);
return_code list_clear(list_t*);
bool_t list_find_item(const list_t*, void*);
return_code list_merge_list(list_t*, list_t*, list_t*);

#define stack_push(stack,object_ptr) list_push_front(stack, object_ptr) ///<stack push defined as list push front
#define stack_pop(stack) list_pop_front(stack)	///<stack pop defined as list pop front
#define stack_peek(stack) list_peek_front(stack) ///<stack peek defined as list peek front
#define queue_enqueue(queue,object_ptr) list_push_front(queue, object_ptr) ///<queue enqueue defined as list push front
#define queue_dequeue(queue) list_pop_back(queue) ///<queue dequeue defined as list pop back
#define queue_peek(queue) list_peek_back(queue) ///<queue peek defined as list peek back

#ifdef __cplusplus
}
#endif

/** @} */


#endif /* ADT_H_ */
