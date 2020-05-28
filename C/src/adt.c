#include "adt.h"

/**
 * Asserts linked list.
 * @param list liked list pointer
 * @return returns:
 * - @ref return_OK if list is empty or if has one or more items
 * - @ref return_ERROR if list front and back are not consistent
 * - @ref return_NULLPTR if liked list pointer is NULL
 */
return_code list_assert(const list_t* list) {
	if (list == NULL)
		return return_NULLPTR;
	if ((list->front == NULL) ^ (list->back == NULL))
		return return_ERROR;
	return return_OK;
}

/**
 * Pushes item to front of linked list.
 * Function dynamically allocates item structure.
 * @param list linked list pointer
 * @param object_ptr pointer to pushed object
 * @return system control library error code
 */
return_code list_push_front(list_t* list, void* object_ptr) {

	if (list_assert(list) != return_OK)
		return return_NULLPTR;
	if (object_ptr == NULL)
		return return_NULLPTR;

	list_item_t* new_item = (list_item_t*) malloc(sizeof(list_item_t));
	if (new_item == NULL)
		return return_NULLPTR;

	new_item->object = object_ptr;
	new_item->prev = NULL;
	new_item->next = list->front;
	if (list->front != NULL)
		list->front->prev = new_item;

	if (list->back == NULL)
		list->back = new_item;

	list->front = new_item;
	return return_OK;
}

/**
 * Pushes item to back of linked list.
 * Function dynamically allocates item structure.
 * @param list linked list pointer
 * @param object_ptr pointer to pushed object
 * @return system control library error code
 */
return_code list_push_back(list_t* list, void* object_ptr) {
	if (list_assert(list) != return_OK)
		return return_NULLPTR;
	if (object_ptr == NULL)
		return return_NULLPTR;
	list_item_t* new_item = (list_item_t*) malloc(sizeof(list_item_t));
	if (new_item == NULL)
		return return_NULLPTR;

	new_item->object = object_ptr;
	new_item->next = NULL;
	new_item->prev = list->back;
	if (list->back != NULL)
		list->back->next = new_item;

	if (list->front == NULL)
		list->front = new_item;

	list->back = new_item;
	return return_OK;
}

/**
 * Removes and returns object from front of linked list.
 * Function frees allocated item structure.
 * @param list linked list pointer
 * @return pointer to item object
 */

void* list_pop_front(list_t* list) {

	if (list_assert(list) != return_OK)
		return NULL;
	if (list->front == NULL)
		return NULL;

	list_item_t* front_item = list->front;
	void* ret_ptr = front_item->object;

	if (list->front == list->back) {
		list->front = NULL;
		list->back = NULL;
	} else {
		list->front = list->front->next;
		list->front->prev = NULL;
	}
	free(front_item);
	return ret_ptr;
}

/**
 * Removes and returns object from back of linked list.
 * Function frees allocated item structure.
 * @param list linked list pointer
 * @return pointer to item object
 */

void* list_pop_back(list_t* list) {

	if (list_assert(list) != return_OK)
		return NULL;
	if (list->back == NULL)
		return NULL;
	list_item_t* back_item = list->back;
	void* ret_ptr = back_item->object;

	if (list->front == list->back) {
		list->front = NULL;
		list->back = NULL;

	} else {
		list->back = list->back->prev;
		list->back->next = NULL;
	}
	free(back_item);
	return ret_ptr;
}
/**
 * Clears linked list.
 * Removes and frees all items of linked list.
 * @param list linked list pointer
 * @return system control library error code
 */
return_code list_clear(list_t* list) {
	if (list == NULL)
		return return_NULLPTR;
	while (list_pop_back(list) != NULL) {
	}

	return return_OK;
}

/**
 * Indexed read access to linked list items.
 * Function does not remove the item from list.
 * If index is out of range returns NULL.
 * @param list linked list pointer
 * @param n index of item
 * @return pointer to item object
 */
void* list_get_nth_element(const list_t* list, uint n) {

	if (list_assert(list) != return_OK)
		return NULL;
	list_item_t* item = list->front;
	for (uint i = 0; i < n; i++) {
		if (item == NULL)
			break;
		item = item->next;
	}

	if (item != NULL)
		return item->object;
	else
		return NULL;

}
/**
 * Indexed write access to linked list items.
 * Function does not add the item to list.
 * If index is out of range returns return_INDEX_OUT_OF_RANGE.
 * @param list linked list pointer
 * @param n index of item
 * @param object_ptr pointer to item object
 * @return system control library error code
 */
return_code list_set_nth_element(list_t* list, uint n, void* object_ptr) {

	if (object_ptr == NULL)
		return return_NULLPTR;

	if (list_assert(list) != return_OK)
		return return_NULLPTR;

	list_item_t* item = list->front;

	for (uint i = 0; i < n; i++) {
		if (item == NULL)
			break;
		item = item->next;
	}

	if (item != NULL) {
		item->object = object_ptr;
		return return_OK;
	} else
		return return_INDEX_OUT_OF_RANGE;
}

/**
 * Checks if object is in linked list.
 * @param list linked list pointer
 * @param object_ptr pointer to object to find in list
 * @return true(1) if found
 */

bool_t list_find_item(const list_t* list, void* object_ptr) {

	if (list_assert(list) != return_OK)
		return bool_false;
	if (object_ptr == NULL)
		return bool_false;
	if (list->front == NULL)
		return bool_false;
	list_item_t* item_ptr = list->front;
	while (item_ptr != NULL) {
		if (item_ptr->object == object_ptr)
			return bool_true;

		item_ptr = item_ptr->next;
	}
	return bool_false;
}

/**
 * Merges linked list A and B.
 * @param list_A linked list A pointer
 * @param list_B linked list B pointer
 * @param list_Dst created linked list pointer
 * @return system control library error code
 */

return_code list_merge_list(list_t* list_A, list_t* list_B, list_t* list_Dst) {

	if (list_A == NULL || list_B == NULL || list_Dst == NULL)
		return return_NULLPTR;
	if (list_is_empty(list_A) || list_is_empty(list_B))
		return return_ERROR;

	list_Dst->front = list_A->front;
	list_Dst->back = list_B->back;

	list_A->back->next = list_B->front;
	list_B->front->prev = list_A->back;

	return return_OK;

}

