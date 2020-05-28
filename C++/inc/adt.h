#ifndef ADT_H_
#define ADT_H_

#include "common_def.h"

namespace SystemControl {

template<typename T>
class List {
public:

	typedef struct Item {
		struct Item* prev;
		struct Item* next;
		T object;
	} Item_T;

	inline bool is_empty() const {
		assert();
		return (front == NULL && back == NULL);
	}
	inline bool is_one_item() const {
		assert();
		return (!is_empty() && front == back);
	}

	inline T& peek_front() throw (exception_code) {
		if (is_empty())
			throw exception_ERROR;
		return front->object;
	}

	inline T& peek_back() throw (exception_code) {
		if (is_empty())
			throw exception_ERROR;
		return back->object;
	}

	inline void assert() const throw (exception_code) {
		if ((front == NULL) ^ (back == NULL))
			throw exception_ERROR;
	}
	List() {
	}

	~List() {
		clear();
	}

	List(List&, List&) throw (exception_code);
	void push_front(T) throw (exception_code);
	void push_back(T) throw (exception_code);
	T pop_front(void) throw (exception_code);
	T pop_back(void) throw (exception_code);
	void clear();
	T* at(uint) throw (exception_code);
	T at(uint) const throw (exception_code);

	bool find_item(const T&) const throw (exception_code);
	size_t get_length() const throw (exception_code);
	template<typename F>
	bool for_each(F) throw (exception_code);
	template<typename F>
	bool for_each(F) const throw (exception_code);

private:
	Item_T* front = NULL;
	Item_T* back = NULL;

};

template<typename T>
void List<T>::push_front(T object) throw (exception_code) {

	assert();
	Item_T* new_item = (Item_T*) malloc(sizeof(Item_T));
	if (new_item == NULL)
		throw exception_NULLPTR;
	new_item->object = object;
	new_item->prev = NULL;
	new_item->next = front;
	if (!is_empty())
		front->prev = new_item;
	else
		back = new_item;
	front = new_item;

}

template<typename T>
void List<T>::push_back(T object) throw (exception_code) {

	assert();
	Item_T* new_item = (Item_T*) malloc(sizeof(Item_T));
	if (new_item == NULL)
		throw exception_NULLPTR;

	new_item->object = object;
	new_item->next = NULL;
	new_item->prev = back;
	if (!is_empty())
		back->next = new_item;
	else
		front = new_item;

	back = new_item;

}

template<typename T>
T List<T>::pop_front(void) throw (exception_code) {

	assert();
	if (is_empty())
		throw exception_ERROR;

	Item_T* front_item = front;
	T object = front_item->object;

	if (is_one_item()) {
		front = NULL;
		back = NULL;
	} else {
		front = front->next;
		front->prev = NULL;
	}
	free(front_item);
	return object;

}

template<typename T>
T List<T>::pop_back(void) throw (exception_code) {

	assert();
	if (is_empty())
		throw exception_ERROR;

	Item_T* back_item = back;
	T object = back_item->object;

	if (is_one_item()) {
		front = NULL;
		back = NULL;
	} else {
		back = back->prev;
		back->next = NULL;
	}
	free(back_item);
	return object;
}

template<typename T>
template<typename F>
bool List<T>::for_each(F lambda) throw (exception_code) {
	assert();
	Item_T* item_ptr = front;
	uint i = 0;
	while (item_ptr != NULL) {
		if (!lambda(item_ptr->object, i))
			return false;
		item_ptr = item_ptr->next;
		i++;
	}
	return true;
}

template<typename T>
template<typename F>
bool List<T>::for_each(F lambda) const throw (exception_code) {
	assert();
	Item_T* item_ptr = front;
	uint i = 0;
	while (item_ptr != NULL) {
		if (!lambda(item_ptr->object, i))
			return false;
		item_ptr = item_ptr->next;
		i++;
	}
	return true;
}

template<typename T>
T* List<T>::at(uint idx) throw (exception_code) {
	T* item_at=NULL;
	auto lambda = [&item_at,idx](T& item,uint i)->auto {
		if(i==idx)
		{
			item_at=&item;
			return false;
		}
		return true;
	};
	if (!for_each(lambda)) {
		return item_at;
	} else
		throw exception_INDEX_OUT_OF_RANGE;
}

template<typename T>
T List<T>::at(uint idx) const throw (exception_code) {
	T item_at;
	auto lambda = [&item_at,idx](T item,uint i)->auto {
		if(i==idx)
		{
			item_at=item;
			return false;
		}
		return true;
	};
	if (!for_each(lambda)) {
		return item_at;
	} else
		throw exception_INDEX_OUT_OF_RANGE;

}

template<typename T>
void List<T>::clear() {
	while (1) {
		try {
			pop_back();
		} catch (...) {
			return;
		}
	}

}

template<typename T>
bool List<T>::find_item(const T& object) const throw (exception_code) {
	auto lambda = [&object](const T& item,uint i)->auto {return (item!=object);};
	return !for_each(lambda);
}
template<typename T>
size_t List<T>::get_length() const throw (exception_code) {
	size_t length = 0;
	auto lambda = [&length](const T& item,uint i)->auto {length++;return true;};
	for_each(lambda);
	return length;
}

template<typename T>

List<T>::List(List<T>& listA, List<T>& listB) throw (exception_code) {

	listA.assert();
	listB.assert();
	if (listA.is_empty() || listB.is_empty())
		throw exception_ERROR;

	front = listA.front;
	back = listB.back;

	listA.back->next = listB.front;
	listB.front->prev = listA.back;

}

template<typename T>
class Stack: public List<T> {

public:
	inline void push(T object) {
		List<T>::push_front(object);
	}
	inline T pop(void) {
		return List<T>::pop_front();
	}
};

template<typename T>
class Queue: public List<T> {
public:
	inline void enqueue(T object) {
		List<T>::push_front(object);
	}
	inline T dequeue(void) {
		return List<T>::pop_back();
	}

};

}

#endif
