#ifndef VECTOR_H
#define VECTOR_H

#include "common_def.h"

namespace SystemControl {

template<typename T>
class Vector {
public:

	Vector() {
	}
	Vector(size_t length, T* data_ptr = NULL) {
		init(length, data_ptr);
	}
	Vector(const Vector& vectorSrc, bool copy_data = true) :
			Vector(vectorSrc.length) {
		vectorSrc.assert();
		if (copy_data)
			load_data(vectorSrc.data_ptr);
	}
	Vector(const Vector& vectorSrc, size_t length, size_t offset) :
			Vector(length, vectorSrc.data_ptr + offset) {
		vectorSrc.assert();
#ifdef ASSERT_DIMENSIONS
		if (length + offset > vectorSrc.length)
		throw exception_code(exception_WRONG_DIMENSIONS);
#endif
	}

	Vector(Vector& vectorSrc, size_t length, size_t offset) throw (exception_code) :
			Vector(length, vectorSrc.data_ptr + offset) {
		vectorSrc.assert();
#ifdef ASSERT_DIMENSIONS
		if (length + offset > vectorSrc.length)
		throw exception_code(exception_WRONG_DIMENSIONS);
#endif
	}
	~Vector() {
		deinit();
	}
	void assert() const throw (exception_code);

	inline size_t get_length() const {
		return length;
	}
	virtual inline T& at(uint idx) {
		return data_ptr[idx];
	}
	virtual inline T at(uint idx) const {
		return data_ptr[idx];
	}
	virtual inline T& operator[](uint idx) {
		return data_ptr[idx];
	}
	virtual inline T operator[](uint idx) const {
		return data_ptr[idx];
	}
	T& at_safe(uint) throw (exception_code);
	T at_safe(uint) const throw (exception_code);

	inline T* get_data_ptr() {
		return data_ptr;
	}
	inline const T* get_data_ptr() const {
		return data_ptr;
	}
	inline void resize(size_t length) throw (exception_code) {
		if (length != this->length) {
			deinit();
			init(length);
		}

	}

	friend std::ostream& operator<<(std::ostream& out, const Vector& vec) {
		vec.assert();
		out << '[';
		for (uint i = 0; i < vec.length; i++) {
			out << vec[i];
			if (i < vec.length - 1)
				out << ",";
		}
		out << ']' << std::endl;
		return out;
	}
	class iterator {
	public:
		iterator(T* ptr) :
				ptr(ptr) {
		}
		iterator(const iterator& other) :
				ptr(other.ptr) {
		}
		iterator& operator=(const iterator& other) {
			this->ptr = other.ptr;
		}
		iterator& operator=(T* ptr) {
			this->ptr = ptr;
		}
		T& operator*() {
			return *ptr;
		}
		T operator*() const {
			return *ptr;
		}
		bool operator==(const iterator& other) const {
			return this->ptr == other.ptr;
		}
		bool operator!=(const iterator& other) const {
			return this->ptr != other.ptr;
		}
		iterator& operator++() {
			ptr++;
			return *this;
		}
		iterator operator++(int) {
			iterator orig = *this;
			++*this;
			return orig;
		}
		iterator& operator+=(int offset) {
			ptr += offset;
			return *this;
		}
		iterator& operator--() {
			ptr--;
			return *this;
		}
		;
		iterator operator--(int) {
			iterator orig = *this;
			--*this;
			return orig;
		}
		iterator& operator-=(int offset) {
			ptr -= offset;
			return *this;
		}
	private:
		T* ptr = NULL;
	};

	iterator begin() {
		return iterator(data_ptr);
	}

	iterator end() {
		return iterator(data_ptr + length);
	}

	Vector& operator=(const Vector&) throw (exception_code);
	Vector& operator=(T) throw (exception_code);
	bool operator==(const Vector&) const throw (exception_code);
	void load_data(const T*) throw (exception_code);
	void set_all(T) throw (exception_code);
	Vector subvector(size_t, size_t = 0) throw (exception_code);
	const Vector subvector(size_t, size_t = 0) const throw (exception_code);

	template<typename F>
	void for_each(F) throw (exception_code);
	template<typename F>
	void for_each(F) const throw (exception_code);
	template<typename F>
	void for_each(const Vector<T>&, F) throw (exception_code);
	template<typename F>
	void for_each(const Vector<T>&, F) const throw (exception_code);
	template<typename F>
	void for_each(const Vector<T>&, const Vector&, F) throw (exception_code);

protected:
	void init(size_t, T* = NULL) throw (exception_code);
	void deinit();
	size_t length = 0;
	T* data_ptr = NULL;
	allocation_type_enum allocation_info = allocation_type_unallocated;

};

template<typename T, size_t length>
class VectorFix: public Vector<T> {

public:
	VectorFix() :
			Vector<T>(length, data) {
	}
private:
	T data[length] = { 0 };
};

template<typename T>
T& Vector<T>::at_safe(uint idx) throw (exception_code) {
	assert();
	if (idx >= length)
		throw exception_code(exception_INDEX_OUT_OF_RANGE);
	return this->at(idx);
}

template<typename T>
T Vector<T>::at_safe(uint idx) const throw (exception_code) {
	assert();
	if (idx >= length)
		throw exception_code(exception_INDEX_OUT_OF_RANGE);
	return this->at(idx);
}

template<typename T>
void Vector<T>::assert() const throw (exception_code) {
#ifdef ASSERT_NULLPTR
	if (data_ptr == NULL)
	throw exception_code(exception_NULLPTR);
#endif
#ifdef ASSERT_DIMENSIONS
	if (length == 0)
	throw exception_code(exception_WRONG_DIMENSIONS);
#endif
	if (allocation_info == allocation_type_unallocated)
		throw exception_code(exception_ERROR);

}

template<typename T>
void Vector<T>::init(size_t length, T* data_ptr) throw (exception_code) {
#ifdef ASSERT_DIMENSIONS
	if (length == 0)
	throw exception_code(exception_WRONG_DIMENSIONS);
#endif
	if (allocation_info != allocation_type_unallocated)
		throw exception_code(exception_ERROR);
	if (data_ptr == NULL) {
		data_ptr = (T*) malloc(length * sizeof(T));
		if (data_ptr == NULL)
			throw exception_code(exception_NULLPTR);
		memset(data_ptr, 0, length * sizeof(T));
		allocation_info = allocation_type_dynamic;
	} else {
		allocation_info = allocation_type_static;
	}
	this->data_ptr = data_ptr;
	this->length = length;

}

template<typename T>
void Vector<T>::deinit() {
	try {
		assert();
	} catch (...) {
		return;
	}
	if (allocation_info == allocation_type_dynamic) {
		free((void*) data_ptr);
	}
	data_ptr = (T*) NULL;
	length = 0;
	allocation_info = allocation_type_unallocated;

}

template<typename T>
Vector<T> Vector<T>::subvector(size_t length, size_t offset) throw (exception_code) {
	return Vector(*this, length, offset);
}

template<typename T>
const Vector<T> Vector<T>::subvector(size_t length, size_t offset) const throw (exception_code) {
	assert();
#ifdef ASSERT_DIMENSIONS
	if (length + offset > this->length)
	throw exception_code(exception_WRONG_DIMENSIONS);
#endif
	return Vector<T>(length, data_ptr + offset);

}

template<typename T>
void Vector<T>::set_all(T value) throw (exception_code) {
	auto lambda = [value](auto& A_i,auto i)->auto {A_i=value;return true;};
	for_each(lambda);
}
template<typename T>
void Vector<T>::load_data(const T* data_ptr) throw (exception_code) {
	assert();
#ifdef ASSERT_NULLPTR
	if (data_ptr == NULL)
	throw exception_code(exception_NULLPTR);
#endif
	memcpy(this->data_ptr, data_ptr, length * sizeof(T));

}

template<typename T>
Vector<T>& Vector<T>::operator=(const Vector& vectorSrc) throw (exception_code) {
	vectorSrc.assert();
	if (this->length != vectorSrc.length) {
		deinit();
		init(vectorSrc.length);
	}
	load_data(vectorSrc.data_ptr);
	return *this;
}
template<typename T>
Vector<T>& Vector<T>::operator=(T value) throw (exception_code) {
	set_all(value);
	return *this;
}

template<typename T>
bool Vector<T>::operator==(const Vector& vecB) const throw (exception_code) {

	bool equal = true;
	auto lambda = [&equal](auto A_i,auto B_i,auto i)->auto {equal=(A_i==B_i);return equal;};
	for_each(vecB, lambda);
	return equal;

}
template<typename T>
template<typename F>
void Vector<T>::for_each(F lambda) throw (exception_code) {
	assert();
	for (uint i = 0; i < length; i++) {
		if (!lambda(at(i), i))
			break;
	}
}

template<typename T>
template<typename F>
void Vector<T>::for_each(F lambda) const throw (exception_code) {

	Vector<T>& A = const_cast<Vector<T>&>(*this);
	A.for_each(lambda);
}

template<typename T>
template<typename F>
void Vector<T>::for_each(const Vector<T>& B, F lambda) throw (exception_code) {

	Vector<T>& A = *this;
	A.assert();
	B.assert();

#ifdef ASSERT_DIMENSIONS
	if (A.length != B.length)
	throw exception_code(exception_WRONG_DIMENSIONS);
#endif
	for (uint i = 0; i < length; i++) {
		if (!lambda(A.at(i), B.at(i), i))
			break;
	}

}

template<typename T>
template<typename F>
void Vector<T>::for_each(const Vector<T>& B, F lambda) const throw (exception_code) {
	Vector<T>& A = const_cast<Vector<T>&>(*this);
	A.for_each(B, lambda);
}

template<typename T>
template<typename F>
void Vector<T>::for_each(const Vector<T>& A, const Vector<T>& B, F lambda) throw (exception_code) {

	assert();
	A.assert();
	B.assert();

#ifdef ASSERT_DIMENSIONS
	if (length != A.length || length != B.length)
	throw exception_code(exception_WRONG_DIMENSIONS);
#endif
	for (uint i = 0; i < length; i++) {
		if (!lambda(A.at(i), B.at(i), this->at(i), i))
			break;
	}

}

}

#endif /* VECTOR_H_ */
