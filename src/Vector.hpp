#ifndef _VECTOR_HPP
#define _VECTOR_HPP

//
// Excruciatingly simple vector-of-pointers class.  Easy & useful.
// No support for addition of elements anywhere but at the end of the
// list, nor for removal of elements.  Does not delete (or interpret
// in any way) its contents.
//
class Vector {
public:
    Vector();
    ~Vector();
    int   add(void* p);
    void* get(int i);
    void  set(int i, void* p);
    int   size();
private:
    void grow();

    int _capacity;
    int _size;
    void** _array;
};

inline Vector::Vector()
{
    _capacity = 0;
    _size = 0;
    _array = 0;
}

inline Vector::~Vector()
{
    delete[] _array;
}

inline int Vector::add(void* p)
{
    if(_capacity == _size)
        grow();
    _array[_size] = p;
    return _size++;
}

inline void* Vector::get(int i)
{
    return _array[i];
}

inline void Vector::set(int i, void* p)
{
    _array[i] = p;
}

inline int Vector::size()
{
    return _size;
}

inline void Vector::grow()
{
    _capacity = 2 * _capacity + 1;
    void** array = new void*[_capacity];
    for(int i = 0; i < _size; i++)
        array[i] = _array[i];
    delete[] _array;
    _array = array;
}

#endif // _VECTOR_HPP
