#ifndef _HANDLE_H_
#define _HANDLE_H_
#include <assert.h>


/// Template for smart pointer. The actual class must inherit from HandleId
template <class T>
class Handle {
protected:
	T* classptr; 
public:
  Handle() { classptr = 0; }
  Handle(const T& ref) {
    classptr = (T*) &ref;
    classptr->increment();
  }
  
  Handle(T* p) {
    classptr = p;
    if (classptr != 0) classptr->increment();
  }
  
  Handle(const Handle<T>& ref) {
    classptr = ref.classptr;
    if (classptr != 0)
      classptr->increment();
  }
  
  ~Handle() {
    if (classptr != 0)
    {
      classptr->decrement ();
      assert(classptr->getNoRefs() >= 0);
      if (!classptr->isReferenced()) {
        if (classptr->dynamicObj())
          delete classptr;
      }
    }
  }
    
  void rebind (const T * pc)
  {
    if (classptr != pc)
    {
      T* p = (T*) pc; // cast const away
      if (p != 0)
        p->increment();
      if (classptr != 0) {
        classptr->decrement ();
        assert(classptr->getNoRefs() >= 0);
        if (!classptr->isReferenced() && classptr->dynamicObj())
          delete classptr;
      }
      classptr = p;
    }
  }
  
  void rebind(const T& p) { rebind(&p); }
  
  const T* operator->() const { return classptr; }
  T* operator->()       { return classptr; }
  const T& operator()() const { return *classptr; }
  T& operator()()	    { return *classptr; }
  const T& operator*()  const { return *classptr; }
  T& operator*()        { return *classptr; }
  const T* getPtr()     const { return classptr; }
  T* getPtr()           { return classptr; }
  const T& getRef()     const { return *classptr; }
  T& getRef()			{ return *classptr; }
  
  
  void operator=(const Handle<T>& h)  { rebind(h.getPtr()); }
  void operator=(const T* p)            { rebind(p); }
  void operator=(const T& p)            { rebind(p); }
  
  bool operator==(const Handle<T>& h) const  { return classptr == h.classptr; }
  bool operator!=(const Handle<T>& h) const  { return classptr != h.classptr; }
  bool operator< (const Handle<T>& h) const  { return classptr < h.classptr; }
  bool operator> (const Handle<T>& h) const  { return classptr > h.classptr; }
};

#endif
