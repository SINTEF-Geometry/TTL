#ifndef _HANDLEID_H_
#define _HANDLEID_H_

#include <stddef.h>

/// Base class with reference counting for smart pointers
class HandleId {
protected:
  int     refcount;         // reference count
  char    dynamic_object;   // '1': explicit call to new created the object

public:
  HandleId ();
  virtual ~HandleId (){};

  bool isReferenced () const {return refcount != 0;}
  int  getNoRefs    () const {return refcount;}
  bool dynamicObj   () const {return dynamic_object=='1';}

  void  increment () {refcount++;}
  void  decrement () {refcount--;}
  void* operator new (size_t t);
  void* operator new (size_t t,int, const char * file, int line);
  void  operator delete (void* v);
}; 
#endif
