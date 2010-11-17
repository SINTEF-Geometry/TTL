//==========================================================================
//
// File: Handle.h
//
// Created:
//
// Author: Øyvind Hjelle <oyvind.hjelle@math.sintef.no>
//
// Revision: $Id: Handle.h,v 1.2 2006/07/26 12:08:44 oyvindhj Exp $
//
// Description:
//
//==========================================================================
// Copyright (C) 2000-2003 SINTEF Applied Mathematics.  All rights reserved.
//
// This file may be distributed under the terms of the Q Public License
// as defined by Trolltech AS of Norway and appearing in the file
// LICENSE.QPL included in the packaging of this file.
// 
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
//==========================================================================


#ifndef _HANDLE_H_
#define _HANDLE_H_


#include <assert.h>


//--------------------------------------------------------------------------
// Handle class
//--------------------------------------------------------------------------

/** \class Handle
*   \brief Template class for smart pointers. The actual class must
*   inherit from HandleId
*/

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
    if (classptr != 0) {
      classptr->decrement ();
      assert(classptr->getNoRefs() >= 0);
      if (!classptr->isReferenced()) {
        if (classptr->dynamicObj())
          delete classptr;
      }
    }
  }

  void rebind(const T * pc) {
    if (classptr != pc) {
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
  
  const T* operator->() const { return  classptr; }
  T* operator->()             { return  classptr; }
  const T& operator()() const { return *classptr; }
  T& operator()()             { return *classptr; }
  const T& operator*() const  { return *classptr; }
  T& operator*()              { return *classptr; }
  const T* getPtr() const     { return  classptr; }
  T* getPtr()                 { return  classptr; }
  const T& getRef() const     { return *classptr; }
  T& getRef()                 { return *classptr; }

  void operator=(const Handle<T>& h) { rebind(h.getPtr()); }
  void operator=(const T* p)         { rebind(p); }
  void operator=(const T& p)         { rebind(p); }

  bool operator==(const Handle<T>& h) const { return classptr == h.classptr; }
  bool operator!=(const Handle<T>& h) const { return classptr != h.classptr; }
  bool operator< (const Handle<T>& h) const { return classptr < h.classptr; }
  bool operator> (const Handle<T>& h) const { return classptr > h.classptr; }

};

#endif
