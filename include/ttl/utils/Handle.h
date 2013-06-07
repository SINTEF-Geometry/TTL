/*
 * Copyright (C) 1998, 2000-2007, 2010, 2011, 2012, 2013 SINTEF ICT,
 * Applied Mathematics, Norway.
 *
 * Contact information: E-mail: tor.dokken@sintef.no                      
 * SINTEF ICT, Department of Applied Mathematics,                         
 * P.O. Box 124 Blindern,                                                 
 * 0314 Oslo, Norway.                                                     
 *
 * This file is part of TTL.
 *
 * TTL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version. 
 *
 * TTL is distributed in the hope that it will be useful,        
 * but WITHOUT ANY WARRANTY; without even the implied warranty of         
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with TTL. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * In accordance with Section 7(b) of the GNU Affero General Public
 * License, a covered work must retain the producer line in every data
 * file that is created or manipulated using TTL.
 *
 * Other Usage
 * You can be released from the requirements of the license by purchasing
 * a commercial license. Buying such a license is mandatory as soon as you
 * develop commercial activities involving the TTL library without
 * disclosing the source code of your own applications.
 *
 * This file may be used in accordance with the terms contained in a
 * written agreement between you and SINTEF ICT. 
 */

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
