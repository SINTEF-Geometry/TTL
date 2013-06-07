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

#ifndef _HANDLEID_H_
#define _HANDLEID_H_


#include <stddef.h>


//--------------------------------------------------------------------------------------------------
// HandleId class
//--------------------------------------------------------------------------------------------------

/** \class HandleId 
*   \brief Base class with reference counting for smart pointers
*/

class HandleId {

protected:
  int     refcount;       // reference count
  char    dynamic_object; // '1': explicit call to new created the object

public:
  HandleId ();
  virtual ~HandleId () {};

  bool isReferenced () const { return refcount != 0; }
  int  getNoRefs    () const { return refcount; }
  bool dynamicObj   () const { return dynamic_object == '1'; }

  void  increment () { refcount++; }
  void  decrement () { refcount--; }

  void* operator new (size_t t);
  void* operator new (size_t t,int, const char * file, int line);

  void  operator delete (void* v);

};
 
#endif
