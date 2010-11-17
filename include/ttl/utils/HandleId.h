//==================================================================================================
//
// File: HandleId.h
//
// Created:
//
// Author: Øyvind Hjelle <oyvind.hjelle@math.sintef.no>
//
// Revision: $Id: HandleId.h,v 1.2 2006/07/26 12:08:44 oyvindhj Exp $
//
// Description:
//
//==================================================================================================
// Copyright (C) 2000-2003 SINTEF Applied Mathematics.  All rights reserved.
//
// This file may be distributed under the terms of the Q Public License
// as defined by Trolltech AS of Norway and appearing in the file
// LICENSE.QPL included in the packaging of this file.
// 
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
//==================================================================================================


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
