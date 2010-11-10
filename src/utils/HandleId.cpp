//==================================================================================================
//
// File: HandleId.cpp
//
// Created:
//
// Author: Øyvind Hjelle <oyvind.hjelle@math.sintef.no>
//
// Revision: $Id: HandleId.cpp,v 1.2 2006/07/26 12:08:44 oyvindhj Exp $
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


#include <ttl/utils/HandleId.h>


#include <iostream>
#include <cstdlib>


using namespace std;


static char local_HandleId_new = '0'; // for hack: op.new and constructor


//--------------------------------------------------------------------------------------------------
HandleId::HandleId () {
  refcount = 0; 
  dynamic_object = local_HandleId_new;
  local_HandleId_new = '0';
}


//--------------------------------------------------------------------------------------------------
void* HandleId::operator new (size_t t) {
	return HandleId::operator new(t,0,0,0);
}


//--------------------------------------------------------------------------------------------------
void* HandleId::operator new (size_t t,int n, const char * file, int line) {
   HandleId* tmp = (HandleId*) calloc(1,t);

  if (local_HandleId_new == '1')
    cout << "HandleId::operator new" <<
	      "nested calls of new are detected, the code may be typically\n" <<
	      "like this: some_obj.rebind (new B(new A())); where both class\n" <<
	      "A and class B are derived from HandleId. This construction is\n" <<
	      "not recommended. Modify your code (the size of the allocated\n" <<
	      "object in this operator new is %d, this may help you to\n"
	      "locate the critical statements)." << t << endl;

  local_HandleId_new = '1';

  return tmp;
}


//--------------------------------------------------------------------------------------------------
void HandleId::operator delete (void* v) {
  free(v);
}
