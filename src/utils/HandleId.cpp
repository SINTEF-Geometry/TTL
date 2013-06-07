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
