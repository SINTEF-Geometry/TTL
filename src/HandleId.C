#include <HandleId.h>

#ifndef TTL_USE_OLD_STD
  #include <iostream>
  #include <cstdlib>
  using namespace std;
#else
  #include <iostream.h>
  #include <stdlib.h>
#endif


static char local_HandleId_new = '0'; // for hack: op.new and constructor

HandleId::HandleId () {
  refcount = 0; 
  dynamic_object = local_HandleId_new;
  local_HandleId_new = '0';
}

void* HandleId::operator new (size_t t) {
	return HandleId::operator new(t,0,0,0);
}

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

void HandleId::operator delete (void* v) {
  free(v);
}
