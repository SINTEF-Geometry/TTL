//===========================================================================
//                                                                           
// File: example_util.h                                                      
//                                                                           
// Created: Thu Jun  6 16:06:08 2002                                         
//                                                                           
// Author: Atgeirr F Rasmussen <atgeirr@sintef.no>
//                                                                           
// Revision: $Id: example_util.h,v 1.2 2003-03-21 11:52:44 afr Exp $
//                                                                           
// Description:
//                                                                           
//===========================================================================

#ifndef _EXAMPLE_UTIL_H
#define _EXAMPLE_UTIL_H

#include <vector>
#include <boost/smart_ptr.hpp>

template <class PointType>
void createRandomData(int noPoints, std::vector< boost::shared_ptr<PointType> >& points, int seed=1)
{
    srand(seed);
    double x, y;
    points.resize(noPoints);
    for (int i = 0; i < noPoints; ++i) {
      int random = rand();
      x = ((double)random/(double)RAND_MAX);
      random = rand();
      y = ((double)random/(double)RAND_MAX);
      points[i].reset(new PointType(x,y));
    }
    /*
#ifdef _MSC_VER
    srand(seed);
#else
    srand48((long int)seed);
#endif
    
    double x, y;
    points.resize(noPoints);
    for (int i = 0; i < noPoints; ++i) {
#ifdef _MSC_VER
      int random = rand();
      x = ((double)random/(double)RAND_MAX);
      random = rand();
      y = ((double)random/(double)RAND_MAX);
      points[i].reset(new PointType(x,y));
#else
      double random = drand48();
      x = random;
      random = drand48();
      y = random;
      points[i].reset(new PointType(x,y));
#endif
    }
    */
  }



#endif // _EXAMPLE_UTIL_H

