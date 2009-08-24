//===========================================================================
//                                                                           
// File: ExampleNode.h                                                       
//                                                                           
// Created: Thu Jun  6 13:27:34 2002                                         
//                                                                           
// Author: Atgeirr F Rasmussen <atgeirr@sintef.no>
//                                                                           
// Revision: $Id: ExampleNode.h,v 1.6 2004-01-21 12:18:11 bsp Exp $
//                                                                           
// Description: Based on old Node.h
//                                                                           
//===========================================================================

#ifndef _EXAMPLENODE_H
#define _EXAMPLENODE_H


#include "GoTools/ttl/ttl_util.h"
#include <boost/smart_ptr.hpp>


/// \b Node class for data structures (Inherits from HandleId)
class ExampleNode
{
    //static int id_count;
    double x_, y_, z_;
public:
    //int id_;
    /// Constructor
    ExampleNode() {init(0,0,0);}
    /// Constructor
    ExampleNode(double x, double y, double z = 0.0) {init(x,y,z);}
    /// Destructor 
    ~ExampleNode(){}
    void init(double x, double y, double z) {/*id_ = id_count++;*/ x_ = x; y_ = y; z_ = z;}
  
    const double& x() const {return x_;}
    const double& y() const {return y_;}
    const double& z() const {return z_;}
    double& x() {return x_;}
    double& y() {return y_;}
    double& z() {return z_;}

    // Hack to make printEdges() work.
    const double& u() const {return x_;}
    const double& v() const {return y_;}
    double& u() {return x_;}
    double& v() {return y_;}
};

/** ExampleNodeTraits -  Short description.
 * Detailed description.
 */
struct ExampleNodeTraits
{
    typedef ExampleNode NodeType;

    static double scalarProduct2d(const ExampleNode& from1, const ExampleNode& to1,
				  const ExampleNode& from2, const ExampleNode& to2)
    {
	return (to1.x()-from1.x())*(to2.x()-from2.x())
	    + (to1.y()-from1.y())*(to2.y()-from2.y());
    }

    static double crossProduct2d(const ExampleNode& from1, const ExampleNode& to1,
				 const ExampleNode& from2, const ExampleNode& to2)
    {
	return ttl_util::crossProduct2d((to1.x()-from1.x()), (to1.y()-from1.y()),
					(to2.x()-from2.x()), (to2.y()-from2.y()));
    }

    static double orient2d(const ExampleNode& n1, const ExampleNode& n2, const ExampleNode& n3)
    {
	double p1[2];
	double p2[2];
	double p3[2];
	p1[0] = n1.x();
	p1[1] = n1.y();
	p2[0] = n2.x();
	p2[1] = n2.y();
	p3[0] = n3.x();
	p3[1] = n3.y();
	return ttl_util::orient2dfast(p1, p2, p3);
    }

    /// Corners are given in the following order: LL LR UR UL
    static void boundingCorners2d(std::vector< boost::shared_ptr<ExampleNode> >::iterator first,
				  std::vector< boost::shared_ptr<ExampleNode> >::iterator last,
				  ExampleNode& c1, ExampleNode& c2, ExampleNode& c3, ExampleNode& c4)
    {
	double xmin, xmax, ymin, ymax;
	std::vector< boost::shared_ptr<ExampleNode> >::iterator it = first;
	xmin = xmax = (*it)->x();
	ymin = ymax = (*it)->y();
	++it;
	for (; it != last; ++it) {
	    double x = (*it)->x();
	    double y = (*it)->y();
	    if (x > xmax) xmax = x;
	    if (x < xmin) xmin = x;
	    if (y > ymax) ymax = y;
	    if (y < ymin) ymin = y;
	}
	// Adding a padding factor of 10%
	double dx = 0.1 * (xmax - xmin);
	double dy = 0.1 * (ymax - ymin);
	c1.init(xmin-dx, ymin-dy, 0);
	c2.init(xmax+dx, ymin-dy, 0);
	c3.init(xmax+dx, ymax+dy, 0);
	c4.init(xmin-dx, ymax+dy, 0);
    }

    static void readNode(ExampleNode& n, std::istream& is)
    {
	is >> n.x() >> n.y() >> n.z();
    }
    static void writeNode(ExampleNode& n, std::ostream& os)
    {
	os << n.x() << ' ' << n.y() << ' ' << n.z() << '\n';
    }
};



#endif // _EXAMPLENODE_H


