//===========================================================================
//                                                                           
// File:
//                                                                           
// Created: March 1 2001
//                                                                           
// Author: Øyvind Hjelle <oyvind.hjelle@sintef.no>,
//         ............. <............ @sintef.no>
//                                                                           
// Revision: $Id: HeTraits.h,v 1.11 2004-01-21 12:23:08 bsp Exp $
//                                                                           
// Description:
//                                                                           
//===========================================================================
// Copyright (c) 2000 SINTEF Applied Mathematics
//===========================================================================
#ifndef _HALF_EDGE_TRAITS_
#define _HALF_EDGE_TRAITS_

#include "HeEdge.h"

namespace hed {
  //----------------------------------------------
  // Traits class for the half-edge data structure
  //----------------------------------------------
  
    /** \struct HeTraits
     * \brief \b Traits class (static struct) for the half-edge data structure.
     *
     *  The member functions are those required by different function templates
     *  in the TTL. Documentation is given here to explain what actions
     *  should be carried out on the actual data structure as required by the functions
     *  in the ttl namespace.
     *
     *  The source code of \c HeTraits.h shows how the traits class is implemented for the
     *  half-edge data structure.
     * 
     * \see \ref api
     *
     */
    template <class NodeTraits>
    struct HeTraits {
	typedef typename NodeTraits::NodeType NodeType;
	typedef Dart<NodeTraits> Dart;
	typedef HalfEdge<NodeType> Edge;

	/** The floating point type used in calculations
	 *   involving scalar products and cross products.
	 */
	typedef double real_type;
    
	/** @name Geometric Predicates */
	//@{
	/** Scalar product between two 2D vectors represented as darts. <br>
	 *   ttl_util::scalarProduct2d can be used.
	 */
	static real_type scalarProduct2d(const Dart& v1, const Dart& v2) {
	    Dart v10 = v1; v10.alpha0();
	    Dart v20 = v2; v20.alpha0();
	    return NodeTraits::scalarProduct2d(v1.getNode(), v10.getNode(),
					       v2.getNode(), v20.getNode());
	}
	/** Scalar product between two 2D vectors.<br>
	 *   The first vector is represented by a dart \e v, and the second
	 *   vector has direction from the source node of \e v to the point \e p. <br>
	 *   NodeTraits::scalarProduct2d can be used.
	 */
	static real_type scalarProduct2d(const Dart& v, const NodeType& p) {
	    Dart d0 = v; d0.alpha0();
	    return NodeTraits::scalarProduct2d(v.getNode(), d0.getNode(),
					       v.getNode(), p);
	}
	/** Cross product between two vectors in the plane represented as darts.<br>
	 *   The z-component of the cross product is returned. <br>
	 *   NodeTraits::crossProduct2d can be used.
	 */
	static real_type crossProduct2d(const Dart& v1, const Dart& v2) {
	    Dart v10 = v1; v10.alpha0();
	    Dart v20 = v2; v20.alpha0();
	    return NodeTraits::crossProduct2d(v1.getNode(), v10.getNode(),
					      v2.getNode(), v20.getNode());
	}
	/** Cross product between two vectors in the plane.<br>
	 *   The first vector is represented by a dart \e v, and the second
	 *   vector has direction from the source node of \e v to the point \e p. <br>
	 *   The z-component of the cross product is returned. <br>
	 *   NodeTraits::crossProduct2d can be used.
	 */
	static real_type crossProduct2d(const Dart& v, const NodeType& p) {
	    Dart d0 = v; d0.alpha0();
	    return NodeTraits::crossProduct2d(v.getNode(), d0.getNode(),
					      v.getNode(), p);
	}
    
    
	/** Let \e n1 and \e n2 be the nodes associated with two darts, and let \e p
	 *   be a point in the plane. Return a positive value if \e n1, \e n2,
	 *   and \e p occur in counterclockwise order; a negative value if they occur
	 *   in clockwise order; and zero if they are collinear.
	 */
	static real_type orient2d(const Dart& n1, const Dart& n2, const NodeType& p) {
	    return NodeTraits::orient2d(n1.getNode(), n2.getNode(), p);
	}

	/** This is the same predicate as represented with the function above,
	    but with a slighty different interface: <br>
	    The last parameter is given as a dart where the source node of the dart
	    represents a point in the plane.
	    This function is required for constrained triangulation. */
	static real_type orient2d(const Dart& n1, const Dart& n2, const Dart& n3) {
	    return NodeTraits::orient2d(n1.getNode(), n2.getNode(), n3.getNode());
	}

	/** Input is a dart that represents an edge.<br>
	    The output is true if the edge is constrained in the
	    underlying data structure. Currently unused. */
	static bool edgeIsConstrained(const Dart& n1) {
	    return n1.getEdge()->isConstrained();
	}

	//@} // End of predicate group
    
	// A rationale for directing these functions to traits is:
	// e.g., constraints
    
	/* Checks if the edge associated with \e dart should be swapped
	 *   according to the Delaunay criterion.<br>
	 *
	 *   \note
	 *   This function is also present in the TTL as ttl::swapTestDelaunay.<br>
	 *   Thus, the function can be implemented simply as:
	 *   \code
	 *   { return ttl::swapTestDelaunay<TTLtraits>(dart); }
	 *   \endcode
	 */
	//static bool swapTestDelaunay(const Dart& dart) {
	//return ttl::swapTestDelaunay<TTLtraits>(dart);}
    
	/* Checks if the edge associated with \e dart can be swapped, i.e.,
	 *   if the edge is a diagonal in a (strictly) convex quadrilateral.
	 *   This function is also present as ttl::swappableEdge.
	 */
	//static bool swappableEdge(const Dart& dart) {
	//return ttl::swappableEdge<TTLtraits>(dart);}
   
	/* Checks if the edge associated with \e dart should be \e fixed, meaning
	 *   that it should never be swapped. ??? Use when constraints.
	 */
	//static bool fixedEdge(const Dart& dart) {return dart.getEdge()->isConstrained();}

	/** @name Functions for Delaunay Triangulation*/
	//@{
	/** Swaps the edge associated with \e dart in the actual data structure. <br>
	 *
	 *  <center>
	 *  \image html swapEdge.gif
	 *  </center>
	 * 
	 * \retval dart
	 *  Some of the functions require a dart as output.
	 *  If this is required by the actual function, the dart should be delivered
	 *  back in a position as seen if it was glued to the edge when swapping (rotating)
	 *  the edge CCW; see the figure.
	 *
	 *  \note
	 *  If the edge is \e constrained, or if it should not be swapped for
	 *  some other reason, this function need not do the actual swap of the edge.
	 *
	 *  \note
	 *   Some functions in TTL require that \c swapEdge is implemented such that
	 *   darts outside the quadrilateral are not affected by the swap.
	 */
	static void swapEdge(Dart& dart) {
	    if (!dart.getEdge()->isConstrained()) dart.getTriang()->swapEdge(*dart.getEdge());
	}
    
	/** Splits the triangle associated with \e dart in the actual data structure into
	 *   three new triangles joining at \e point. <br>
	 *
	 *  <center>
	 *  \image html splitTriangle.gif
	 *  </center>
	 *
	 *   \retval dart
	 *    A CCW dart incident with the new node
	 */
	static void splitTriangle(Dart& dart, NodeType& point) {
	    HalfEdge<NodeType>* edge = dart.getTriang()->splitTriangle(*dart.getEdge(), point);
	    dart.init(edge, dart.getTriang());
	}
    
	//@} // End of "delaunay" group
    
	/** @name Functions for removing nodes */
	//@{
	/** The reverse operation of TTLtraits::splitTriangle. <br>
	 *   This function is only required for functions that involve
	 *   removal of interior nodes; see for example ttl::removeInteriorNode.
	 *  <center>
	 *  \image html reverse_splitTriangle.gif
	 *  </center>
	 */
	static void reverse_splitTriangle(Dart& dart) {
	    dart.getTriang()->reverse_splitTriangle(*dart.getEdge());}
    
	/** Removes a triangle with an edge at the boundary of the triangulation
	 *   in the actual data structure<br>
	 */
	static void removeBoundaryTriangle(Dart& dart) {
	    dart.getTriang()->removeTriangle(*dart.getEdge());}
	//@} // end group
    };
}; //end of half_edge namespace
#endif
