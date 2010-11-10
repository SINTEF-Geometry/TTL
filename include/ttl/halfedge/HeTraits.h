//==================================================================================================
//
// File: HeTraits.h
//
// Created: March 1 2001
//
// Author: Øyvind Hjelle <oyvind.hjelle@math.sintef.no>
//
// Revision: $Id: HeTraits.h,v 1.2 2006/07/26 12:08:44 oyvindhj Exp $
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


#ifndef _HALF_EDGE_TRAITS_
#define _HALF_EDGE_TRAITS_


#include <ttl/halfedge/HeTriang.h>
#include <ttl/halfedge/HeDart.h>


namespace hed {


  //------------------------------------------------------------------------------------------------
  // Traits class for the half-edge data structure
  //------------------------------------------------------------------------------------------------
  
  /** \struct TTLtraits
  *   \brief \b Traits class (static struct) for the half-edge data structure.
  *
  *   The member functions are those required by different function templates
  *   in the TTL. Documentation is given here to explain what actions
  *   should be carried out on the actual data structure as required by the functions
  *   in the \ref ttl namespace.
  *
  *   The source code of \c %HeTraits.h shows how the traits class is implemented for the
  *   half-edge data structure.
  * 
  *   \see \ref api
  *
  */

  struct TTLtraits {
    
    // The actual triangulation object
    static Triangulation* triang_;
    
    /** The floating point type used in calculations
    *   involving scalar products and cross products.
    */
    typedef double real_type;


    //----------------------------------------------------------------------------------------------
    // ------------------------------- Geometric Predicates Group ---------------------------------
    //----------------------------------------------------------------------------------------------

    /** @name Geometric Predicates */
    //@{

    //----------------------------------------------------------------------------------------------
    /** Scalar product between two 2D vectors represented as darts.\n
    *
    *   ttl_util::scalarProduct2d can be used.
    */
    static real_type scalarProduct2d(const Dart& v1, const Dart& v2) {
      Dart v10 = v1; v10.alpha0();
      Dart v20 = v2; v20.alpha0();
      return ttl_util::scalarProduct2d(v10.x()-v1.x(), v10.y()-v1.y(),
                                       v20.x()-v2.x(), v20.y()-v2.y());
    }


    //----------------------------------------------------------------------------------------------
    /** Scalar product between two 2D vectors.
    *   The first vector is represented by a dart \e v, and the second
    *   vector has direction from the source node of \e v to the point \e p.\n
    *
    *   ttl_util::scalarProduct2d can be used.
    */
    static real_type scalarProduct2d(const Dart& v, const Node& p) {
      Dart d0 = v; d0.alpha0();
      return ttl_util::scalarProduct2d(d0.x() - v.x(), d0.y() - v.y(),
                                       p.x() - v.x(), p.y() - v.y());
    }


    //----------------------------------------------------------------------------------------------
    /** Cross product between two vectors in the plane represented as darts.
    *   The z-component of the cross product is returned.\n
    *
    *   ttl_util::crossProduct2d can be used.
    */
    static real_type crossProduct2d(const Dart& v1, const Dart& v2) {
      Dart v10 = v1; v10.alpha0();
      Dart v20 = v2; v20.alpha0();
      return ttl_util::crossProduct2d(v10.x()-v1.x(), v10.y()-v1.y(), 
                                      v20.x()-v2.x(), v20.y()-v2.y());
    }


    //----------------------------------------------------------------------------------------------
    /** Cross product between two vectors in the plane.
    *   The first vector is represented by a dart \e v, and the second
    *   vector has direction from the source node of \e v to the point \e p.
    *   The z-component of the cross product is returned.\n
    *
    *   ttl_util::crossProduct2d can be used.
    */
    static real_type crossProduct2d(const Dart& v, const Node& p) {
      Dart d0 = v; d0.alpha0();
      return ttl_util::crossProduct2d(d0.x() - v.x(), d0.y() - v.y(),
                                      p.x() - v.x(), p.y() - v.y());
    }


    //----------------------------------------------------------------------------------------------
    /** Let \e n1 and \e n2 be the nodes associated with two darts, and let \e p
    *   be a point in the plane. Return a positive value if \e n1, \e n2,
    *   and \e p occur in counterclockwise order; a negative value if they occur
    *   in clockwise order; and zero if they are collinear.
    */
    static real_type orient2d(const Dart& n1, const Dart& n2, const Node& p) {
      real_type pa[2]; real_type pb[2]; real_type pc[2];
      pa[0] = n1.x(); pa[1] = n1.y();
      pb[0] = n2.x(); pb[1] = n2.y();
      pc[0] =  p.x(); pc[1] =  p.y();
      return ttl_util::orient2dfast(pa, pb, pc);
    }


    //----------------------------------------------------------------------------------------------
    /** This is the same predicate as represented with the function above,
    *   but with a slighty different interface:
    *   The last parameter is given as a dart where the source node of the dart
    *   represents a point in the plane.
    *   This function is required for constrained triangulation. 
    */
    static real_type orient2d(const Dart& n1, const Dart& n2, const Dart& p) {
      real_type pa[2]; real_type pb[2]; real_type pc[2];
      pa[0] = n1.x(); pa[1] = n1.y();
      pb[0] = n2.x(); pb[1] = n2.y();
      pc[0] =  p.x(); pc[1] =  p.y();
      return ttl_util::orient2dfast(pa, pb, pc);
    }

    //@} // End of Geometric Predicates Group


    // A rationale for directing these functions to traits is:
    // e.g., constraints

    //----------------------------------------------------------------------------------------------
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
    //  return ttl::swapTestDelaunay<TTLtraits>(dart);
    //}


    //----------------------------------------------------------------------------------------------
    /* Checks if the edge associated with \e dart can be swapped, i.e.,
    *   if the edge is a diagonal in a (strictly) convex quadrilateral.
    *   This function is also present as ttl::swappableEdge.
    */
    //static bool swappableEdge(const Dart& dart) {
    //  return ttl::swappableEdge<TTLtraits>(dart);
    //}


    //----------------------------------------------------------------------------------------------
    /* Checks if the edge associated with \e dart should be \e fixed, meaning
    *   that it should never be swapped. ??? Use when constraints.
    */
    //static bool fixedEdge(const Dart& dart) {
    //  return dart.getEdge()->isConstrained();
    //}


    //----------------------------------------------------------------------------------------------
    // ----------------------- Functions for Delaunay Triangulation Group -------------------------
    //----------------------------------------------------------------------------------------------

    /** @name Functions for Delaunay Triangulation */
    //@{

    //----------------------------------------------------------------------------------------------
    /** Swaps the edge associated with \e dart in the actual data structure.
    *
    *   <center>
    *   \image html swapEdge.gif
    *   </center>
    * 
    *   \param dart
    *   Some of the functions require a dart as output.
    *   If this is required by the actual function, the dart should be delivered
    *   back in a position as seen if it was glued to the edge when swapping (rotating)
    *   the edge CCW; see the figure.
    *
    *   \note
    *   - If the edge is \e constrained, or if it should not be swapped for
    *     some other reason, this function need not do the actual swap of the edge.
    *   - Some functions in TTL require that \c swapEdge is implemented such that
    *     darts outside the quadrilateral are not affected by the swap.
    */
    static void swapEdge(Dart& dart) {
      if (!dart.getEdge()->isConstrained()) triang_->swapEdge(*dart.getEdge());
    }


    //----------------------------------------------------------------------------------------------
    /** Splits the triangle associated with \e dart in the actual data structure into
    *   three new triangles joining at \e point.
    *
    *   <center>
    *   \image html splitTriangle.gif
    *   </center>
    *
    *   \param dart
    *   Output: A CCW dart incident with the new node; see the figure.
    */
    static void splitTriangle(Dart& dart, Node& point) {
      Edge* edge = triang_->splitTriangle(*dart.getEdge(), point);
      dart.init(edge);
    }
    
    //@} // End of Functions for Delaunay Triangulation group


    //----------------------------------------------------------------------------------------------
    // --------------------------- Functions for removing nodes Group -----------------------------
    //----------------------------------------------------------------------------------------------

    /** @name Functions for removing nodes */
    //@{

    //----------------------------------------------------------------------------------------------
    /** The reverse operation of TTLtraits::splitTriangle.
    *   This function is only required for functions that involve
    *   removal of interior nodes; see for example ttl::removeInteriorNode.
    *
    *   <center>
    *   \image html reverse_splitTriangle.gif
    *   </center>
    */
    static void reverse_splitTriangle(Dart& dart) {
      triang_->reverse_splitTriangle(*dart.getEdge());
    }


    //----------------------------------------------------------------------------------------------
    /** Removes a triangle with an edge at the boundary of the triangulation
    *   in the actual data structure
    */
    static void removeBoundaryTriangle(Dart& d) {
      triang_->removeTriangle(*d.getEdge());
    }

    //@} // End of Functions for removing nodes Group

  };

}; // End of hed namespace

#endif
