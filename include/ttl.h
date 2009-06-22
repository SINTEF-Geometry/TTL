//===========================================================================
//                                                                           
// File: ttl.h                                                          
//                                                                           
// Created: Sep 08 2000
//                                                                           
// Author: Oeyvind Hjelle <oyvind.hjelle@sintef.no>,
//         ............. <............ @sintef.no>
//                                                                           
// Revision: $Id: ttl.h,v 1.23 2005-07-21 12:44:50 sbr Exp $
//                                                                           
// Description:
//                                                                           
//===========================================================================
// Copyright (c) 2000-2002 SINTEF Applied Mathematics
//===========================================================================

#ifndef _TTL_H_
#define _TTL_H_

// Debugging
#ifdef DEBUG_TTL
#include <iostream>
  static void errorAndExit(char* message) {
  cout << "\n!!! ERROR: "<< message << " !!!\n" << endl; exit(-1);}
#endif

#ifndef TTL_USE_OLD_STD
  #include <list>
#else
  #include <list.h>
#endif

// Next on TOPOLOGY:
// - get triangle strips
// - weighted graph, algorithms using a weight (real) for each edge,
//   e.g. an "abstract length". Use for minimum spanning tree
//   or some arithmetics on weights?
// - Circulators as defined in CGAL with more STL compliant code

// - analyze in detail locateFace: e.g. detect 0-orbit in case of infinite loop 
//   around a node etc.

/** \brief Main interface to TTL
*
*   This namespace contains the basic generic algorithms for the TTL,
*   the SINTEF Triangulation Template Library. <br>
*   Example of functionality are:
* - Incremental Delaunay triangulation
* - Constrained triangulation
* - Insert/remove nodes and constrained edges
* - Traversal operations
* - Misc. queries for extracting information for visualisation systems etc.
*
* \par General requirements and assumptions:
* - \e DartType and \e TraitsType should be implemented in accordance with the description
*   in \ref api.
* - A "\b Requires:" section in the documentation of a function template
*   shows which functionality is required in \e TraitsType to
*   support that specific function. <br>
*   Functionalty required in \e DartType is the same (almost) for all
*   function templates; see \ref api and the example referred to.
* - When a reference to a \e dart object is passed to a function in TTL,
*   it is assumed that it is oriented \e counterclockwise (CCW) in a triangle
*   unless it is explicitly mentioned that it can also be clockwise (CW).
*   The same applies for a dart that is passed from a function in TTL to
*   the users TraitsType class (or struct).
* - When an edge (represented with a dart) is swapped, it is assumed that darts
*   outside the quadrilateral where the edge is a diagonal are not affected by
*   the swap. Thus, \ref hed::TTLtraits::swapEdge "TraitsType::swapEdge"
*   must be implemented in accordance with this rule.
*
* \par Glossary
* - General terms are explained in \ref api.
* - \e CCW - counterclockwise
* - \e CW  - clockwise
* - \e 0_orbit, \e 1_orbit and \e 2_orbit: A sequence of darts around
*   a node, around an edge and in a triangle respectively;
*   see ttl::get_0_orbit_interior and ttl::get_0_orbit_boundary
* - \e arc - In a triangulation an arc is equivalent with an edge
* 
* \see ttl_util and \ref api
* \author Øyvind Hjelle, oyvind.hjelle@sintef.no <br>
*/
namespace ttl {

  //--------------------------------------------------------------------------
  // ------------ F O R W A R D   D E C L A R A T I O N S --------------------
  //--------------------------------------------------------------------------
#if ((_MSC_VER > 0) && (_MSC_VER < 1300))
#else
    template <class TraitsType, class PointType, class DartType>
    bool locateFaceSimplest(const PointType& point, DartType& dart);
    template <class TraitsType, class PointType, class DartType>
    bool locateTriangle(const PointType& point, DartType& dart);
    template <class TraitsType, class PointType, class DartType>
    bool inTriangle(const PointType& point, const DartType& dart);
    template <class TraitsType, class DartType>
    void removeBoundaryNode(DartType& dart);
    template <class TraitsType, class DartType>
    void removeInteriorNode(DartType& dart);
    template <class TraitsType, class DartType, class DartListType>
    void optimizeDelaunay(DartListType& elist);
    template <class TraitsType, class DartType, class DartListType>
    void optimizeDelaunay(DartListType& elist, const typename DartListType::iterator end);
    template <class TraitsType, class DartType>
    bool swapTestDelaunay(const DartType& dart, bool cycling_check = false);
    template <class TraitsType, class DartType>
    void recSwapDelaunay(DartType& diagonal);
    template <class TraitsType, class DartType, class ListType>
    void swapEdgesAwayFromInteriorNode(DartType& dart, ListType& swapped_edges);
    template <class TraitsType, class DartType, class ListType>
    void swapEdgesAwayFromBoundaryNode(DartType& dart, ListType& swapped_edges);
    template <class TraitsType, class DartType, class DartListType>
    void swapEdgeInList(const typename DartListType::iterator& it, DartListType& elist);
    template <class DartType>
      bool isBoundaryEdge(const DartType& dart);
  template <class DartType>
     void positionAtNextBoundaryEdge(DartType& dart) ;
  template <class DartType>
     bool isBoundaryNode(const DartType& dart);
#endif

  
  //--------------------------------------------------------------------------
  // --------------- D E L A U N A Y   G R O U P -----------------------------
  //--------------------------------------------------------------------------
  
  /** @name Delaunay Triangulation */
  //@{
  //--------------------------------------------------------------------------------------
  /** Inserts a new node in an existing Delaunay triangulation and
  *   swaps edges to obtain a new Delaunay triangulation.
  *
  *   This is the basic function for incremental Delaunay triangulation.
  *   When starting from a set of points, an initial Delaunay triangulation
  *   can be made as two triangles forming a rectangle that contains
  *   all the points.
  *   After \c insertNode has been called repeatedly with all the points,
  *   ttl::removeRectangularBoundary can be called to remove triangles
  *   at the boundary of the triangulation so that the boundary
  *   form the convex hull of the points. <br>
  *   Note that this incremetal scheme will run much faster if the points
  *   have been sorted lexicographically on \e x and \e y.
  *
  * \param dart  An arbitrary CCW dart in the tringulation. Output: a CCW dart incident to the new node.
  * \param point A point (node) to be inserted in the triangulation.
  * \retval bool \c false if the input dart was not valid.
  *
  * \require
  *  - \ref hed::TTLtraits::splitTriangle "TraitsType::splitTriangle" (DartType&, const PointType&)
  *  
  * \using
  *  - ttl::locateTriangle
  *  - ttl::recSwapDelaunay
  *
  * \note For efficiency reasons \e dart should be close to the insertion \e point.
  * \see ttl::removeRectangularBoundary
  */
  template <class TraitsType, class DartType, class PointType>
    bool insertNode(DartType& dart, PointType& point) {
    
    bool found = ttl::locateTriangle<TraitsType>(point, dart);
    if (!found) {
#ifdef DEBUG_TTL
      std::cout << "ERROR: Triangulation::insertNode: NO triangle found. /n";
#endif
      return false;
    }
    
    // ??? can we hide the dart? this is not possible if one triangle only
    TraitsType::splitTriangle(dart,point);
    
    DartType d1 = dart;
    d1.alpha2().alpha1().alpha2().alpha0().alpha1();
    
    DartType d2 = dart;
    d2.alpha1().alpha0().alpha1();
    
    // Preserve a dart as output incident to the node and CCW
    DartType d3 = dart;
    d3.alpha2();
    dart = d3; // and see below
    //DartType dsav = d3;
    d3.alpha0().alpha1();
    
    
    //if (!TraitsType::fixedEdge(d1) && !ttl::isBoundaryEdge(d1)) {
    if (!ttl::isBoundaryEdge(d1)) {
      d1.alpha2();
      recSwapDelaunay<TraitsType>(d1);
    }
    
    //if (!TraitsType::fixedEdge(d2) && !ttl::isBoundaryEdge(d2)) {
    if (!ttl::isBoundaryEdge(d2)) {
      d2.alpha2();
      recSwapDelaunay<TraitsType>(d2);
    }
    
    // Preserve the incoming dart as output incident to the node and CCW
    //d = dsav.alpha2();
    dart.alpha2();
    //if (!TraitsType::fixedEdge(d3) && !ttl::isBoundaryEdge(d3)) {
    if (!ttl::isBoundaryEdge(d3)) {
      d3.alpha2();
      recSwapDelaunay<TraitsType>(d3);
    }
    
    return true;
  }
  
  // ---------------------------------------------------------------------------------------
  template <class TraitsType, class ForwardIterator, class DartType>
    void insertNodes(ForwardIterator first, ForwardIterator last, DartType& dart) {
    
    // Assumes that the dereferenced point objects are pointers.
    // References to the point objects are then passed to TTL.
    
    ForwardIterator it;
    for (it = first; it != last; ++it) {
      bool status = insertNode<TraitsType>(dart, **it);
    }
  }
  
  /** Removes a rectangular boundary of a triangulation as a final step of an
  *   incremental Delaunay triangulation. <br>
  *   The four nodes at the corners will be removed and the resulting triangulation
  *   will have a convex boundary and be Delaunay.
  *
  *   \param dart
  *   A CCW dart at the boundary of the triangulation
  *
  *   \retval dart
  *   A CCW dart at the new boundary
  *
  *   \using
  *   - ttl::removeBoundaryNode
  *
  *   \par Note:
  *   This function requires that the boundary of the triangulation is
  *   a rectangle with four nodes (one in each corner).
  *
  */
  template <class TraitsType, class DartType>
    void removeRectangularBoundary(DartType& dart) {
    
    DartType d_next = dart;
    DartType d_iter;
    
    for (int i = 0; i < 4; i++) {
      d_iter = d_next;
      d_next.alpha0();
      ttl::positionAtNextBoundaryEdge(d_next);
      ttl::removeBoundaryNode<TraitsType>(d_iter);
    }
    
    dart = d_next; // Return a dart at the new boundary
  }
  
  /** Removes the node associated with \e dart and
  *   updates the triangulation to be Delaunay. <br>
  *
  *   \using
  *   - ttl::removeBoundaryNode if \e dart represents a node at the boundary
  *   - ttl::removeInteriorNode if \e dart represents an interior node
  *
  *   \note The node cannot belong to a fixed (constrained) edge that is not
  *         swappable. (An endless loop is likely to occur in this case).
  */
  template <class TraitsType, class DartType>
    void removeNode(DartType& dart) {
    
    if (ttl::isBoundaryNode(dart))
      ttl::removeBoundaryNode<TraitsType>(dart);
    else
      ttl::removeInteriorNode<TraitsType>(dart);
  }
  
  //------------------------------------------------------------------------------
  /** Removes the boundary node associated with \e dart and
  *   updates the triangulation to be Delaunay. <br>
  *
  *   \using
  *   - ttl::swapEdgesAwayFromBoundaryNode
  *   - ttl::optimizeDelaunay
  *
  *   \require
  *   \ref hed::TTLtraits::removeBoundaryTriangle "TraitsType::removeBoundaryTriangle" (Dart&)
  *
  */
  template <class TraitsType, class DartType>
    void removeBoundaryNode(DartType& dart) {
    
    // ... and update Delaunay
    // - CCW dart must be given (for remove)
    // - No dart is delivered back now (but this is possible if
    //   we assume that there is not only one triangle left in the triangulation.
    
    
    // Position at boundary edge and CCW
    if (!ttl::isBoundaryEdge(dart)) {
      dart.alpha1(); // ensures that next function delivers back a CCW dart (if the given dart is CCW)
      ttl::positionAtNextBoundaryEdge(dart);
    }

    std::list<DartType> swapped_edges;
    ttl::swapEdgesAwayFromBoundaryNode<TraitsType>(dart, swapped_edges);
    
    // Remove boundary triangles and remove the new boundary from the list
    // of swapped edges, see below.
    DartType d_iter = dart;
    DartType dnext = dart;
    bool bend = false;
    while (bend == false) {
      dnext.alpha1().alpha2();
      if (ttl::isBoundaryEdge(dnext))
        bend = true; // Stop when boundary
      
      // Generic: Also remove the new boundary from the list of swapped edges
      DartType n_bedge = d_iter;
      n_bedge.alpha1().alpha0().alpha1().alpha2(); // new boundary edge
      
      // ??? can we avoid find if we do this in swap away?
      typename std::list<DartType>::iterator it;
      it = std::find(swapped_edges.begin(), swapped_edges.end(), n_bedge);
      
      if (it != swapped_edges.end())
        swapped_edges.erase(it);
      
      // Remove the boundary triangle
      TraitsType::removeBoundaryTriangle(d_iter);
      d_iter = dnext;
    }
    
    // Optimize Delaunay
    typedef std::list<DartType> DartListType;
    ttl::optimizeDelaunay<TraitsType, DartType, DartListType>(swapped_edges);
  }  
  
  /** Removes the interior node associated with \e dart and
  *   updates the triangulation to be Delaunay. <br>
  *
  *   \using
  *   - ttl::swapEdgesAwayFromInteriorNode
  *   - ttl::optimizeDelaunay   
  *
  *   \require
  *   - \ref hed::TTLtraits::reverse_splitTriangle "TraitsType::reverse_splitTriangle" (Dart&)
  *
  *   \note The node cannot belong to a fixed (constrained) edge that is not
  *         swappable. (An endless loop is likely to occur in this case).
  */
  template <class TraitsType, class DartType>
    void removeInteriorNode(DartType& dart) {
    
    // ... and update to Delaunay.
    // Must allow degeneracy temporarily, see comments in swap edges away
    // Assumes:
    // - revese_splitTriangle does not affect darts
    //   outside the resulting triangle.
    
    // 1) Swaps edges away from the node until degree=3 (generic)
    // 2) Removes the remaining 3 triangles and creates a new to fill the hole
    //    unsplitTriangle which is required
    // 3) Runs LOP on the platelet to obtain a Delaunay triangulation
    // (No dart is delivered as output)
    
    // Assumes dart is counterclockwise
    
    std::list<DartType> swapped_edges;
    ttl::swapEdgesAwayFromInteriorNode<TraitsType>(dart, swapped_edges);
    
    
    // The reverse operation of split triangle:
    // Make one triangle of the three triangles at the node associated with dart
    // TraitsType::
    TraitsType::reverse_splitTriangle(dart);
    
    // ???? Not generic yet if we are very strict:
    // When calling unsplit triangle, darts at the three opposite sides may
    // change!
    // Should we hide them longer away??? This is possible since they cannot
    // be boundary edges.
    // ----> Or should we just require that they are not changed???
    
    // Make the swapped-away edges Delaunay.
    // Note the theoretical result: if there are no edges in the list,
    // the triangulation is Delaunay already
    
    ttl::optimizeDelaunay<TraitsType, DartType>(swapped_edges);
  }
  //@}

   
  //--------------------------------------------------------------------------
  // --------------- Q U E R I E S   G R O U P ------------------------------
  //--------------------------------------------------------------------------
  
  /** @name Topological and Geometric Queries */
  //@{
  // Private functions (might be changed later)
  template <class TopologyElementType, class DartType>
    bool isMemberOfFace(const TopologyElementType& topologyElement, const DartType& dart) {
    
    // Check if the given topology element (node, edge or face) is a member of the face
    // Assumes:
    // - DartType::isMember(TopologyElementType)
    
    DartType dart_iter = dart;
    do {
      if (dart_iter.isMember(topologyElement))
        return true;
      dart_iter.alpha0().alpha1();
    } while (dart_iter != dart);
    
    return false;
  }
  
  template <class TraitsType, class NodeType, class DartType>
    bool locateFaceWithNode(const NodeType& node, DartType& dart_iter) {
    // Locate a face in the topology structure with the given node as a member
    // Assumes:
    // - TraitsType::orient2d(DartType, DartType, NodeType)
    // - DartType::isMember(NodeType)
    // - Note that if false is returned, the node might still be in the
    //   topology structure. Application programmer
    //   should check all if by hypothesis the node is in the topology structure;
    //   see doc. on locateTriangle. 
    
    bool status = locateFaceSimplest<TraitsType>(node, dart_iter);
    if (status == false)
      return status;
    
    // True was returned from locateFaceSimplest, but if the located triangle is
    // degenerate and the node is on the extension of the edges,
    // the node might still be inside. Check if node is a member and return false
    // if not. (Still the node might be in the topology structure, see doc. above
    // and in locateTriangle(const PointType& point, DartType& dart_iter)
    
    return isMemberOfFace(node, dart_iter);
  }
  
  /** Locate the face containing the given point. <br>
  *   It is assumed that the tessellation (e.g. a triangulation) is \e regular in the sense that there
  *   are no holes, the boundary is convex and there are no degenerate faces.
  *
  *   \param dart
  *   An arbitrary CCW dart in the triangulation 
  *
  *   \retval dart
  *   A CCW dart in the located face
  *
  *   \require
  *   - \ref hed::TTLtraits::orient2d "TraitsType::orient2d" (DartType&, DartType&, PointType&)
  *
  *   \note
  *   If false is returned, \e point may still be inside a face if the tessellation is not \e regular
  *   as explained above.
  *
  *   \see ttl::locateTriangle
  */
  template <class TraitsType, class PointType, class DartType>
    bool locateFaceSimplest(const PointType& point, DartType& dart) {
    // Not degenerate triangles if point is on the extension of the edges
    // But inTriangle may be called in case of true (may update to inFace2)
    // Convex boundary
    // no holes
    // convex faces (works for general convex faces)
    // Not specialized for triangles, but ok?
    //
    // TraitsType::orint2d(PointType) is the half open half-plane defined
    // by the dart:
    // n1 = dart.node()
    // n2 = dart.alpha0().node
    // Only the following gives true:
    // ((n2->x()-n1->x())*(point.y()-n1->y()) >= (point.x()-n1->x())*(n2->y()-n1->y()))
    
    DartType dart_start;
    dart_start = dart;
    DartType dart_prev;
    
    DartType d0;
    for (;;) {
      d0 = dart; d0.alpha0();
      if (TraitsType::orient2d(dart, d0, point) >= 0) {
        dart.alpha0().alpha1();
        if (dart == dart_start)
          return true; // left to all edges in face
      }
      else {
        dart_prev = dart;
        dart.alpha2();
        if (dart == dart_prev)
          return false; // iteration to outside boundary
        
        dart_start = dart;
        dart_start.alpha0();
        
        dart.alpha1(); // avoid twice on same edge and ccw in next
      }
    }
  }
  
  
  /** Locates a triangle in a triangulation. <br>
  *   It is assumed that the triangulation is \e regular in the sense that there
  *   are no holes and the boundary is convex.
  *   This function deals with degeneracy to some extent, but round-off errors may still
  *   lead to a wrong result if triangles are degenerate. <br>
  *
  *   \param point A point to be located
  *   \param dart An arbitrary CCW dart in the triangulation
  *
  *   \retval dart
  *   A CCW dart in the located triangle
  *
  *   \return
  *   \c true if a triangle is found; \c false if not. <br>
  *   If \e point is outside the triangulation, in which case \c false is returned,
  *   then the edge associated with \e dart will be at the boundary of the triangulation.
  *    
  *   \using
  *   - ttl::locateFaceSimplest
  *   - ttl::inTriangle
  *
  */
  template <class TraitsType, class PointType, class DartType>
    bool locateTriangle(const PointType& point, DartType& dart) {
    // The purpose is to have a fast and stable procedure that
    //  i) avoids concluding that a point is inside a triangle if it is not inside
    // ii) avoids infinite loops
    
    // Thus, if false is returned, the point might still be inside a triangle in
    // the triangulation. But this will probably only occur in the following cases:
    //  i) There are holes in the triangulation which causes the procedure to stop.
    // ii) The boundary of the triangulation is not convex.
    // ii) There might be degenerate triangles interior to the triangulation, or on the
    //     the boundary, which in some cases might cause the procedure to stop there due
    //     to the logic of the algorithm.
    
    // It is the application programmer's responsibility to check further if false is
    // returned. For example, if by hypothesis the point is inside a triangle
    // in the triangulation and and false is returned, then all triangles in the
    // triangulation should be checked by the application. This can be done using
    // the function:
    // bool inTriangle(const PointType& point, const DartType& dart).
    
    
    // Assumes:
    // - crossProduct2d, scalarProduct2d etc., see functions called
    
    bool status = locateFaceSimplest<TraitsType>(point, dart);
    if (status == false)
      return status;
    
    // There may be degeneracy, i.e., the point might be outside the triangle
    // on the extension of the edges of a degenerate triangle.
    
    // The next call returns true if inside a non-degenerate or a degenerate triangle,
    // but false if the point coincides with the "supernode" in the case where all
    // edges are degenerate.
    return inTriangle<TraitsType>(point, dart);
  }
  
  /** Checks if \e point is inside the triangle associated with \e dart. <br>
  *   A fast an simple function that does not deal with degeneracy.
  *
  *   \param dart
  *   A CCW dart in the triangle
  *
  *   \require
  *   - \ref hed::TTLtraits::orient2d "TraitsType::orient2d" (DartType&, DartType&, PointType&)
  *
  *   \see ttl::inTriangle for a more robust function
  */
  template <class TraitsType, class PointType, class DartType>
    bool inTriangleSimplest(const PointType& point, const DartType& dart) {
    
    // Fast and simple: Do not deal with degenerate faces, i.e., if there is
    // degeneracy, true will be returned if the point is on the extension of the
    // edges of a degenerate triangle
    
    DartType d_iter = dart;
    DartType d0 = d_iter; d0.alpha0();
    if (!TraitsType::orient2d(d_iter, d0, point) >= 0)
      return false;
    
    d_iter.alpha0().alpha1();
    d0 = d_iter; d0.alpha0();
    if (!TraitsType::orient2d(d_iter, d0, point) >= 0)
      return false;
    
    d_iter.alpha0().alpha1();
    d0 = d_iter; d0.alpha0();
    if (!TraitsType::orient2d(d_iter, d0, point) >= 0)
      return false;
    
    return true;
  }
  
  /** Checks if \e point is inside the triangle associated with \e dart. <br>
  *   This function deals with degeneracy to some extent, but round-off errors may still
  *   lead to wrong result if the triangle is degenerate. <br>
  *
  *   \param dart A CCW dart in the triangle
  *
  *   \require
  *    - \ref hed::TTLtraits::crossProduct2d "TraitsType::crossProduct2d" (DartType&, PointType&)
  *    - \ref hed::TTLtraits::scalarProduct2d "TraitsType::scalarProduct2d" (DartType&, PointType&)
  *
  *    \see ttl::inTriangleSimplest
  */
  template <class TraitsType, class PointType, class DartType>
    bool inTriangle(const PointType& point, const DartType& dart) {

      double zero_tol = 1e-08; //12; // @@ Avoid hardcoding!
    
    // SHOULD WE INCLUDE A STRATEGY WITH EDGE X e_1 ETC? TO GUARANTEE THAT
    // ONLY ON ONE EDGE? BUT THIS DOES NOT SOLVE PROBLEMS WITH
    // notInE1 && notInE1.neghbour ?
    
    
    // Returns true if inside (but not necessarily strictly inside)
    // Works for degenerate triangles, but not when all edges are degenerate,
    // and the point coincides with all nodes;
    // then false is always returned.
    
    
    typedef typename TraitsType::real_type real_type;
    
    DartType dart_iter = dart;
    
    real_type cr1 = TraitsType::crossProduct2d(dart_iter, point);
    if (cr1 < -zero_tol)
      return false;
    
    dart_iter.alpha0().alpha1();
    real_type cr2 = TraitsType::crossProduct2d(dart_iter, point);
    
    if (cr2 < -zero_tol)
      return false;
    
    dart_iter.alpha0().alpha1();
    real_type cr3 = TraitsType::crossProduct2d(dart_iter, point);
    if (cr3 < -zero_tol)
      return false;
    
    // All cross products are >= 0
    // Check for degeneracy
    
    if (fabs(cr1) > zero_tol || fabs(cr2) > zero_tol || fabs(cr3) > zero_tol)
      return true; // inside non-degenerate face
    
    // All cross-products are zero, i.e. degenerate triangle, check if inside
    // Strategy: d.scalarProduct2d >= 0 && alpha0(d).d.scalarProduct2d >= 0 for one of
    // the edges. But if all edges are degenerate and the point is on (all) the nodes,
    // then "false is returned".
    
    DartType dart_tmp = dart_iter;
    real_type sc1 = TraitsType::scalarProduct2d(dart_tmp,point);
    real_type sc2 = TraitsType::scalarProduct2d(dart_tmp.alpha0(), point);
    
    if (sc1 >= 0 && sc2 >= 0) {
      // test for degenerate edge
      if (fabs(sc1) > zero_tol || fabs(sc2) > zero_tol) //(sc1 != 0 || sc2 != 0)
        return true; // interior to this edge or on a node (but see comment above)
    }
    
    dart_tmp = dart_iter.alpha0().alpha1();
    sc1 = TraitsType::scalarProduct2d(dart_tmp,point);
    sc2 = TraitsType::scalarProduct2d(dart_tmp.alpha0(),point);
    if (sc1 >= 0 && sc2 >= 0) {
      // test for degenerate edge
      if (fabs(sc1) > zero_tol || fabs(sc2) > zero_tol) //(sc1 != 0 || sc2 != 0)
        return true; // interior to this edge or on a node (but see comment above)
    }
    
    dart_tmp = dart_iter.alpha1();
    sc1 = TraitsType::scalarProduct2d(dart_tmp,point);
    sc2 = TraitsType::scalarProduct2d(dart_tmp.alpha0(),point);
    if (sc1 >= 0 && sc2 >= 0) {
      // test for degenerate edge
      if (fabs(sc1) > zero_tol || fabs(sc2) > zero_tol) //(sc1 != 0 || sc2 != 0)
        return true; // interior to this edge or on a node (but see comment above)
    }
    
    // Not on any of the edges of the degenerate triangle.
    // The only possibility for the point to be "inside" is that all edges are degenerate
    // and the point coincide with all nodes. So false is returned in this case.
    
    return false;
  }
  
  
  template <class DartType>
    void getAdjacentTriangles(const DartType& dart,
    DartType& t1, DartType& t2, DartType& t3) {
    
    DartType dart_iter = dart;
    
    // add first
    if (dart_iter.alpha2() != dart) {
      t1 = dart_iter;
      dart_iter = dart;
    }
    
    // add second
    dart_iter.alpha0();
    dart_iter.alpha1();
    DartType dart_prev = dart_iter;
    if ((dart_iter.alpha2()) != dart_prev)
    {
      t2 = dart_iter;
      dart_iter = dart_prev;
    }
    
    // add third
    dart_iter.alpha0();
    dart_iter.alpha1();
    dart_prev = dart_iter;
    if ((dart_iter.alpha2()) != dart_prev)
      t3 = dart_iter;
  }
  
  /** Given a dart with an associating edge at the boundary of a topology structure.
  *   Get the boundary as a sequence of darts, where the edges associated with the darts
  *   are boundary edges.
  *   The first dart in the sequence will be the given one, and the other have the same
  *   orientation (CCW or CW) as the first.
  *   Assumes that the given dart is at the boundary.
  *
  *   \require
  *   - DartListType::push_back (DartType&)
  */
  template <class DartType, class DartListType>
    void getBoundary(const DartType& dart, DartListType& boundary) {
    // assumes the given dart is at the boundary (by edge)
    
    DartType dart_iter(dart);
    boundary.push_back(dart_iter); // Given dart as first element
    dart_iter.alpha0();
    positionAtNextBoundaryEdge(dart_iter);
    
    while (dart_iter != dart) {
      boundary.push_back(dart_iter);
      dart_iter.alpha0();
      positionAtNextBoundaryEdge(dart_iter);
    }
  }
  
  /*
  // Asumes a fixed point (a boundary edge) is geven
  // 
  template <class DartType>
  class boundary_1_Iterator { // i.e. "circulator"
  
    DartType current_;
    public:
    boundaryEdgeIterator(const DartType& dart) {current_ = dart;}
    DartType& operator * () const {return current_;}
    void operator ++ () {current_.alpha0(); positionAtNextBoundaryEdge(current_);}
    };
  */  
  
  /** Checks if the edge associated with \e dart is at
  *   the boundary of the triangulation. Implements:
  *   \code
  *   DartType dart_iter = dart;
  *   if (dart_iter.alpha2() == dart)
  *     return true;
  *   else
  *     return false;
  *   \endcode
  */
  template <class DartType>
    bool isBoundaryEdge(const DartType& dart) {
    
    DartType dart_iter = dart;
    if (dart_iter.alpha2() == dart)
      return true;
    else
      return false;
  }
  
  
  /** Checks if the face associated with \e dart is at
  *   the boundary of the triangulation.
  */
  template <class DartType>
    bool isBoundaryFace(const DartType& dart) {
    
    // Strategy: boundary if alpha2(d)=d
    
    DartType dart_iter(dart);
    DartType dart_prev;
    
    do {      
      dart_prev = dart_iter;
      
      if (dart_iter.alpha2() == dart_prev)
        return true;
      else
        dart_iter = dart_prev; // back again
      
      dart_iter.alpha0();
      dart_iter.alpha1();
      
    } while (dart_iter != dart);
    
    return false;
  }
  
  //----------------------------------------------------
  /** Checks if the node associated with \e dart is at
  *   the boundary of the triangulation.
  */
  template <class DartType>
    bool isBoundaryNode(const DartType& dart) {
    
    // Strategy: boundary if alpha2(d)=d
    
    DartType dart_iter(dart);
    DartType dart_prev;
    
    // If input dart is reached again, then internal node
    // If alpha2(d)=d, then boundary
    
    do {
      dart_iter.alpha1();
      dart_prev = dart_iter;
      dart_iter.alpha2();
      
      if (dart_iter == dart_prev)
        return true;
      
    } while (dart_iter != dart);
    
    
    return false;
  }
  
  /** Get degree of the node associated with \e dart. <br>
  \par Definition:
  The \em degree (or valency) of a node \e V in a triangulation,
  is defined as the number of edges incident with \e V, i.e.,
  the number of edges joining \e V with another node in the triangulation.
  */
  template <class DartType>
    int getDegreeOfNode(const DartType& dart) {
    
    DartType dart_iter(dart);
    DartType dart_prev;
    
    // If input dart is reached again, then interior node
    // If alpha2(d)=d, then boundary
    
    int degree = 0;
    bool boundaryVisited = false;
    do {
      dart_iter.alpha1();
      degree++;
      dart_prev = dart_iter;
      
      dart_iter.alpha2();
      
      if (dart_iter == dart_prev) {
        if (!boundaryVisited) {
          boundaryVisited = true;
          // boundary is reached first time, count in the reversed direction
          degree++; // count the start since it is not done above
          dart_iter = dart;
          dart_iter.alpha2();
        }
        else
          return degree;
      }
      
    } while (dart_iter != dart);
    
    
    return degree;
  }
  
  
  
  //------------------------------------------------------------------
  // Modification of getDegreeOfNode:
  // Strategy, reverse the list and start in the other direction if the boundary
  // is reached. NB. copying of darts but ok., or we could have collected pointers,
  // but the memory management.
  
  // NOTE: not symmetry if we choose to collect opposite edges
  //       now we collect darts with radiating edges
  
  // Remember that we must also copy the node, but ok with push_back
  // The size of the list will be the degree of the node
  
  // No CW/CCW since topology only
  
  
  // Each dart consists of an incident edge and an adjacent node.
  // But note that this is only how we interpret the dart in this implementation.
  // Given this list, how can we find the opposite edges:
  //   We can perform alpha1 on each, but for boundary nodes we will get one edge twice.
  //   But this is will always be the last dart!
  // The darts in the list are in sequence and starts with the alpha0(dart)
  // alpha0, alpha1 and alpha2
  
  template <class DartType>
    void getNeighborNodes(const DartType& dart, std::list<DartType>& node_list, bool& boundary) {
    
    DartType dart_iter(dart);
    
    dart_iter.alpha0(); // position the dart at an opposite node
    
    DartType dart_prev = dart_iter;
    
    bool start_at_boundary = false;
    dart_iter.alpha2();
    if (dart_iter == dart_prev)
      start_at_boundary = true;
    else
      dart_iter = dart_prev; // back again 
    
    
    DartType dart_start = dart_iter;
    
    do {
      node_list.push_back(dart_iter);
      dart_iter.alpha1();
      dart_iter.alpha0();
      dart_iter.alpha1();
      dart_prev = dart_iter;
      dart_iter.alpha2();
      if (dart_iter == dart_prev) {
        // boundary reached
        boundary = true;
        if (start_at_boundary == true) {
          // add the dart which now is positioned at the opposite boundary
          node_list.push_back(dart_iter);
          return;
        }
        else {
          // call the function again such that we start at the boundary
          // first clear the list and reposition to the initial node
          dart_iter.alpha0();
          node_list.clear();
          getNeighborNodes(dart_iter, node_list, boundary);
          return; // after one recursive step
        }
      }
    } while (dart_iter != dart_start);
    
    boundary = false;
  }
  
  /** Get the 0-orbit around an interior node.
  *
  *   \param dart
  *   A dart (CCW or CW) positioned at an \e interior node.
  *
  *   \retval orbit
  *   Sequence of darts with one orbit for each arc. All the darts have the same
  *   orientation (CCW or CW) as \e dart, and \e dart is the first element
  *   in the sequence.
  *
  *   \require
  *   - DartListType::push_back (DartType&)
  *
  *   \see ttl::get_0_orbit_boundary
  */
  template <class DartType, class DartListType>
    void get_0_orbit_interior(const DartType& dart, DartListType& orbit) {
    
    DartType d_iter = dart;
    orbit.push_back(d_iter);
    d_iter.alpha1().alpha2();
    
    while (d_iter != dart) {
      orbit.push_back(d_iter);
      d_iter.alpha1().alpha2();
    }
  }
  
  /** Get the 0-orbit around a node at the boundary
  *
  *   \param dart
  *   A dart (CCW or CW) positioned at a \e boundary \e node and at a \e boundary \e edge.
  *
  *   \retval orbit
  *   Sequence of darts with one orbit for each arc. All the darts, \e exept \e the \e last,
  *   have all the same orientation (CCW or CW) as \e dart, and \e dart is the first element
  *   in the sequence.
  *
  *   \require
  *   - DartListType::push_back (DartType&)
  *
  *   \note
  *   - The last dart in the sequence have opposite orientation compared the others!
  *
  *   \see ttl::get_0_orbit_interior
  */
  template <class DartType, class DartListType>
    void get_0_orbit_boundary(const DartType& dart, DartListType& orbit) {
    
    DartType dart_prev;
    DartType d_iter = dart;
    do {
      orbit.push_back(d_iter);
      d_iter.alpha1();
      dart_prev = d_iter;
      d_iter.alpha2();
    } while (d_iter != dart_prev);
    
    orbit.push_back(d_iter); // the last one with opposite orientation
  }
  
  /** Checks if the two darts belong to the same 0-orbit, i.e.,
  *   if they share a node. <br>
  *   d1 and/or d2 can be CCW or CW. <br>
  *   (This function also examines if the the node associated with
  *   d1 is at the boundary, which slows down the function (slightly).
  *   If it is known that the node associated with d1 is an interior
  *   node and a faster version is needed, the user should implement his/her
  *   own version.)
  */
  template <class DartType>
    bool same_0_orbit(const DartType& d1, const DartType& d2) {
    
    // Two copies of the same dart
    DartType d_iter = d2;
    DartType d_end = d2;
    
    if (ttl::isBoundaryNode(d_iter)) {
      // position at both boundary edges
      ttl::positionAtNextBoundaryEdge(d_iter);
      d_end.alpha1();
      ttl::positionAtNextBoundaryEdge(d_end);
    }
    
    for (;;) {
      if (d_iter == d1)
        return true;
      d_iter.alpha1();
      if (d_iter == d1)
        return true;
      d_iter.alpha2();
      if (d_iter == d_end)
        break;
    }
    
    return false;
  }
  
  /** Checks if the two darts belong to the same 1-orbit, i.e.,
  *   if they share an edge. <br>
  *   d1 and/or d2 can be CCW or CW
  */
  template <class DartType>
    bool same_1_orbit(const DartType& d1, const DartType& d2) {
    
    DartType d_iter = d2;
    // (Also works at the boundary)
    if (d_iter == d1 || d_iter.alpha0() == d1 || d_iter.alpha2() == d1 || d_iter.alpha0() == d1)
      return true;
    return false;
  }
  
  /** Checks if the two darts belong to the same 2-orbit, i.e.,
  *   if they lie in the same triangle. <br>
  *   d1 and/or d2 can be CCW or CW
  */
  template <class DartType>
    bool same_2_orbit(const DartType& d1, const DartType& d2) {
    
    DartType d_iter = d2;
    if (d_iter == d1 || d_iter.alpha0() == d1 ||
      d_iter.alpha1() == d1 || d_iter.alpha0() == d1 ||
      d_iter.alpha1() == d1 || d_iter.alpha0() == d1)
      return true;
    return false;
  }
  
  template <class TraitsType, class DartType>
    bool degenerateTriangle(const DartType& dart) {
    
    // Check if triangle is degenerate
    // Assumes CCW dart
    
    DartType d1 = dart;
    DartType d2 = d1;
    d2.alpha1();
    if (TraitsType::crossProduct2d(d1,d2) == 0)
      return true;
    
    return false;
  }
  
  /** Checks if the edge associated with \e dart is swappable, i.e., if the edge
  *   is a diagonal in a \e strictly convex (or convex) quadrilateral.
  *   
  *   \param allowDegeneracy
  *   If set to true, the function will also return true if the numerical calculations
  *   indicate that the quadrilateral is convex only, and not necessarily strictly
  *   convex.
  *
  *   \require
  *   - \ref hed::TTLtraits::crossProduct2d "TraitsType::crossProduct2d" (Dart&, Dart&)
  */
  template <class TraitsType, class DartType>
    bool swappableEdge(const DartType& dart, bool allowDegeneracy = false) {
    
    // How "safe" is it?
    
    if (isBoundaryEdge(dart))
      return false;
    
    // "angles" are at the diagonal
    DartType d1 = dart;
    d1.alpha2().alpha1();
    DartType d2 = dart;
    d2.alpha1();
    if (allowDegeneracy) {
      if (TraitsType::crossProduct2d(d1,d2) < 0.0)
        return false;
    }
    else {
      if (TraitsType::crossProduct2d(d1,d2) <= 0.0)
        return false;
    }
    
    // Opposite side (still angle at the diagonal)
    d1 = dart;
    d1.alpha0();
    d2 = d1;
    d1.alpha1();
    d2.alpha2().alpha1();
    
    if (allowDegeneracy) {
      if (TraitsType::crossProduct2d(d1,d2) < 0.0)
        return false;
    }
    else {
      if (TraitsType::crossProduct2d(d1,d2) <= 0.0)
        return false;
    }
    return true;
  }
  
  /** Given a \e dart, CCW or CW, positioned in a 0-orbit at the boundary of a tessellation.
  *   Position \e dart at a boundary edge in the same 0-orbit. <br>
  *   If the given \e dart is CCW, \e dart is positioned at the left boundary edge
  *   and will be CW. <br>
  *   If the given \e dart is CW, \e dart is positioned at the right boundary edge
  *   and will be CCW. <br>
  *
  *   \note
  *   - The given \e dart must have a source node at the boundary, otherwise an
  *     infinit loop occurs.
  */
  template <class DartType>
    void positionAtNextBoundaryEdge(DartType& dart) {
    
    DartType dart_prev;
    
    // If alpha2(d)=d, then boundary
    
    //old convention: dart.alpha0();
    do {
      dart.alpha1();
      dart_prev = dart;
      dart.alpha2();
    } while (dart != dart_prev);
  }

  /** Checks if the boundary of a triangulation is convex.
  *
  * \param dart A CCW dart at the boundary of the triangulation
  *
  * \require
  * \ref hed::TTLtraits::crossProduct2d "TraitsType::crossProduct" (const Dart&, const Dart&)
  *
  */
  template <class TraitsType, class DartType>
  bool convexBoundary(const DartType& dart) {
    
    std::list<DartType> blist;
    ttl::getBoundary(dart, blist);
    
    int no = blist.size();
    typename std::list<DartType>::const_iterator bit = blist.begin();
    DartType d1 = *bit;
    ++bit;
    DartType d2;
    bool convex = true;
    for (; bit != blist.end(); ++bit) {
      d2 = *bit;
      double crossProd = TraitsType::crossProduct2d(d1, d2);
      if (crossProd < 0.0) {
        //cout << "!!! Boundary is NOT convex: crossProd = " << crossProd << endl;
        convex = false;
        return convex;
      }
      d1 = d2;
    }
    
    // Check the last angle
    d2 = *blist.begin();
    double crossProd = TraitsType::crossProduct2d(d1, d2);
    if (crossProd < 0.0) {
      //cout << "!!! Boundary is NOT convex: crossProd = " << crossProd << endl;
      convex = false;
    }
    
    //if (convex)
    //  cout << "\n---> Boundary is convex\n" << endl;
    //cout << endl;
    return convex;
  }
  //@} // End of queries group


  //--------------------------------------------------------------------------
  // --------- U T I L I T I E S  F O R  D E L A U N A Y   G R O U P ---------
  //--------------------------------------------------------------------------

  /** @name Utilities for Delaunay Triangulation */
  //@{
  /** Optimize edges in the given sequence according to the
  *   Delaunay criterion, i.e., such that the edge will fulfil the
  *   circumcircle criterion (or equivalently the MaxMin 
  *   angle criterion) with respect to the quadrilaterals where
  *   they are diagonals. <br>
  *
  *   \param elist The sequence of edges
  *   \param end   Past-the-end of the last edge element to be optimized.
  *                The default is equivalent to \c elist.end().
  *                
  *
  *   \require
  *   - \ref hed::TTLtraits::swapEdge "TraitsType::swapEdge" (DartType&)
  *     where the dart is delivered back in a position as
  *     seen if it was glued to the edge when swapping (rotating) the edge CCW
  *
  *   \using
  *   - ttl::swapTestDelaunay
  *
  */
  template <class TraitsType, class DartType, class DartListType>
  void optimizeDelaunay(DartListType& elist)
  {
      optimizeDelaunay<TraitsType, DartType, DartListType>(elist, elist.end());
  }

  template <class TraitsType, class DartType, class DartListType>
  void optimizeDelaunay(DartListType& elist, const typename DartListType::iterator end)
  {
    // CCW darts
    // Optimize here means Delaunay, but could be any criterion by
    // requiring a "should swap" in the traits class, or give
    // a function object?
    // Assumes that elist has only one dart for each arc.
    // Darts outside the quadrilateral are preserved
    
    // For some data structures it is possible to preserve
    // all darts when swapping. Thus a preserve_darts_when swapping
    // ccould be given to indicate this and we would gain performance by avoiding
    // find in list.
    
    // Requires that swap retuns a dart in the "same position when rotated CCW"
    // (A vector instead of a list may be better.)
    
    // Avoid cycling by more extensive circumcircle test
    bool cycling_check = true;
    bool optimal = false;
    typename DartListType::iterator it;

    while(!optimal) {
      optimal = true;
      for (it = elist.begin(); it != end; ++it) {
        if (ttl::swapTestDelaunay<TraitsType>(*it, cycling_check)) {

          // Preserve darts. Potential darts in the list are:
          // - The current dart
          // - the four CCW darts on the boundary of the quadrilateral
          // (the current arc has only one dart)
          
          ttl::swapEdgeInList<TraitsType, DartType>(it, elist);
          
          optimal = false;
        } // end if should swap
      } // end for
    } // End pass
  }
  
  /** Checks if the edge associated with \e dart should be swapped according
  *   to the Delaunay criterion, i.e., the \e circumcircle criterion (or
  *   equivalently the \e MaxMin angle criterion).
  *
  *   \param cycling_check
  *   Must be set to \c true when used in connection with optimization algorithms,
  *   e.g., optimizeDelaunay. This will avoid cycling and infinite loops in nearly
  *   neutral cases.
  *
  *   \require
  *   - \ref hed::TTLtraits::scalarProduct2d "TraitsType::scalarProduct2d" (DartType& ,DartType&)
  *   - \ref hed::TTLtraits::crossProduct2d "TraitsType::crossProduct2d" (DartType& ,DartType&)
  */
  template <class TraitsType, class DartType>
#if ((_MSC_VER > 0) && (_MSC_VER < 1300))//#ifdef _MSC_VER
    bool swapTestDelaunay(const DartType& dart, bool cycling_check = false)
#else
    bool swapTestDelaunay(const DartType& dart, bool cycling_check)
#endif
  { 
    // The general strategy is taken from Cline & Renka. They claim that
    // their algorithm insure numerical stability, but experiments show
    // that this is not correct for neutral, or almost neutral cases.
    // I have extended this strategy (without using tolerances) to avoid
    // cycling and infinit loops when used in connection with LOP algorithms;
    // see the comments below.
    
    typedef typename TraitsType::real_type real_type;
    
    if (isBoundaryEdge(dart))
      return false;
    
    DartType v11 = dart;
    v11.alpha1().alpha0();
    DartType v12 = v11;
    v12.alpha1();
    
    DartType v22 = dart;
    v22.alpha2().alpha1().alpha0();
    DartType v21 = v22;
    v21.alpha1();
    
    real_type cos1 = TraitsType::scalarProduct2d(v11,v12);
    real_type cos2 = TraitsType::scalarProduct2d(v21,v22);
    
    // "Angles" are opposite to the diagonal.
    // The diagonals should be swapped iff (t1+t2) .gt. 180
    // degrees. The following two tests insure numerical
    // stability according to Cline & Renka. But experiments show
    // that cycling may still happen; see the aditional test below.
    if (cos1 >= 0  &&  cos2 >= 0) // both angles are grater or equual 90
      return false;
    if (cos1 < 0  &&  cos2 < 0) // both angles are less than 90
      return true;
    
    real_type sin1 = TraitsType::crossProduct2d(v11,v12);
    real_type sin2 = TraitsType::crossProduct2d(v21,v22);
    real_type sin12 = sin1*cos2 + cos1*sin2;
    if (sin12 >= 0) // equality represents a neutral case
      return false;
    
    if (cycling_check) {
      // situation so far is sin12 < 0. Test if this also
      // happens for the swapped edge.
      
      // The numerical calculations so far indicate that the edge is
      // not Delaunay and should not be swapped. But experiments show that
      // in neutral cases, or almost neutral cases, it may happen that
      // the swapped edge may again be found to be not Delaunay and thus
      // be swapped if we return true here. This may lead to cycling and
      // an infinte loop when used, e.g., in connection with optimizeDelaunay.
      //
      // In an attempt to avoid this we test if the swapped edge will
      // also be found to be not Delaunay by repeating the last test above
      // for the swapped edge.
      // We now rely on the general requirement for TraitsType::swapEdge which
      // should deliver CCW dart back in "the same position"; see the general
      // description. This will insure numerical stability as the next calculation
      // is the same as if this function was called again with the swapped edge.
      // Cycling is thus impossible provided that the initial tests above does
      // not result in ambiguity (and they should probably not do so).
      
      v11.alpha0();
      v12.alpha0();
      v21.alpha0();
      v22.alpha0();
      // as if the edge was swapped/rotated CCW
      cos1 = TraitsType::scalarProduct2d(v22,v11);
      cos2 = TraitsType::scalarProduct2d(v12,v21);
      sin1 = TraitsType::crossProduct2d (v22,v11);
      sin2 = TraitsType::crossProduct2d (v12,v21);
      sin12 = sin1*cos2 + cos1*sin2;
      if (sin12 < 0) {
        // A neutral case, but the tests above lead to swapping
        return false;
      }
    }
    
    return true;
  }
  
  
  //-----------------------------------------------------------------------------
  //
  //        x
  //      /   \       ----------------
  //     /  |  \      Darts:
  //oe2 /   |   \     oe2 = oppEdge2
  //   x....|.... x
  //    \  d|  d /    d   = diagonal (input and output)
  //     \  |   /     
  //   oe1 \   /      oe1 = oppEdge1
  //        x         ----------------
  //
  //---------------------------------------------------------------------------
  /** Reqursively swaps edges in the triangulation according to the Delaunay
  *   criterion. <br>
  *   
  *   \param diagonal A CCW dart representing the edge where the recursion
  *                   starts from.
  *
  *   \require
  *   - \ref hed::TTLtraits::swapEdge "TraitsType::swapEdge" (DartType&) <br>
  *     \b Note: must be implemented such that darts outside the quadrilateral
  *     are not affected by the swap.
  *
  *   \using
  *   - Calls itself recursively
  */
  template <class TraitsType, class DartType>
    void recSwapDelaunay(DartType& diagonal)
  {     
    if (!ttl::swapTestDelaunay<TraitsType, DartType>(diagonal)) // ??? ttl::swapTestDelaunay also checks if boundary, so this can be optimized
      return;
    
    // Get the other "edges" of the current triangle; see illustration above.
    DartType oppEdge1 = diagonal;    
    oppEdge1.alpha1();
    bool b1;
    if (ttl::isBoundaryEdge(oppEdge1))
      b1 = true;
    else {
      b1 = false;
      oppEdge1.alpha2();
    }
    
    
    DartType oppEdge2 = diagonal;
    oppEdge2.alpha0().alpha1().alpha0();
    bool b2;
    if (ttl::isBoundaryEdge(oppEdge2))
      b2 = true;
    else {
      b2 = false;
      oppEdge2.alpha2();
    }
    
    // Swap the given diagonal
    TraitsType::swapEdge(diagonal);
    
    if (!b1)
      recSwapDelaunay<TraitsType>(oppEdge1);
    if (!b2)
      recSwapDelaunay<TraitsType>(oppEdge2);
  }
  
  /** Swaps edges away from the (interior) node associated with
  *   \e dart such that that exactly three edges remain incident
  *   with the node. <br>
  *   This function is used as a first step in ttl::removeInteriorNode
  *   
  *
  *   \retval dart
  *   A CCW dart incident with the node
  *
  *   \par Asumes:
  *   - The node associated with \e dart is interior to the
  *     triangulation.
  *
  *   \require
  *   - \ref hed::TTLtraits::swapEdge "TraitsType::swapEdge" (DartType& dart)
  *     where \c dart is delivered back in a position as
  *     seen if it was glued to the edge when swapping (rotating) the edge CCW
  *
  *   \note
  *   - A degenerate triangle may be left at the node.
  *   - The function is not unique as it depends on which dart
  *     at the node that is given as input.
  *
  *   \see ttl::swapEdgesAwayFromBoundaryNode
  */
  template <class TraitsType, class DartType, class ListType>
    void swapEdgesAwayFromInteriorNode(DartType& dart, ListType& swapped_edges) {
    
    // Same iteration as in fixEdgesAtCorner, but not boundary    
    DartType dnext = dart;
    
    // Allow degeneracy, otherwise we might end up with degree=4.
    // For example, the reverse operation of inserting a point on an
    // existing edge gives a situation where all edges are non-swappable.
    // Ideally, degeneracy in this case should be along the actual node,
    // but there is no strategy for this now.
    // ??? An alternative here is to wait with degeneracy till we get an
    // infinite loop with degree > 3.
    bool allowDegeneracy = true;
    
    int degree = ttl::getDegreeOfNode(dart);
    DartType d_iter;
    while (degree > 3) {
      d_iter = dnext;
      dnext.alpha1().alpha2();
      
      if (ttl::swappableEdge<TraitsType>(d_iter, allowDegeneracy)) {
        TraitsType::swapEdge(d_iter); // swap the edge away
        // Collect swapped edges in the list
        // "Hide" the dart on the other side of the edge to avoid it being changed for
        // other swaps
        DartType swapped_edge = d_iter; // it was delivered back
        swapped_edge.alpha2().alpha0(); // CCW (if not at boundary)
        swapped_edges.push_back(swapped_edge);
        
        degree--;
      }
    }
    // Output, incident to the node
    dart = dnext;
  }
  
  
  /** Swaps edges away from the (boundary) node associated with
  *   \e dart in such a way that when removing the edges that remain incident
  *   with the node, the boundary of the triangulation will be convex. <br>
  *
  *   \retval dart
  *   A CCW dart incident with the node
  *
  *   \require
  *   - \ref hed::TTLtraits::swapEdge "TraitsType::swapEdge" (DartType& dart)
  *     where \c dart is delivered back in a position as
  *     seen if it was glued to the edge when swapping (rotating) the edge CCW
  *
  *   \par Asumes:
  *   - The node associated with \e dart is a boundary node.
  *
  *   \see ttl::swapEdgesAwayFromInteriorNode
  */
  template <class TraitsType, class DartType, class ListType>
    void swapEdgesAwayFromBoundaryNode(DartType& dart, ListType& swapped_edges) {
    
    // All darts that are swappable.
    // To treat collinear nodes at an existing boundary, we must allow degeneracy
    // when swapping to the boundary.
    // dart is CCW and at the boundary.
    // The 0-orbit runs CCW
    // Deliver the dart back in the "same position".
    // Assume for the swap in the traits class:
    // - A dart on the swapped edge is delivered back in a position as
    //   seen if it was glued to the edge when swapping (rotating) the edge CCW
    
    //int degree = ttl::getDegreeOfNode(dart);
    
passes:
  
  // Swap swappable edges that radiate from the node away
  DartType d_iter = dart; // ???? can simply use dart
  d_iter.alpha1().alpha2(); // first not at boundary
  DartType d_next = d_iter;
  bool bend = false;
  bool swapped_next_to_boundary = false;
  bool swapped_in_pass = false;
  
  bool allowDegeneracy; // = true;
  DartType tmp1, tmp2;
  
  while (!bend) {
    
    d_next.alpha1().alpha2();
    if (ttl::isBoundaryEdge(d_next))
      bend = true;  // then it is CW since alpha2
    
    // To allow removing among collinear nodes at the boundary,
    // degenerate triangles must be allowed
    // (they will be removed when used in connection with removeBoundaryNode)
    tmp1 = d_iter; tmp1.alpha1();
    tmp2 = d_iter; tmp2.alpha2().alpha1(); // don't bother with boundary (checked later)
    
    if (ttl::isBoundaryEdge(tmp1) && ttl::isBoundaryEdge(tmp2))
      allowDegeneracy = true;
    else
      allowDegeneracy = false;
    
    if (ttl::swappableEdge<TraitsType>(d_iter, allowDegeneracy)) {
      TraitsType::swapEdge(d_iter);
      
      // Collect swapped edges in the list
      // "Hide" the dart on the other side of the edge to avoid it being changed for
      // other swapps
      DartType swapped_edge = d_iter; // it was delivered back
      swapped_edge.alpha2().alpha0(); // CCW
      swapped_edges.push_back(swapped_edge);
      
      //degree--; // if degree is 2, or bend=true, we are done
      swapped_in_pass = true;
      if (bend)
        swapped_next_to_boundary = true;
    }
    if (!bend)
      d_iter = d_next;
  }
  
  // Deliver a dart as output in the same position as the incoming dart
  if (swapped_next_to_boundary) {
    // Assume that "swapping is CCW and dart is preserved in the same position
    d_iter.alpha1().alpha0().alpha1();  // CW and see below
  }
  else {
    d_iter.alpha1(); // CW and see below
  }
  ttl::positionAtNextBoundaryEdge(d_iter); // CCW 
  
  dart = d_iter; // for next pass or output
  
  // If a dart was swapped in this iteration we must run it more
  if (swapped_in_pass)
    goto passes;
  }
  
  /**  Swap the the edge associated with iterator \e it and update affected darts
  *   in \e elist accordingly. <br> 
  *   The darts affected by the swap are those in the same quadrilateral.
  *   Thus, if one want to preserve one or more of these darts on should
  *   keep them in \c elist.
  */
  template <class TraitsType, class DartType, class DartListType>
    void swapEdgeInList(const typename DartListType::iterator& it, DartListType& elist) {

    typename DartListType::iterator it1, it2, it3, it4;
    DartType dart(*it);
    
    //typename TraitsType::DartType d1 = dart; d1.alpha2().alpha1();
    //typename TraitsType::DartType d2 =   d1; d2.alpha0().alpha1();
    //typename TraitsType::DartType d3 = dart; d3.alpha0().alpha1();
    //typename TraitsType::DartType d4 =   d3; d4.alpha0().alpha1();
    DartType d1 = dart; d1.alpha2().alpha1();
    DartType d2 =   d1; d2.alpha0().alpha1();
    DartType d3 = dart; d3.alpha0().alpha1();
    DartType d4 =   d3; d4.alpha0().alpha1();
    
    // Find pinters to the darts that may change.
    // ??? Note, this is not very efficient since we must use find, which is O(N),
    // four times.
    // - Solution?: replace elist with a vector of std::pair (dart,number)
    //   and avoid std::find?
    // - make a function for swapping generically?
    // - sould we use another container type or,
    // - erase them and reinsert?
    // - or use two lists?
    it1 = std::find(elist.begin(), elist.end(), d1);
    it2 = std::find(elist.begin(), elist.end(), d2);
    it3 = std::find(elist.begin(), elist.end(), d3);
    it4 = std::find(elist.begin(), elist.end(), d4);
    
    TraitsType::swapEdge(dart);
    // Update the current dart which may have changed
    *it = dart;
    
    // Update darts that may have changed again (if they were present)
    // Note that dart is delivered back after swapping
    if (it1 != elist.end()) {
      d1 = dart; d1.alpha1().alpha0();
      *it1 = d1;
    }
    if (it2 != elist.end()) {
      d2 = dart; d2.alpha2().alpha1();
      *it2 = d2;
    }
    if (it3 != elist.end()) {
      d3 = dart; d3.alpha2().alpha1().alpha0().alpha1();
      *it3 = d3;
    }
    if (it3 != elist.end()) {
      d4 = dart; d4.alpha0().alpha1();
      *it4 = d4;
    }
  }
  
  //@} // End of Utility for Delaunay group

  
}; // End ttl namespace scope (but other files may also contain functions for ttl)


  //--------------------------------------------------------------------------
  // --------------- C O N S T R A I N E D   G R O U P -----------------------
  //                 Still namespace ttl
  //--------------------------------------------------------------------------
#include "ttl_constr.h"

#endif //_TTL_H_
