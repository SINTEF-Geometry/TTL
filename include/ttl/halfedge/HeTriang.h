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

#ifndef _HE_TRIANG_H_
#define _HE_TRIANG_H_


#define TTL_USE_NODE_ID   // Each node gets it's own unique id
#define TTL_USE_NODE_FLAG // Each node gets a flag (can be set to true or false)


#include <list>
#include <vector>
#include <iostream>
#include <fstream>
#include <ttl/ttl.h>
#include <ttl/ttl_util.h>
#include <ttl/utils/HandleId.h>
#include <ttl/utils/Handle.h>


// using namespace std;

//--------------------------------------------------------------------------------------------------
// The half-edge data structure
//--------------------------------------------------------------------------------------------------

namespace hed {

  //------------------------------------------------------------------------------------------------
  // Node class for data structures
  //------------------------------------------------------------------------------------------------

  /** \class Node
   *   \brief \b Node class for data structures (Inherits from HandleId)
   *
   *   \note
   *   - To enable node IDs, TTL_USE_NODE_ID must be defined.
   *   - To enable node flags, TTL_USE_NODE_FLAG must be defined.
   *   - TTL_USE_NODE_ID and TTL_USE_NODE_FLAG should only be enabled if this functionality is
   *     required by the application, because they increase the memory usage for each Node object.
   */

  class Node : public virtual HandleId {

#ifdef TTL_USE_NODE_ID
    /// TTL_USE_NODE_ID must be defined
    static int id_count;
#endif

    double x_, y_, z_;

#ifdef TTL_USE_NODE_ID
    /// A unique id for each node (TTL_USE_NODE_ID must be defined)
    int id_;
#endif

#ifdef TTL_USE_NODE_FLAG
    /// TTL_USE_NODE_FLAG must be defined
    bool flag_;
#endif

public:
    /// Default constructor
    Node() { init(0, 0, 0); }

    /// Constructor
    Node(double x, double y, double z = 0.0) { init(x, y, z); }

    /// Destructor
    ~Node() {}

    /// Initialize the position
    void init(double x, double y, double z) { 

#ifdef TTL_USE_NODE_ID
      id_ = id_count++;
#endif

      x_ = x; y_ = y; z_ = z; 
    }

    /// Returns the x-coordinate
    const double& x() const { return x_; }

    /// Returns the y-coordinate
    const double& y() const { return y_; }

    /// Returns the z-coordinate
    const double& z() const { return z_; }

#ifdef TTL_USE_NODE_ID
    /// Returns the id (TTL_USE_NODE_ID must be defined)
    const int& id() const { return id_; }
#endif

#ifdef TTL_USE_NODE_FLAG
    /// Sets the flag (TTL_USE_NODE_FLAG must be defined)
    void setFlag(bool flag) { flag_ = flag; }

    /// Returns the flag (TTL_USE_NODE_FLAG must be defined)
    const bool& getFlag() const { return flag_; }
#endif

  }; // End og class Node

  
  //------------------------------------------------------------------------------------------------
  // Edge class in the half-edge data structure
  //------------------------------------------------------------------------------------------------

  /** \class Edge
  *   \brief \b %Edge class in the in the half-edge data structure.
  */

  class Edge {
    
    Handle<Node> sourceNode_;
    Edge*        twinEdge_;
    Edge*        nextEdgeInFace_;

    struct {
      bool isLeadingEdge_;
      bool isConstrained_;
    } flags_;

  public:
    /// Constructor
    Edge() { sourceNode_ = NULL; twinEdge_ = NULL; nextEdgeInFace_ = NULL; 
      flags_.isLeadingEdge_ = false; flags_.isConstrained_ = false; }

    /// Destructor
    ~Edge() { if(twinEdge_) twinEdge_->setTwinEdge(NULL); }

    /// Sets the source node
    void setSourceNode(Node* node) { sourceNode_ = node; }

    /// Sets the next edge in face
    void setNextEdgeInFace(Edge* edge) { nextEdgeInFace_ = edge; }

    /// Sets the twin edge
    void setTwinEdge(Edge* edge) { twinEdge_ = edge; }

    /// Sets the edge as a leading edge
    void setAsLeadingEdge(bool val=true) { flags_.isLeadingEdge_ = val; }

    /// Checks if an edge is a leading edge
    bool isLeadingEdge() const { return flags_.isLeadingEdge_; }

    /// Sets the edge as a constrained edge
      void setConstrained(bool val=true) { flags_.isConstrained_ = val;
      if (twinEdge_) twinEdge_->flags_.isConstrained_ = val; }

    /// Checks if an edge is constrained
      bool isConstrained() const { return flags_.isConstrained_; }

    /// Returns the twin edge
    Edge* getTwinEdge() const { return twinEdge_; };

    /// Returns the next edge in face
    Edge* getNextEdgeInFace() const { return nextEdgeInFace_; }

    /// Retuns the source node
    Node* getSourceNode() { return sourceNode_.getPtr(); }

    /// Returns the target node
    Node* getTargetNode() { return getNextEdgeInFace()->getSourceNode(); }

  }; // End of class Edge


  //------------------------------------------------------------------------------------------------
  class Dart; // Forward declaration (class in this namespace)


  //------------------------------------------------------------------------------------------------
  // Triangulation class in the half-edge data structure
  //------------------------------------------------------------------------------------------------

  /** \class Triangulation
  *   \brief \b %Triangulation class for the half-edge data structure with adaption to TTL.
  */

  class Triangulation {

  protected:
    list<Edge*> leadingEdges_; // one half-edge for each arc
    void addLeadingEdge(Edge* edge) { edge->setAsLeadingEdge(); leadingEdges_.push_front(edge); }
    bool removeLeadingEdgeFromList(Edge* leadingEdge);
    void cleanAll();
    
  public:
    /// Default constructor
    Triangulation() {}
    
    /// Copy constructor
    Triangulation(const Triangulation& tr) { 
	std::cout << "Triangulation: Copy constructor not present - EXIT."; 
      exit(-1);
    }

    /// Destructor
    ~Triangulation() { cleanAll(); }
    
    /// Creates a Delaunay triangulation from a set of points
    void createDelaunay(std::vector<Node*>::iterator first,
                        std::vector<Node*>::iterator last);

    /// Creates an initial Delaunay triangulation from two enclosing triangles
    //  When using rectangular boundary - loop through all points and expand.
    //  (Called from createDelaunay(...) when starting)
    Edge* initTwoEnclosingTriangles(std::vector<Node*>::iterator first,
                                    std::vector<Node*>::iterator last);


    // These two functions are required by TTL for Delaunay triangulation
    
    /// Swaps the edge associated with diagonal
    void swapEdge(Edge& diagonal);

    /// Splits the triangle associated with edge into three new triangles joining at point 
    Edge* splitTriangle(Edge& edge, Node& point);


    // Functions required by TTL for removing nodes in a Delaunay triangulation
    
    /// Removes the boundary triangle associated with edge
    void removeTriangle(Edge& edge); // boundary triangle required    

    /// The reverse operation of removeTriangle
    void reverse_splitTriangle(Edge& edge);


    /// Creates an arbitrary CCW dart
    Dart createDart();

    /// Returns a list of "triangles" (one leading half-edge for each triangle)
    const list<Edge*>& getLeadingEdges() const { return leadingEdges_; }

    /// Returns the number of triangles
      int noTriangles() const { return (int)leadingEdges_.size(); }
    
    /// Returns a list of half-edges (one half-edge for each arc)
    list<Edge*>* getEdges(bool skip_boundary_edges = false) const;

#ifdef TTL_USE_NODE_FLAG
    /// Sets flag in all the nodes  
    void flagNodes(bool flag) const;

    /// Returns a list of nodes. This function requires TTL_USE_NODE_FLAG to be defined. \see Node.
    list<Node*>* getNodes() const;
#endif

    /// Swaps edges until the triangulation is Delaunay (constrained edges are not swapped)
    void optimizeDelaunay();

    /// Checks if the triangulation is Delaunay
    bool checkDelaunay() const;    

    /// Returns an arbitrary interior node (as the source node of the returned edge)
    Edge* getInteriorNode() const;

    /// Returns an arbitrary boundary edge
    Edge* getBoundaryEdge() const;

    /// Print edges for plotting with, e.g., gnuplot
      void printEdges(std::ofstream& os) const;

  }; // End of class Triangulation


}; // End of hed namespace

#endif
