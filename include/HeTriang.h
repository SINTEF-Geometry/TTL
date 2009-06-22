//===========================================================================
//                                                                           
// File:
//                                                                           
// Created: March 1 2001
//                                                                           
// Author: Øyvind Hjelle <oyvind.hjelle@sintef.no>,
//                                                                           
// Revision: $Id: HeTriang.h,v 1.31 2004-01-21 12:23:08 bsp Exp $
//                                                                           
// Description:
//                                                                           
//===========================================================================
// Copyright (c) 2000 SINTEF Applied Mathematics
//===========================================================================
#ifndef _HE_TRIANG_H_
#define _HE_TRIANG_H_

#include <list>
#include <map>
#include <vector>
#include <iostream>
#include <boost/smart_ptr.hpp>

#include "ttl.h"
#include "ttl_util.h"

#include "HeEdge.h"

// The half-edge data structure

namespace hed
{
    /// \b Triangulation class for the half-edge data structure with adaption to TTL.
    template <class NodeTraits>
    class Triangulation
    {
    public:
	typedef typename NodeTraits::NodeType NodeType;
	typedef HalfEdge<NodeType> Edge;
	typedef Dart<NodeTraits> Dart;

	Triangulation() {}
	/// The copy constructor makes new edges, but not new nodes.
	/// The nodes are shared between the original and the copy.
	Triangulation(const Triangulation& tr);
	~Triangulation() {cleanAll();}

	/// As with the copy constructor, nodes are shared.
	Triangulation& operator= (const Triangulation& tr);

	void swap(Triangulation& other);

	void read(std::istream& is);
	void write(std::ostream& os) const;
    
	/// Create Delaunay triangulation from a set of points
	void createDelaunay(typename std::vector< boost::shared_ptr<NodeType> >::iterator first,
			    typename std::vector< boost::shared_ptr<NodeType> >::iterator last);

	/// When using rectangular boundary - loop through all points and expand.
	/// (Called from createDelaunay(...) when starting)
	Edge* initTwoEnclosingTriangles(typename std::vector< boost::shared_ptr<NodeType> >::iterator first,
					typename std::vector< boost::shared_ptr<NodeType> >::iterator last);

	// These two functions are required by TTL for Delaunay triangulation
	void swapEdge(Edge& diagonal);
	Edge* splitTriangle(Edge& edge, NodeType& point)
	{
   
	// Add a node by just splitting a triangle into three triangles
	// Assumes the half edge is located in the triangle
	// Returns a half edge with source node as the new node
    
	// e#_n are new edges
	// e# are existing edges
	// e#_n and e##_n are new twin edges
	// e##_n are edges incident to the new node

	/// Add the node to the structure
	boost::shared_ptr<NodeType> new_node(new NodeType(point));

	//	NodeType* new_node = &point;
	boost::shared_ptr<NodeType> n1 = edge.getSourceNode();
	Edge* e1 = &edge;
  
	Edge* e2 = edge.getNextEdgeInFace();
	boost::shared_ptr<NodeType> n2 = e2->getSourceNode();
  
	Edge* e3 = e2->getNextEdgeInFace();
	boost::shared_ptr<NodeType> n3 = e3->getSourceNode();
  
	Edge* e1_n  = new Edge;
	Edge* e11_n = new Edge;
	Edge* e2_n  = new Edge;
	Edge* e22_n = new Edge;
	Edge* e3_n  = new Edge;
	Edge* e33_n = new Edge;
  
	e1_n->setSourceNode(n1);
	e11_n->setSourceNode(new_node);
	e2_n->setSourceNode(n2);
	e22_n->setSourceNode(new_node);
	e3_n->setSourceNode(n3);
	e33_n->setSourceNode(new_node);
  
	e1_n->setTwinEdge(e11_n);
	e11_n->setTwinEdge(e1_n);
	e2_n->setTwinEdge(e22_n);
	e22_n->setTwinEdge(e2_n);
	e3_n->setTwinEdge(e33_n);
	e33_n->setTwinEdge(e3_n);
  
	e1_n->setNextEdgeInFace(e33_n);
	e2_n->setNextEdgeInFace(e11_n);
	e3_n->setNextEdgeInFace(e22_n);
  
	e11_n->setNextEdgeInFace(e1);
	e22_n->setNextEdgeInFace(e2);
	e33_n->setNextEdgeInFace(e3);
  
  
	// and update old's next edge
	e1->setNextEdgeInFace(e2_n);
	e2->setNextEdgeInFace(e3_n);
	e3->setNextEdgeInFace(e1_n);
  
	// add the three new leading edges, 
	// Must remove the old leading edge from the list.
	// Use the field telling if an edge is a leading edge
	// NOTE: Must search in the list!!!
  
  
	Edge* leadingEdge;
	if (e1->isLeadingEdge())
	    leadingEdge = e1;
	else if (e2->isLeadingEdge())
	    leadingEdge = e2;
	else if(e3->isLeadingEdge())
	    leadingEdge = e3;
	else
	    return NULL;
  
	removeLeadingEdgeFromList(leadingEdge);
  
  
	addLeadingEdge(e1_n);
	addLeadingEdge(e2_n);
	addLeadingEdge(e3_n);
  
	// Return a half edge incident to the new node (with the new node as source node)

	return e11_n;
    }

	// Functions required by TTL for removing nodes in a Delaunay triangulation
	/// @@ Seems to me that any triangle may be removed, but that the remaining
	/// triangulation must be connected (not counting connection in nodes)...?
	void removeTriangle(Edge& edge); // boundary triangle required    
	void reverse_splitTriangle(Edge& edge);
    
	/// Create an arbitrary CCW dart
	Dart createDart();
	/// Create an arbitrary CCW dart, that has no knowledge of this triangulation
	Dart createFreeDart() const;

	/// Get a list of "triangles" (one leading half-edge per triangle)
	const std::list<Edge*>& getLeadingEdges() const {return leadingEdges_;}

	/// Number of triangles
	int noTriangles() const {return leadingEdges_.size();}
    
	/// One half-edge for each arc
	void getEdges(std::list<Edge*>& edges, 
				   bool skip_boundary_edges = false) const;

	void getNodes(std::list<Edge*>& nodes) const;
	//	std::list<boost::shared_ptr<NodeType> >* getNodes() const;

	/// Swap edges until the triangulation is Delaunay
	void optimizeDelaunay();

	/// Check if this triangulation is Delaunay
	bool checkDelaunay();    

	/// Get an arbitrary interior node (as the source node of the returned edge)
	Edge* getInteriorNode() const;
    
	/// Get an arbitrary boundary edge
	Edge* getBoundaryEdge() const;

	/// Return all boundary edges.
	std::vector<Edge*> getBoundaryEdges() const;

	/// Print edges for plotting with, e.g., gnuplot
	void printEdges(std::ostream& os, bool space_pts = true) const;

	// Direct manipulation of the topology
	void addLeadingEdge(Edge* edge) {edge->setAsLeadingEdge(); leadingEdges_.push_front(edge);}
	bool removeLeadingEdgeFromList(Edge* leadingEdge);
	void cleanAll();


    private:
	std::list<Edge*> leadingEdges_; // one half-edge for each arc

	typedef boost::shared_ptr<NodeType> SmartNodePointer;
	typedef std::map<SmartNodePointer, int> NodeMap;
	int map_vertices(NodeMap& nodeMap, std::vector<Edge*>& edgeVector) const;
	void flagNodes(bool flag) const;

    };
}; //end of half_edge namespace

// Include all the member functions
#include "HeTriang_templates.h"


#endif
