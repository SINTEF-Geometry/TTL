
//===========================================================================
//                                                                           
// File:
//                                                                           
// Created: March 1 2001
//                                                                           
// Author: Oeyvind Hjelle <oyvind.hjelle@sintef.no>,
//         ............. <............ @sintef.no>
//                                                                           
// Revision: $Id: HeTriang_templates.h,v 1.25 2008-03-17 15:48:09 sbr Exp $
//                                                                           
// Description:
//                                                                           
//===========================================================================
// Copyright (c) 2000 SINTEF Applied Mathematics
//===========================================================================


#include "GoTools/ttl/HeTraits.h"
// #include "GoTools/utils/errormacros.h"
#include <sstream>
#include <iostream>

#define DEBUG_HE
#ifdef DEBUG_HE
static void errorAndExit(char* message) {
    std::cout << "\n!!! ERROR: "<< message << " !!!\n" << std::endl; exit(-1);}
#endif

namespace hed
{
    /// Exception class
    class FileFormatError{};
    /// Exception class
    class StructuralMeshError{};

#ifndef _MSC_VER
#define TNAME typename
#else
#define TNAME
#endif

#ifndef _MSC_VER
    namespace // anonymous
    {
#endif
	//---------------------------------------------------------
	template <class NodeType>
	inline HalfEdge<NodeType>* getLeadingEdgeInTriangle(HalfEdge<NodeType>* e) {
  
	    HalfEdge<NodeType>* edge = e;
  
	    // Code: 3EF (assumes triangle)
	    if (!edge->isLeadingEdge()) {
		edge = edge->getNextEdgeInFace();
		if (!edge->isLeadingEdge())
		    edge = edge->getNextEdgeInFace();
	    }
  
	    if (!edge->isLeadingEdge()) {
		return NULL;
	    }
  
	    return edge;
	}

	// -----------------------------------------------------
	template <class NodeType>
	inline void printEdge(const Dart<NodeType>& dart, std::ostream& ofile) {
  
	    Dart<NodeType> d0 = dart;
	    d0.alpha0();
  
	    ofile << dart.x() << " " << dart.y() << std::endl;
	    ofile << d0.x() << " " << d0.y() << std::endl;
	}
	// -----------------------------------------------------
	template <class NodeType>
	inline HalfEdge<NodeType>* getBoundaryEdgeInTriangle(HalfEdge<NodeType>* edge) {
  
	    if (!edge->getTwinEdge())
		return edge;  
	    edge = edge->getNextEdgeInFace();
	    if (!edge->getTwinEdge())
		return edge;
	    edge = edge->getNextEdgeInFace();
	    if (!edge->getTwinEdge())
		return edge;
  
	    return NULL;
	}


#ifndef _MSC_VER
    } // anonymous namespace
#endif

    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline Triangulation<NodeTraits>::Triangulation(const Triangulation<NodeTraits>&  other)
    {
	// Making new edges.
	typename std::list<EdgeNode*> new_leading_edges;
	typename std::list<EdgeNode*>::const_iterator it;
	const std::list<EdgeNode*>& le = other.leadingEdges_;
	// node2edge will contain one entry per edge.
	std::multimap<SmartNodePointer, EdgeNode*> node2edge;
	EdgeNode* new_edges_in_tri[3];
	for (it = le.begin(); it != le.end(); ++it) {
	    EdgeNode* old = *it;
	    int i;
	    for (i = 0; i < 3; ++i) {
		new_edges_in_tri[i] = new EdgeNode;
		new_edges_in_tri[i]->setSourceNode(old->getSourceNode());
		old = old->getNextEdgeInFace();
		node2edge.insert(std::make_pair(new_edges_in_tri[i]->getSourceNode(),
						new_edges_in_tri[i]));
	    }
	    for (i = 0; i < 3; ++i) {
		new_edges_in_tri[i]->setNextEdgeInFace(new_edges_in_tri[(i+1)%3]);
	    }
	    new_leading_edges.push_front(new_edges_in_tri[0]);
	}

	// Removing the contents of this triangulation
	cleanAll();

	// Swapping in the new ones
	leadingEdges_.swap(new_leading_edges);

	// For every node
	typedef typename std::multimap<SmartNodePointer, EdgeNode*>::iterator NIter;
	NIter mit = node2edge.begin();
	while(mit != node2edge.end()) {
	    // Pick the source and target node of the current edge
	    SmartNodePointer snode = mit->second->getSourceNode();
	    SmartNodePointer tnode = mit->second->getTargetNode();
	    // Find all edges with tnode as source node
	    std::pair<NIter, NIter> er = node2edge.equal_range(tnode);
	    // Find one that has snode as target node
	    for (; er.first != er.second; ++er.first) {
		if (er.first->second->getTargetNode() == snode) {
		    break;
		}
	    }
	    // If one was found...
	    if (er.first != er.second) {
		// Connect it with the current edge
		mit->second->setTwinEdge(er.first->second);
		er.first->second->setTwinEdge(mit->second);
		// Remove both edges from the map. The iterator
		// it will be invalid, so we have to use a temporary
		// iterator to hold the next entry in the map.
		NIter tmp = mit;
		++tmp;
		if (tmp == er.first) {
		    ++tmp;
		}
		node2edge.erase(mit);
		node2edge.erase(er.first);
		mit = tmp;
	    } else {
		++mit;
	    }
	}
    }

    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline Triangulation<NodeTraits>&
    Triangulation<NodeTraits>::operator=(const Triangulation<NodeTraits>&  other)
    {
	Triangulation<NodeTraits> temp(other);
	swap(temp);
	return *this;
    }


    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::swap(Triangulation<NodeTraits>& other)
    {
	leadingEdges_.swap(other.leadingEdges_);
    }

    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline int Triangulation<NodeTraits>::map_vertices(Triangulation<NodeTraits>::NodeMap& nodeMap, 
						     std::vector<EdgeNode*>& edgeVector) const
    {
	// assuring a clean depature
	nodeMap.clear();
	edgeVector.clear();

	const std::list<EdgeNode*>& triangles = getLeadingEdges();
	EdgeNode* tempEdge;
	EdgeNode* iterEdge;
	SmartNodePointer tempNode;
	typename std::list<EdgeNode*>::const_iterator ti;
	typename NodeMap::iterator ni;
	int node_number = 0;

	for (ti = triangles.begin(); ti != triangles.end(); ++ti) {
	    tempEdge = *ti;
	    iterEdge = *ti;
	
	    do {
		tempNode = iterEdge->getSourceNode();
		ni = nodeMap.find(tempNode);
		if (ni == nodeMap.end()) {
		    edgeVector.push_back(iterEdge);
		    nodeMap[tempNode] = node_number++;
		}
		iterEdge = iterEdge->getNextEdgeInFace();

	    } while (iterEdge != tempEdge);

        }
	return edgeVector.size();
    }

    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::read(std::istream& is)
    {
	// The format of incoming data is supposed to be:
	// * number_of_vertices (integer)
	// * number of polygons
	// * list of 3D vertex coordinates (floating points)
	// * list of (n+1)-tuples of integers specifying the corners of the n-gons
	//   in counterclockwise order, each tuple starting with the integer n.
 
    // removing current triangulation
//         ASSERT(is.good());
	if (!is.good()) {
	    throw std::exception();
	}
	cleanAll();
   
	// determining number of vertices and polygons
	int V, T;
	is >> V >> T;
	if ( !(V >= 3 &&  T>=1) ) 
	    throw FileFormatError();
    
    // creation of point vector
	std::vector< SmartNodePointer > nodes;
	double x, y, z;
	int i,j;
	for (i = 0; i < V; ++i) { 
	    SmartNodePointer np(new NodeType);
	    NodeTraits::readNode(*np, is);
	    if (!is.good()) {
		std::cerr << "File has gone bad, maybe premature eof? Current node is: " << i << std::endl;
		throw std::exception();
	    }
	    nodes.push_back(np);
//  	    is >> x >> y >> z;
//  	    if (!is.good()) {
//  		THROW("File has gone bad, maybe premature eof?");
//  	    }
//  	    nodes.push_back(SmartNodePointer(new NodeType(x, y, z)));
	}

	// construction of faces
	std::vector<EdgeNode*> all_edges;
	std::vector<EdgeNode*> poly_edges;
	for (i = 0; i < T; ++i) {
	    int n;
	    is >> n;
	    if (n != 3) {
		throw FileFormatError();
	    }
	
	    poly_edges.clear();
	    for (j = 0; j < n; ++j) {
		poly_edges.push_back(new(EdgeNode));
		int node_ind;
		is >> node_ind;
		if (node_ind >= nodes.size()) 
		    throw FileFormatError();
		poly_edges.back()->setSourceNode(nodes[node_ind]);
	    }

	    poly_edges[0]->setAsLeadingEdge();
	    addLeadingEdge(poly_edges[0]);

	    for (j = 0; j < n; ++j) {
		poly_edges[j]->setNextEdgeInFace(poly_edges[(j + 1) % n]);
	    }
	    all_edges.insert(all_edges.end(), poly_edges.begin(), poly_edges.end());
	}

	if (!is.good()) {
	    std::cerr << "File has gone bad, maybe premature eof?" << std::endl;
	    throw std::exception();
	}

	typedef typename std::multimap<SmartNodePointer, int> NodeMultiMap;
	typedef typename NodeMultiMap::const_iterator NMIter;
	NodeMultiMap nodes_to_edges;
	for (i = 0; i < all_edges.size(); ++i) {
	    nodes_to_edges.insert(make_pair(all_edges[i]->getSourceNode(), i));
	    nodes_to_edges.insert(make_pair(all_edges[i]->getTargetNode(), i));
	}

	// linking of triangles
	std::vector<int> incident_edges;
	for (i = 0; i < all_edges.size(); ++i) {
	    //	cout << "Edge " << i << '/' << all_edges.size() << endl;
	    incident_edges.clear();
	    std::pair<NMIter, NMIter> range = nodes_to_edges.equal_range(all_edges[i]->getSourceNode());
	    for (; range.first != range.second; ++range.first) {
		if (range.first->second != i) {
		    incident_edges.push_back(range.first->second);
		}
	    }
	    range = nodes_to_edges.equal_range(all_edges[i]->getTargetNode());
	    for (; range.first != range.second; ++range.first) {
		if (range.first->second != i) {
		    incident_edges.push_back(range.first->second);
		}
	    }

	    for (j = 0; j < incident_edges.size(); ++j) {
		int edgenum = incident_edges[j];
		if (all_edges[i]->getSourceNode() == all_edges[edgenum]->getSourceNode() &&
		    all_edges[i]->getTargetNode() == all_edges[edgenum]->getTargetNode()) {
		    std::cerr << " Error in user defined mesh\n";
		    throw StructuralMeshError();
		}
		if (all_edges[i]->getSourceNode() == all_edges[edgenum]->getTargetNode() &&
		    all_edges[i]->getTargetNode() == all_edges[edgenum]->getSourceNode()) {
		    all_edges[i]->setTwinEdge(all_edges[edgenum]);
		    all_edges[edgenum]->setTwinEdge(all_edges[i]);
		}
	    }
	}
    }

    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::write(std::ostream& os) const
    {  
	// Write the mesh to an output stream in the minimal triangle based
	// data structure

	// mapping out nodes and triangles
	NodeMap nodesToIndexes;

	std::vector<EdgeNode*> indexesToNodes;
	std::list<EdgeNode*> triangles = getLeadingEdges();
	typename std::list<EdgeNode*>::const_iterator ti;
	std::vector<int>::const_iterator vi;
	EdgeNode* temp_edge;
	EdgeNode* first_edge;
	std::vector<int> face_vertices;

    // indexing nodes
	int nodeNumber =  map_vertices(nodesToIndexes, indexesToNodes); 
    
	// writing size information
	os << nodeNumber << " " << triangles.size() << std::endl << std::endl;

	// writing vertex coordinates
	for (int i = 0; i < nodeNumber; ++i) {
//  	    os << indexesToNodes[i]->getSourceNode()->x() << " " <<
//  		indexesToNodes[i]->getSourceNode()->y() << " " <<
//  		indexesToNodes[i]->getSourceNode()->z() << endl;
	    NodeTraits::writeNode(*(indexesToNodes[i]->getSourceNode()), os);
	}
	os << std::endl;
    
	// writing triangles
	for (ti = triangles.begin(); ti != triangles.end(); ++ti) {
	    face_vertices.clear();
	    first_edge = temp_edge = *ti;
	    do {
		face_vertices.push_back(nodesToIndexes[temp_edge->getSourceNode()]);
		temp_edge = temp_edge->getNextEdgeInFace();
	    } while (temp_edge != first_edge);
	    os << face_vertices.size() << ' ';
	    for (vi = face_vertices.begin(); vi != face_vertices.end(); ++vi) {
		os << *vi << ' ';
	    }
	    os << std::endl;
	}
    }

    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline HalfEdge<TNAME NodeTraits::NodeType>*
    Triangulation<NodeTraits>::initTwoEnclosingTriangles(typename std::vector< boost::shared_ptr<TNAME NodeTraits::NodeType> >::iterator first,
						       typename std::vector< boost::shared_ptr<TNAME NodeTraits::NodeType> >::iterator last) {

	
	boost::shared_ptr<NodeType> n1(new NodeType);
	boost::shared_ptr<NodeType> n2(new NodeType);
	boost::shared_ptr<NodeType> n3(new NodeType);
	boost::shared_ptr<NodeType> n4(new NodeType);
	NodeTraits::boundingCorners2d(first, last, *n1, *n2, *n3, *n4);
  
	// diagonal
	EdgeNode* e1d = new EdgeNode; // lower
	EdgeNode* e2d = new EdgeNode; // upper, the twin edge
  
	// lower triangle
	EdgeNode* e11 = new EdgeNode;
	EdgeNode* e12 = new EdgeNode;
  
	// upper triangle
	EdgeNode* e21 = new EdgeNode; // upper upper
	EdgeNode* e22 = new EdgeNode;
  
	// lower triangle
	e1d->setSourceNode(n3);
	e1d->setNextEdgeInFace(e11);
	e1d->setTwinEdge(e2d);
	e1d->setAsLeadingEdge();
	addLeadingEdge(e1d);
  
	e11->setSourceNode(n1);
	e11->setNextEdgeInFace(e12);
  
	e12->setSourceNode(n2);
	e12->setNextEdgeInFace(e1d);
  
	// upper triangle
	e2d->setSourceNode(n1);
	e2d->setNextEdgeInFace(e21);
	e2d->setTwinEdge(e1d);
	e2d->setAsLeadingEdge();
	addLeadingEdge(e2d);
  
	e21->setSourceNode(n3);
	e21->setNextEdgeInFace(e22);
  
	e22->setSourceNode(n4);
	e22->setNextEdgeInFace(e2d);
  
	return e11;
    }

    //------------------------------------------------------------------------------
    //------------------------------------------------------------------------------
    template <class NodeTraits>  
    inline void Triangulation<NodeTraits>::createDelaunay(typename std::vector< boost::shared_ptr<NodeType> >::iterator first,
							typename std::vector< boost::shared_ptr<NodeType> >::iterator last) {
  
	cleanAll();
    
	EdgeNode* bedge = initTwoEnclosingTriangles(first, last);
	DartNode dc(bedge, this);
  
	DartNode d_iter = dc;
  
	typedef HeTraits<NodeTraits> MyTraits;
	typename std::vector< boost::shared_ptr<NodeType> >::iterator it;
	for (it = first; it != last; ++it) {
	    bool status = ttl::insertNode<MyTraits>(d_iter, **it);
#ifdef TTL_DEBUG
	    if (!status) {
		std::cout << "It seems we failed inserting node into structure." << std::endl;
		throw;
	    }
#endif // TTL_DEBUG
	}
  
	// In general (e.g. for the triangle based data structure), the initial dart
	// may have been changed.
	// It is the users responsibility to get a valid boundary dart here.
	// The half-edge data structure preserves the initial dart.
	// (A dart at the boundary can also be found by trying to locate a
	// triangle "outside" the triangulation.)

	// Assumes rectangular domain
	ttl::removeRectangularBoundary<MyTraits>(dc);
    }


    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::removeTriangle(EdgeNode& edge) {
  
	EdgeNode* e1 = getLeadingEdgeInTriangle(&edge);

#ifdef DEBUG_HE
	if (e1 == NULL) {
// 	    errorAndExit("Triangulation<NodeTraits>::removeTriangle: could not find leading edge");
// 	    THROW("Triangulation<NodeTraits>::removeTriangle: could not find leading edge");
	    cout << "Triangulation<NodeTraits>::removeTriangle: could not find leading edge" << endl;
	    throw std::exception();
	}
#endif
  
	removeLeadingEdgeFromList(e1);
	// std::cout << "No leading edges = " << leadingEdges_.size() << std::endl;  
	// Remove the triangle
	EdgeNode* e2 = e1->getNextEdgeInFace();
	EdgeNode* e3 = e2->getNextEdgeInFace();
  
	if (e1->getTwinEdge())
	    e1->getTwinEdge()->setTwinEdge(NULL);
	if (e2->getTwinEdge())
	    e2->getTwinEdge()->setTwinEdge(NULL);
	if (e3->getTwinEdge())
	    e3->getTwinEdge()->setTwinEdge(NULL);
  
	delete e1;
	delete e2;
	delete e3;
    }


    /* OK, but not needed
       //------------------------------------------------------------------------------
       template <class NodeType>
       void Triangulation<NodeTraits>::removeBoundaryNode(Dart<NodeType>& dart) {
  
       // Assumes that swapping has been done in the Delaunay case
       // No dart given as output
       // Assumes that the dart represents a boundary edge
       // Note, convex boundary is not necessarily preserved.
  
       // Removing boundary node but NOT swap Delaunay
  
       Dart d_iter = dart;
       Dart dnext = dart;
       bool bend = false;
       while (bend == false) {
       dnext.alpha1().alpha2();
       if (ttl::isBoundaryEdge(dnext))
       bend = true; // Stop when boundary
    
       removeTriangle(*d_iter.getEdge());
       d_iter = dnext;
       }
       }
    */

    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::reverse_splitTriangle(EdgeNode& edge) {
  
	// Reverse operation of splitTriangle
  
	EdgeNode* e1 = edge.getNextEdgeInFace();
	EdgeNode* le = getLeadingEdgeInTriangle(e1);
#ifdef DEBUG_HE
	if (le == NULL) {
// 	    errorAndExit("Triangulation<NodeTraits>::removeTriangle: could not find leading edge");
// 	    THROW("Triangulation<NodeTraits>::removeTriangle: could not find leading edge");
	    cout << "Triangulation<NodeTraits>::removeTriangle: could not find leading edge" << endl;
	    throw std::exception();
	}
#endif
	removeLeadingEdgeFromList(le);
  
	EdgeNode* e2 = e1->getNextEdgeInFace()->getTwinEdge()->getNextEdgeInFace();
	le = getLeadingEdgeInTriangle(e2);
#ifdef DEBUG_HE
	if (le == NULL) {
// 	    errorAndExit("Triangulation<NodeTraits>::removeTriangle: could not find leading edge");
// 	    THROW("Triangulation<NodeTraits>::removeTriangle: could not find leading edge");
	    cout << "Triangulation<NodeTraits>::removeTriangle: could not find leading edge" << endl;
	    throw std::exception();
	}
#endif
	removeLeadingEdgeFromList(le);
  
  
	EdgeNode* e3= edge.getTwinEdge()->getNextEdgeInFace()->getNextEdgeInFace();
	le = getLeadingEdgeInTriangle(e3);
#ifdef DEBUG_HE
	if (le == NULL) {
// 	    errorAndExit("Triangulation<NodeTraits>::removeTriangle: could not find leading edge");
// 	    THROW("Triangulation<NodeTraits>::removeTriangle: could not find leading edge");
	    cout << "Triangulation<NodeTraits>::removeTriangle: could not find leading edge" << endl;
	    throw std::exception();
	}
#endif
	removeLeadingEdgeFromList(le);
  
	// The three triangles at the node have now been removed
	// from the triangulation, but the arcs have not been deleted.
	// Next delete the 6 half edges radiating from the node
	// The node is maintained by handle and need not be deleted explicitly
  
	EdgeNode* estar = &edge;
	EdgeNode* enext = estar->getTwinEdge()->getNextEdgeInFace();
	delete estar->getTwinEdge();
	delete estar;  
  
	estar = enext;
	enext = estar->getTwinEdge()->getNextEdgeInFace();
	delete estar->getTwinEdge();
	delete estar;
  
	delete enext->getTwinEdge();
	delete enext;
  
	// Create the new triangle
	e1->setNextEdgeInFace(e2);
	e2->setNextEdgeInFace(e3);
	e3->setNextEdgeInFace(e1);
	addLeadingEdge(e1);
    }


    //------------------------------------------------------------------------------
    // This is a "template" for iterating the boundary    
    /*
      static void iterateBoundary(const Dart<NodeType>& dart) {
      std::cout << "Iterate boundary 2" << std::endl;
      // input is a dart at the boundary

      Dart dart_iter = dart;
      do {
      if (ttl::isBoundaryEdge(dart_iter))
      dart_iter.alpha0().alpha1();
      else
      dart_iter.alpha2().alpha1();
  
      } while(dart_iter != dart);
      }
    */


    // -----------------------------------
    template <class NodeTraits>
    inline Dart<NodeTraits> Triangulation<NodeTraits>::createDart() { 
  
	// Return an arbitrary CCW dart
	return DartNode(*leadingEdges_.begin(), this);
    }
    template <class NodeTraits>
    inline Dart<NodeTraits> Triangulation<NodeTraits>::createFreeDart() const { 
  
	// Return an arbitrary CCW dart, with no knowledge of its triangulation
	return DartNode(*leadingEdges_.begin(), 0);
    }

    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline bool Triangulation<NodeTraits>::removeLeadingEdgeFromList(EdgeNode* leadingEdge) {
  
	// Remove the edge from the list of leading edges,
	// but don't delete it.
	// Also set flag for leading edge to false.
	// Must search from start of list. Since edges are added to the
	// start of the list during triangulation, this operation will
	// normally be fast (when used in the triangulation algorithm)
	typename std::list<EdgeNode*>::iterator it;
	for (it = leadingEdges_.begin(); it != leadingEdges_.end(); ++it) {
    
	    EdgeNode* edge = *it;
	    if (edge == leadingEdge) {
      
		edge->setAsLeadingEdge(false);
		leadingEdges_.erase(it);
      
		break;
	    }
	}
  
	if (it == leadingEdges_.end())
	    return false;
  
	return true;
    }


    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::cleanAll() {
  
	typename std::list<EdgeNode*>::const_iterator it;  
	for (it = leadingEdges_.begin(); it != leadingEdges_.end(); ++it) {
	    EdgeNode* e1 = *it;
	    EdgeNode* e2 = e1->getNextEdgeInFace();
	    EdgeNode* e3 = e2->getNextEdgeInFace();
    
	    delete e1;
	    delete e2;
	    delete e3;
	}
  
	leadingEdges_.clear();
    }


    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::flagNodes(bool flag) const {
	
	typename std::list<EdgeNode*>::const_iterator it;
	for (it = leadingEdges_.begin(); it != leadingEdges_.end(); ++it) {
	    EdgeNode* edge = *it;
	    
	    for (int i = 0; i < 3; ++i) {
		edge->getSourceNode()->setFlag(flag);
		edge = edge->getNextEdgeInFace();
	    }
	}
    }


    
    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void
    Triangulation<NodeTraits>
    ::getNodes(std::list< HalfEdge<TNAME NodeTraits::NodeType>* >& nodeList) const { 
	flagNodes(false);
	nodeList.clear();
	typename std::list<EdgeNode*>::const_iterator it;
	for (it = leadingEdges_.begin(); it != leadingEdges_.end(); ++it) {
	    EdgeNode* edge = *it;
	    for (int i = 0; i < 3; ++i) {
		SmartNodePointer node = edge->getSourceNode();
		if (node->getFlag() == false) {
		    nodeList.push_back(edge);
		    node->setFlag(true);
		}
		edge = edge->getNextEdgeInFace();
	    }
	}
    }


    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void
    Triangulation<NodeTraits>
    ::getEdges(std::list< HalfEdge<TNAME NodeTraits::NodeType>* >& elist,
	       bool skip_boundary_edges) const {
  
	// collect all arcs (one half edge for each arc)
	// (boundary edges are also collected).
  
	typename std::list<EdgeNode*>::const_iterator it;
	elist.clear();
	for (it = leadingEdges_.begin(); it != leadingEdges_.end(); ++it) {
	    EdgeNode* edge = *it;
	    for (int i = 0; i < 3; ++i) {
		EdgeNode* twinedge = edge->getTwinEdge();
		// only one of the half-edges
      
		if ( (twinedge == NULL && !skip_boundary_edges) ||
		     (twinedge != NULL && (edge - twinedge > 0)) )
		    elist.push_front(edge);
      
		edge = edge->getNextEdgeInFace();
	    }
	}
    }
    

  /*
    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline HalfEdge<TNAME NodeTraits::NodeType>*
    Triangulation<NodeTraits>::splitTriangle(HalfEdge<TNAME NodeTraits::NodeType>& edge,
					     TNAME NodeTraits::NodeType& point) {

   
	// Add a node by just splitting a triangle into three triangles
	// Assumes the half edge is located in the triangle
	// Returns a half edge with source node as the new node
    
	// e#_n are new edges
	// e# are existing edges
	// e#_n and e##_n are new twin edges
	// e##_n are edges incident to the new node

	// Add the node to the structure
	boost::shared_ptr<NodeType> new_node(new NodeType(point));

	//	NodeType* new_node = &point;
	boost::shared_ptr<NodeType> n1 = edge.getSourceNode();
	EdgeNode* e1 = &edge;
  
	EdgeNode* e2 = edge.getNextEdgeInFace();
	boost::shared_ptr<NodeType> n2 = e2->getSourceNode();
  
	EdgeNode* e3 = e2->getNextEdgeInFace();
	boost::shared_ptr<NodeType> n3 = e3->getSourceNode();
  
	EdgeNode* e1_n  = new EdgeNode;
	EdgeNode* e11_n = new EdgeNode;
	EdgeNode* e2_n  = new EdgeNode;
	EdgeNode* e22_n = new EdgeNode;
	EdgeNode* e3_n  = new EdgeNode;
	EdgeNode* e33_n = new EdgeNode;
  
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
  
  
	EdgeNode* leadingEdge;
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
*/  
    
    //------------------------------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::swapEdge(EdgeNode& diagonal) {
  
	// Note that diagonal is both input and output and it is always
	// kept in counterclockwise direction (this is not required by all 
	// finctions in ttl:: now)
  
	// Swap by rotating counterclockwise
	// Use the same objects - no deletion or new objects
	EdgeNode* eL   = &diagonal;
	EdgeNode* eR   = eL->getTwinEdge();
	EdgeNode* eL_1 = eL->getNextEdgeInFace();
	EdgeNode* eL_2 = eL_1->getNextEdgeInFace();
	EdgeNode* eR_1 = eR->getNextEdgeInFace();
	EdgeNode* eR_2 = eR_1->getNextEdgeInFace();
  
	// avoid node to be dereferenced to zero and deleted
	boost::shared_ptr<NodeType> nR = eR_2->getSourceNode();
	boost::shared_ptr<NodeType> nL = eL_2->getSourceNode();
  
	eL->setSourceNode(nR);
	eR->setSourceNode(nL);
  
	// and now 6 1-sewings
	eL->setNextEdgeInFace(eL_2);
	eL_2->setNextEdgeInFace(eR_1);
	eR_1->setNextEdgeInFace(eL);
  
	eR->setNextEdgeInFace(eR_2);
	eR_2->setNextEdgeInFace(eL_1);
	eL_1->setNextEdgeInFace(eR);
  
	EdgeNode* leL = 0;
	if (eL->isLeadingEdge())
	    leL = eL;
	else if (eL_1->isLeadingEdge())
	    leL = eL_1;
	else if (eL_2->isLeadingEdge())
	    leL = eL_2;
  
	EdgeNode* leR = 0;
	if (eR->isLeadingEdge())
	    leR = eR;
	else if (eR_1->isLeadingEdge())
	    leR = eR_1;
	else if (eR_2->isLeadingEdge())
	    leR = eR_2;
  
	removeLeadingEdgeFromList(leL);
	removeLeadingEdgeFromList(leR);
	addLeadingEdge(eL);
	addLeadingEdge(eR);
    }

    //------------------------------------------
    template <class NodeTraits>
    inline bool Triangulation<NodeTraits>::checkDelaunay() {
  
	// ???? outputs !!!!
	// ofstream os("qweND.dat");
	const std::list<EdgeNode*>& leadingEdges = getLeadingEdges();
  
	typename std::list<EdgeNode*>::const_iterator it;
	bool ok = true;
	int noNotDelaunay = 0;
  
	for (it = leadingEdges.begin(); it != leadingEdges.end(); ++it) {
	    EdgeNode* edge = *it;
    
	    for (int i = 0; i < 3; ++i) {
		EdgeNode* twinedge = edge->getTwinEdge();
      
		// only one of the half-edges
		if (twinedge == NULL || (long int)edge > (long int)twinedge) {
		    DartNode dart(edge, this);
		    if (ttl::swapTestDelaunay< HeTraits<NodeTraits> >(dart)) {
			noNotDelaunay++;
          
			//printEdge(dart,os); os << "\n";
			ok = false;
			//std::cout << "............. not Delaunay .... " << std::endl;
		    }
		}
		edge = edge->getNextEdgeInFace();
	    }
	}
  
#ifdef DEBUG_HE
	std::cout << "!!! Triangulation is NOT Delaunay: " << noNotDelaunay << " edges\n" << std::endl;
#endif
  
	return ok;
    }

    // -----------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::optimizeDelaunay() {

	// This function is also present in ttl where it is implemented
	// generically.
	// The implementation below is tailored for the half-edge data structure,
	// and is thus more efficient

	// Collect all interior edges (one half edge for each arc)
	bool skip_boundary_edges = true;
	std::list<EdgeNode*>* elist = getEdges(skip_boundary_edges);
  
	// Assumes that elist has only one half-edge for each arc.
	bool cycling_check = true;
	bool optimal = false;
	typename std::list<EdgeNode*>::const_iterator it;
	while(!optimal) {
	    optimal = true;
	    for (it = elist->begin(); it != elist->end(); ++it) {
		EdgeNode* edge = *it;
      
		DartNode dart(edge, this);
		if (ttl::swapTestDelaunay< HeTraits<NodeTraits> >(dart, cycling_check)) {
		    optimal = false;
		    swapEdge(*edge);
		}
	    }
	}
	delete elist;
    }

    // -----------------------------------------------------
    template <class NodeTraits>
    inline HalfEdge<TNAME NodeTraits::NodeType>* Triangulation<NodeTraits>::getInteriorNode() const {
  
	const std::list<EdgeNode*>& leadingEdges = getLeadingEdges();
	typename std::list<EdgeNode*>::const_iterator it;
	for (it = leadingEdges.begin(); it != leadingEdges.end(); ++it) {
	    EdgeNode* edge = *it;
    
	    // multiple checks, but only until found
	    for (int i = 0; i < 3; ++i) {
		if (edge->getTwinEdge() != NULL) {
        
		    if (!ttl::isBoundaryNode(DartNode(edge)))
			return edge;
		}
		edge = edge->getNextEdgeInFace();
	    }
	}
	return NULL; // no boundary nodes
    }

    

    // -----------------------------------------------------
    template <class NodeTraits>
    inline HalfEdge<TNAME NodeTraits::NodeType>* Triangulation<NodeTraits>::getBoundaryEdge() const {

	// Get an arbitrary (CCW) boundary edge
	// If the triangulation is closed, NULL is returned

	const std::list<EdgeNode*>& leadingEdges = getLeadingEdges();
	typename std::list<EdgeNode*>::const_iterator it;
	EdgeNode* edge;
  
	for (it = leadingEdges.begin(); it != leadingEdges.end(); ++it) {
	    edge = getBoundaryEdgeInTriangle(*it);
    
	    if (edge)
		return edge;
	}
	return NULL;
    };

    // -----------------------------------------------------
    template <class NodeTraits>
    inline std::vector<HalfEdge<TNAME NodeTraits::NodeType>*>
      Triangulation<NodeTraits>::getBoundaryEdges() const {

	// Get all (CCW) boundary edges
	// If the triangulation is closed, empty vector is returned

	const std::list<EdgeNode*>& leadingEdges = getLeadingEdges();
	typename std::list<EdgeNode*>::const_iterator it;
	EdgeNode* edge;

	std::vector<EdgeNode*> bd_edges;
	for (it = leadingEdges.begin(); it != leadingEdges.end(); ++it) {
	    edge = getBoundaryEdgeInTriangle(*it);
    
	    if (edge) {
	        bd_edges.push_back(edge);
		for (int i = 0; i < 2; ++i) {
		  edge = edge->getNextEdgeInFace();
		  if (edge->getTwinEdge() == 0)
		    bd_edges.push_back(edge);
		}
	    }

	}
	return bd_edges;
    };
    

    // ----------------------------------------------------------
    template <class NodeTraits>
    inline void Triangulation<NodeTraits>::printEdges(std::ostream& os, bool space_pts) const {
      
	// Print source node and target node for each edge face by face,
	// but only one of the half-edges.
  
	const std::list<EdgeNode*>& leadingEdges = getLeadingEdges();
	typename std::list<EdgeNode*>::const_iterator it;
	for (it = leadingEdges.begin(); it != leadingEdges.end(); ++it) {
	    EdgeNode* edge = *it;
    
	    for (int i = 0; i < 3; ++i) {
		EdgeNode* twinedge = edge->getTwinEdge();
      
		// Print only one edge (the highest value of the pointer)
		if (twinedge == NULL || (long int)edge > (long int)twinedge) {
		    // Print source node and target node
		    boost::shared_ptr<NodeType> node = edge->getSourceNode();
		    if (space_pts)
		      os << node->x() << " " << node->y() << " " << node->z() << std::endl;
		    else
		      os << node->u() << " " << node->v() << std::endl;
		    node = edge->getTargetNode();
		    if (space_pts)
		      os << node->x() << " " << node->y() << " " << node->z() << std::endl;
		    else
		      os << node->u() << " " << node->v() << std::endl;
		    os << '\n'; // blank line
		}
		edge = edge->getNextEdgeInFace();
	    }
	}
    }


} // namespace hed

