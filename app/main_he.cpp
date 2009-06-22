//#define DEBUG_TTL_CONSTR
//#define DEBUG_TTL_CONSTR_PLOT

#include <fstream>
#include <iostream>
#include <algorithm>
using namespace std;

#include "ExampleNode.h"
#include "example_util.h"
#include "HeTriang.h"
#include "HeTraits.h"
#include "timeutils.h"

typedef boost::shared_ptr<ExampleNode> NodePtr;
typedef hed::Triangulation<ExampleNodeTraits> Triangulation;
typedef Triangulation::Edge Edge;
typedef Triangulation::Dart Dart;
typedef hed::HeTraits<ExampleNodeTraits> Traits;

// ----------------------------------------------------------
// Interpret two points as being coincident
inline bool eqPoints(NodePtr p1, NodePtr p2)
{
    double dx = p1->x() - p2->x();
    double dy = p1->y() - p2->y();
    double dist2 = dx*dx + dy*dy;
    const double eps = 1.0e-12;
    if (dist2 < eps)
	return true;
    else
	return false;
}

// Lexicographically compare two points (2D)
inline bool ltLexPoint(NodePtr p1, NodePtr p2)
{  
    return (p1->x() < p2->x()) || (p1->x() == p2->x() && p1->y() < p2->y());
};


// =========================================================================
// A main program using TTL and the half-edge data structure to create
// a Delaunay triangulation.
// The program also demonstrates how to use other function templates in TTL.
// =========================================================================

int main()
{  
    // ===============================================================
    // CREATE A DELAUNAY TRIANGULATION FROM RANDOM POINTS IN THE PLANE
    // ===============================================================
  
    // Create random test data.
    int no_of_nodes = 100;
    std::vector<NodePtr> nodes;
    double time = getCurrentTime();
    std::cout << "Press enter to start." << std::endl;
    char c;
    std::cin >> c;
    int seed = int(100000*(getCurrentTime() - time));
    createRandomData(no_of_nodes, nodes, seed);

  // Sort the nodes lexicographically in the plane.
  // This is recommended since the triangulation algorithm will run much faster.
  // (ltLexPoint is defined above)
    std::sort(nodes.begin(), nodes.end(), ltLexPoint);
  
    // Remove coincident points to avoid degenerate triangles. (eqPoints is defined above)
    std::vector<NodePtr>::iterator new_end = std::unique(nodes.begin(), nodes.end(), eqPoints);
  
    // Make the triangulation
    Triangulation triang;
    triang.createDelaunay(nodes.begin(), new_end);
  
  // <... Print triangulation; see end of file ...>
  
  // ========================================================
  // SOME EXAMPLES USING TTL (Functions in namespace ttl)
  // ========================================================
  
  // Insert a new node in the Delaunay triangulation.
  // We need an arbitrary CCW (counterclockwise) dart for TTL.
  // Make the dart from the first edge in the list of leading edges.
  // ( Could also use Triangulation::createDart() )
    const list<Edge*>& l_edges = triang.getLeadingEdges();
    Edge* edge = *l_edges.begin();
    Dart dart(edge, &triang);
    NodePtr point(new ExampleNode(0.3, 0.6, 0));
    ttl::insertNode<Traits>(dart, *point);
  
    // Locate a triangle in the triangulation containing the given point.
    // The given dart will be repositioned to that triangle while maintaining
    // its orientation (CCW or CW).
    // If the given point is outside the triangulation, the dart will be
    // positioned at a boundary edge.
    point->init(0.5, 0.5, 0);
    bool found = ttl::locateTriangle<Traits>(*point, dart);
    if (!found) {
	cout << "The given points is outside the triangulation" << endl;
	// (and the dart is positioned at a boundary edge)
	exit(-1);
    }
  
    // The degree (or valency) of a node V in a triangulation, is defined
    // as the number of edges incident with V.
    // Get the degree of the node associated with the dart.
    int degree = ttl::getDegreeOfNode(dart);
    cout << "Degree of node = " << degree << endl;

  // Check if the edge associated with the dart is at the boundary of the triangulation.
    if (ttl::isBoundaryEdge(dart))
	cout << "The edge is at the boundary" << endl;

  // Check if the node associated with the dart is at the boundary of the triangulation.
    if (ttl::isBoundaryNode(dart))
	cout << "The node is at the boundary" << endl;

  // Remove the node associated with the dart used above.
    ttl::removeNode<Traits>(dart);
  
    // Get the boundary of the triangulation represented as a list of darts.
    // Start with an arbitrary dart at the boundary.
    edge = triang.getBoundaryEdge();
    Dart b_dart(edge, &triang);
    list<Dart> boundary;
    ttl::getBoundary(b_dart,boundary);
    cout << "No. of edges on boundary = " << boundary.size() << endl;

  // Check if the triangulation is Delaunay
  // (This is not a TTL function)
    if (triang.checkDelaunay())
	cout << "Triangulation is Delaunay" << endl;
    else
	cout << "WARNING: Triangulation is not Delaunay" << endl;

  // Insert two nodes and then insert a constrained edge between them
  // (Note that this could also be implemented in the code for the data structure.
  //  Here we call ttl directly to demonstrate the generic concept.) 
    Dart d1 = triang.createDart(); // an arbitrary CCW dart
    NodePtr nn(new ExampleNode(0.1, 0.25, 0));
    ttl::insertNode<Traits>(d1, *nn);
    Dart d2 = triang.createDart();
    nn.reset(new ExampleNode(0.6, 0.85, 0));
    ttl::insertNode<Traits>(d2, *nn);
    // (Note that d1 is not necessarily valid after having inserted d2 since insertion
    //  of d2 may affect d1. Here d2 is "far from" d1, so we are relatively safe).
    bool optimizeDelaunay = true; // optimizes to a constrained Delaunay triangulation
    dart = ttl::insertConstraint<Traits>(d1, d2, optimizeDelaunay);

  // Set the edge as constrained (fixed) such that it is not swapped when inserting nodes later
    dart.getEdge()->setConstrained();

    // Insert nodes and a constraint near that above to demonstrate fixed edges
    d1 = triang.createDart();
    nn.reset(new ExampleNode(0.35, 0.56, 0));
    ttl::insertNode<Traits>(d1, *nn);
    d2 = triang.createDart();
    nn.reset(new ExampleNode(0.1, 0.9, 0));
    ttl::insertNode<Traits>(d2, *nn);
    dart = ttl::insertConstraint<Traits>(d1, d2, optimizeDelaunay);
    dart.getEdge()->setConstrained();

  // Check if the boundary is convex (in the plane)
    if (ttl::convexBoundary<Traits>(b_dart))
	cout << "\nBoundary is convex:" << endl;

  // Print the nodes at the boundary
    list<Dart>::const_iterator it;
    for (it = boundary.begin();  it != boundary.end(); ++it)
	cout << it->getNode().x() << " " << it->getNode().y() << '\n';
  
  // Print the triangulation (its edges) to file
  // (for gnuplot or other line drawing programs)
    ofstream ofile("qweEdge.dat");
    triang.printEdges(ofile);

    // Print it to a file in "g2" format
    ofstream ofile2("example.g2");
    triang.write(ofile2);
  
    return 0;
}
