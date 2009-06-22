//===========================================================================
//                                                                           
// File: HeEdge.h                                                            
//                                                                           
// Created: Mon Jun 10 13:52:05 2002                                         
//                                                                           
// Author: Atgeirr F Rasmussen <atgeirr@sintef.no>
//                                                                           
// Revision: $Id: HeEdge.h,v 1.6 2004-01-21 12:18:11 bsp Exp $
//                                                                           
// Description:
//                                                                           
//===========================================================================

#ifndef _HEEDGE_H
#define _HEEDGE_H

namespace hed {

    // ---------------------------------------------------------------------
    /// \b Edge class in the in the half-edge data structure
    template <class NodeType>
    class HalfEdge
    {
    public:
    /** Constructor.
     * Detailed description.
     */
	HalfEdge() 
	    : twinEdge_(0), nextEdgeInFace_(0), isLeadingEdge_(false), isConstrained_(false)
	{
	}

        /// Destructor
	~HalfEdge()
	{
	    if(twinEdge_) twinEdge_->setTwinEdge(0);
	}
    
	void setSourceNode(boost::shared_ptr<NodeType> node)
	{
	    sourceNode_ = node;
	}
	void setNextEdgeInFace(HalfEdge* edge)
	{
	    nextEdgeInFace_ = edge;
	}
	void setTwinEdge(HalfEdge* edge)
	{
	    twinEdge_ = edge;
	}

	void setAsLeadingEdge(bool val=true)
	{
	    isLeadingEdge_ = val;
	}
	bool isLeadingEdge() const
	{
	    return isLeadingEdge_;
	}
	void setConstrained(bool val=true)
	{
	    isConstrained_ = val;
	    if (twinEdge_) twinEdge_->isConstrained_ = val;
	}
	bool isConstrained() const
	{
	    return isConstrained_;
	}

	HalfEdge* getTwinEdge() const {return twinEdge_;};
	HalfEdge* getNextEdgeInFace() const {return nextEdgeInFace_;}
	boost::shared_ptr<NodeType> getSourceNode() {return sourceNode_;}
	boost::shared_ptr<NodeType> getTargetNode() {return getNextEdgeInFace()->getSourceNode();}

    private:
	boost::shared_ptr<NodeType> sourceNode_;
	HalfEdge*        twinEdge_;
	HalfEdge*        nextEdgeInFace_;

	bool isLeadingEdge_;
	bool isConstrained_;

    };

    // Forward declaration of the triangulation class
    template <class NodeTraits> class Triangulation;

    // ---------------------------------------------------------------
    //--------------------------------------------
    // Dart class for the half-edge data structure
    //--------------------------------------------
    /** \class Dart
     * \brief \b Dart class for the half-edge data structure.
     *
     * See \ref api for a detailed description of how the member functions
     * should be implemented.
     *
     *  This is how the dart class is implemented for the half-edge data structure:
     *  \include HeDart.h
     */
    template <class NodeTraits>
    class Dart
    {
    public:
	typedef typename NodeTraits::NodeType NodeType;
	typedef HalfEdge<NodeType> Edge;
	/// Default constructor
	Dart() : edge_(0), dir_(true), triang_(0) {}
        /// Constructur
	Dart(Edge* edge, Triangulation<NodeTraits>* triang, bool dir = true)
	    : edge_(edge), dir_(dir), triang_(triang)
	{}
	/// Copy constructor
	Dart(const Dart& dart) : edge_(dart.edge_), dir_(dart.dir_), triang_(dart.triang_) {}
	/// Destructor
	~Dart() {}

	/// Assignment operator
	Dart& operator = (const Dart& dart) {
	    if (this == &dart)
		return *this;
	    edge_ = dart.edge_;
	    dir_  = dart.dir_;
	    triang_ = dart.triang_;
	    return *this;
	}
	/// Comparing dart objects
	bool operator==(const Dart& dart) const {
	    if (dart.edge_ == edge_ && dart.dir_ == dir_ && dart.triang_ == triang_)
		return true;
	    return false;
	}
	/// Comparing dart objects
	bool operator!=(const Dart& dart) const {
	    return !(dart==*this);
	}
	/// Maps the dart to a different node
	Dart& alpha0(){dir_ = !dir_; return *this;}
	/// Maps the dart to a different edge
	Dart& alpha1() {
	    if (dir_) {
		edge_ = edge_->getNextEdgeInFace()->getNextEdgeInFace();
		dir_ = false;
	    }
	    else {
		edge_ = edge_->getNextEdgeInFace();
		dir_ = true;
	    }
	    return *this;
	}
	/// Maps the dart to a different triangle. \b Note: the dart is not changed if it is at the boundary!
	Dart& alpha2() {
	    if (edge_->getTwinEdge()) {
		edge_ = edge_->getTwinEdge();
		dir_ = !dir_;
	    }
	    // else, the dart is at the boundary and should not be changed
	    return *this;
	}
    
	/** @name Utilities not required by TTL */
	//@{
	void init(Edge* edge, Triangulation<NodeTraits>* triang, bool dir = true)
	{
	    edge_ = edge; dir_ = dir; triang_ = triang;
	}

//  	double x() const {return getNode()->x();} // x-coordinate of source node
//  	double y() const {return getNode()->y();} // y-coordinate of source node
  	bool isCounterClockWise() const {return dir_;}
  	const NodeType& getOppositeNode() const {return dir_ ? (* edge_->getTargetNode()) : (* edge_->getSourceNode());}
 	const NodeType& getNode() const {return dir_ ? (* edge_->getSourceNode()) : (* edge_->getTargetNode());}
	Edge* getEdge() const {return edge_;}
	Triangulation<NodeTraits>* getTriang() const {return triang_;}
	
	//@}

    private:
	Edge* edge_;
	bool dir_; // true if dart is counterclockwise in face    
	Triangulation<NodeTraits>* triang_;
    };

} // namespace hed

#endif // _HEEDGE_H

