//==================================================================================================
//
// File: HeDart.h
//
// Created: March 1 2001
//
// Author: Øyvind Hjelle <oyvind.hjelle@math.sintef.no>
//
// Revision: $Id: HeDart.h,v 1.2 2006/07/26 12:08:44 oyvindhj Exp $
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


#ifndef _HALF_EDGE_DART_
#define _HALF_EDGE_DART_


#include <ttl/halfedge/HeTriang.h>


namespace hed {


  //------------------------------------------------------------------------------------------------
  // Dart class for the half-edge data structure
  //------------------------------------------------------------------------------------------------

  /** \class Dart
  *   \brief \b %Dart class for the half-edge data structure.
  *
  *   See \ref api for a detailed description of how the member functions
  *   should be implemented.
  */

  class Dart {

    Edge* edge_;
    bool dir_; // true if dart is counterclockwise in face

  public:
    /// Default constructor
    Dart() { edge_ = NULL; dir_ = true; }

    /// Constructor
    Dart(Edge* edge, bool dir = true) { edge_ = edge; dir_ = dir; }

    /// Copy constructor
    Dart(const Dart& dart) { edge_ = dart.edge_; dir_ = dart.dir_; }

    /// Destructor
    ~Dart() {}

    /// Assignment operator
    Dart& operator = (const Dart& dart) {
      if (this == &dart)
        return *this;
      edge_ = dart.edge_;
      dir_  = dart.dir_;
      return *this;
    }

    /// Comparing dart objects
    bool operator==(const Dart& dart) const {
      if (dart.edge_ == edge_ && dart.dir_ == dir_)
        return true;
      return false;
    }

    /// Comparing dart objects
    bool operator!=(const Dart& dart) const {
      return !(dart==*this);
    }

    /// Maps the dart to a different node
    Dart& alpha0() { dir_ = !dir_; return *this; }

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


    // Utilities not required by TTL
    // -----------------------------

    /** @name Utilities not required by TTL */
    //@{

    void init(Edge* edge, bool dir = true) { edge_ = edge; dir_ = dir; }

    double x() const { return getNode()->x(); } // x-coordinate of source node
    double y() const { return getNode()->y(); } // y-coordinate of source node

    bool isCounterClockWise() const { return dir_; }

    Node* getNode() const { return dir_ ? edge_->getSourceNode() : edge_->getTargetNode(); }
    Node* getOppositeNode() const { return dir_ ? edge_->getTargetNode() : edge_->getSourceNode(); }
    Edge* getEdge() const { return edge_; }

    //@} // End of Utilities not required by TTL

  };

}; // End of hed namespace

#endif
