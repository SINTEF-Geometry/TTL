This is the TTL (Triangulation Template Library) version 1.1.0 README file
==========================================================================

TTL is a generic triangulation library developed at SINTEF Applied
Mathematics.  TTL is generic in the sense that it does not rely on a
special data structure. Thus, you can operate with your own
application data structure and benefit from a variety of generic
algorithms in TTL that can work directly on any data structure for
triangulations.

If you do not want to bother with making your own data structure and adapt it to
TTL, you can use one of the data structures that comes with TTL and use TTL
as you would use any other triangulation library.

TTL runs on a number of platforms, including Unix, Linux and Windows.
If you have any problems it is straightforward to modify the source code.
The C++ compiler must support the syntax: function<  >(..,..) for calling
function templates.

Examples of generic tools currently available are:
- Incremental Delaunay triangulation
- Constrained Delaunay triangulation
- Insert and remove nodes in a triangulation
- Searching and traversal operations
- Misc. queries for extracting information for visualisation systems etc.

More information about the TTL library can be found at: 
  
  http://www.sintef.no/math


Please report any problems, bugs or comments to: jan.b.thomassen@sintef.no
