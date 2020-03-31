//
//  my_graph.cpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/13.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#include "my_graph.hpp"

Edge::Edge(const int v0, const int v1)
 {
     vertexIndices_[0] = v0;
     vertexIndices_[1] = v1;
 }

int Edge::operator[](const std::size_t j) const
 {
     assert(j < 2);

     return vertexIndices_[j];
 }
int& Edge::operator[](const std::size_t j)
{
    assert(j < 2);

    return vertexIndices_[j];
}

/// Construct an adjacency.
///
/// \param vertex Vertex.
/// \param edge Edge.
///
inline Adjacency::Adjacency(const int vertex, const int edge):vertex_(vertex), edge_(edge)
{}

/// Access the vertex.
///
inline int Adjacency::vertex() const
{
    return vertex_;
}

/// Access the vertex.
///
inline int& Adjacency::vertex()
{
    return vertex_;
}

/// Access the edge.
///
inline int Adjacency::edge() const
{
    return edge_;
}

/// Access the edge.
///
inline int& Adjacency::edge()
{
    return edge_;
}

/// Adjacencies are ordered first wrt the vertex, then wrt the edge.
///
inline bool Adjacency::operator<(const Adjacency& in) const
{
    if(vertex_ < in.vertex_)
    {
        return true;
    }
    else if(vertex_ == in.vertex_)
    {
        return edge_ < in.edge_;
    }
    else
    {
        return false;
    }
}

/// Adjacencies are ordered first wrt the vertex, then wrt the edge.
///
inline bool Adjacency::operator<=(const Adjacency& in) const
{
    if(vertex_ < in.vertex_)
    {
        return true;
    }
    else if(vertex_ == in.vertex_)
    {
        return edge_ <= in.edge_;
    }
    else
    {
        return false;
    }
}

/// Adjacencies are ordered first wrt the vertex, then wrt the edge.
///
inline bool Adjacency::operator>(const Adjacency& in) const
{
    if(vertex_ > in.vertex_)
    {
        return true;
    }
    else if(vertex_ == in.vertex_)
    {
        return edge_ > in.edge_;
    }
    else
    {
        return false;
    }
}

/// Adjacencies are ordered first wrt the vertex, then wrt the edge.
///
inline bool Adjacency::operator>=(const Adjacency& in) const
{
    if(vertex_ > in.vertex_)
    {
        return true;
    }
    else if(vertex_ == in.vertex_)
    {
        return edge_ >= in.edge_;
    }
    else
    {
        return false;
    }
}

/// Adjacencies are equal if both the vertex and the edge are equal.
///
inline bool Adjacency::operator==(const Adjacency& in) const
{
    return vertex_ == in.vertex_ && edge_ == in.edge_;
}

/// Adjacencies are unequal if either the vertex or the edge are unqual.
///
inline bool Adjacency::operator!=(const Adjacency& in) const
{
    return !(*this == in);
}


// implementation of IteratorHelper

template<bool T>
inline
IteratorHelper<T>::IteratorHelper()
:   Base()
{}

template<bool T>
inline
IteratorHelper<T>::IteratorHelper(
    const Base& it
)
:   Base(it)
{}

template<bool T>
inline
IteratorHelper<T>::IteratorHelper(
    const IteratorHelper<T>& it
)
:   Base(it)
{}

template<bool T>
inline IteratorHelper<T>
IteratorHelper<T>::operator=(
    const Base& it
) {
    Base::operator=(it);
    return *this;
}

template<bool T>
inline IteratorHelper<T>
IteratorHelper<T>::operator=(
    const IteratorHelper<T>& it
) {
    Base::operator=(it);
    return *this;
}

template<bool T>
inline typename IteratorHelper<T>::value_type
IteratorHelper<T>::operator*() const {
    if(T) { // evaluated at compile time
        return Base::operator*().vertex();
    }
    else {
        return Base::operator*().edge();
    }
}

template<bool T>
inline typename IteratorHelper<T>::value_type
IteratorHelper<T>::operator[](
    const std::size_t j
) const {
    if(T) { // evaluated at compile time
        return Base::operator[](j).vertex();
    }
    else {
        return Base::operator[](j).edge();
    }
}

template<bool T>
inline IteratorHelper<T>&
IteratorHelper<T>::operator+=(
    const difference_type d
) {
    Base::operator+=(d);
    return *this;
}

template<bool T>
inline IteratorHelper<T>&
IteratorHelper<T>::operator-=(
    const difference_type d
) {
    Base::operator-=(d);
    return *this;
}

template<bool T>
inline IteratorHelper<T>&
IteratorHelper<T>::operator++() { // prefix
    Base::operator++();
    return *this;
}

template<bool T>
inline IteratorHelper<T>&
IteratorHelper<T>::operator--() { // prefix
    Base::operator--();
    return *this;
}

template<bool T>
inline IteratorHelper<T>
IteratorHelper<T>::operator++(int) { // postfix
    return Base::operator++(int());
}

template<bool T>
inline IteratorHelper<T>
IteratorHelper<T>::operator--(int) { // postfix
    return Base::operator--(int());
}

template<bool T>
inline IteratorHelper<T>
IteratorHelper<T>::operator+(
    const difference_type d
) const {
    return Base::operator+(d);
}

template<bool T>
inline IteratorHelper<T>
IteratorHelper<T>::operator-(
    const difference_type d
) const {
    return Base::operator-(d);
}

#ifdef _MSC_VER
template<bool T>
inline typename IteratorHelper<T>::difference_type
IteratorHelper<T>::operator-(
    const IteratorHelper<T>& other
) const {
    return Base::operator-(other);
}
#endif
            
/// Construct an undirected graph.
///
/// \param visitor Visitor to follow changes of integer indices of vertices and edges.
///
inline Graph::Graph():vertices_(), edges_()
{}

/// Get an iterator to the beginning of the sequence of adjacencies that originate from a given vertex.
///
/// \param vertex Integer index of the vertex.
/// \return AdjacencyIterator.
///
/// \sa adjacenciesFromVertexEnd()
///
inline Graph::AdjacencyIterator Graph::adjacenciesFromVertexBegin(const std::size_t vertex) const
{
    return vertices_[vertex].begin();
}

/// Get an iterator to the end of the sequence of adjacencies that originate from a given vertex.
///
/// \param vertex Integer index of the vertex.
/// \return AdjacencyIterator.
///
/// \sa adjacenciesFromVertexBegin()
///
inline Graph::AdjacencyIterator Graph::adjacenciesFromVertexEnd(const std::size_t vertex) const
{
    return vertices_[vertex].end();
}

/// Search for an edge (in logarithmic time).
///
/// \param vertex0 first vertex of the edge.
/// \param vertex1 second vertex of the edge.
/// \return if an edge from vertex0 to vertex1 exists, pair.first is true
///     and pair.second is the index of such an edge. if no edge from vertex0
///     to vertex1 exists, pair.first is false and pair.second is undefined.
///
inline std::pair<bool, std::size_t> Graph::findEdge(const std::size_t vertex0, const std::size_t vertex1) const
{
    assert(vertex0 < numberOfVertices());
    assert(vertex1 < numberOfVertices());

    std::size_t v0 = vertex0;
    std::size_t v1 = vertex1;
    if(numberOfEdgesFromVertex(vertex1) < numberOfEdgesFromVertex(vertex0)) {
        v0 = vertex1;
        v1 = vertex0;
    }
    VertexIterator it = std::lower_bound(verticesFromVertexBegin(v0), verticesFromVertexEnd(v0), v1); // binary search
    if(it != verticesFromVertexEnd(v0) && *it == v1) {
        // access the corresponding edge in constant time
        const std::size_t j = std::distance(verticesFromVertexBegin(v0), it);
        return std::make_pair(true, edgeFromVertex(v0, j));
    }
    else {
        return std::make_pair(false, 0);
    }
}
    
/// Get an iterator to the beginning of the sequence of vertices reachable from a given vertex via a single edge.
///
/// \param vertex Integer index of the vertex.
/// \return VertexIterator.
///
/// \sa verticesFromVertexEnd()
///
inline VertexIterator Graph::verticesFromVertexBegin(const std::size_t vertex) const
{
    return vertices_[vertex].begin();
}

/// Get an iterator to the end of the sequence of vertices reachable from a given vertex via a single edge.
///
/// \param vertex Integer index of the vertex.
/// \return VertexIterator.
///
/// \sa verticesFromVertexBegin()
///
inline VertexIterator Graph::verticesFromVertexEnd(const std::size_t vertex) const
{
    return vertices_[vertex].end();
}

/// Get the number of vertices.
///
inline std::size_t Graph::numberOfVertices() const
{
    return vertices_.size();
}

/// Get the number of edges that originate from a given vertex.
///
/// \param vertex Integer index of a vertex.
///
/// \sa edgeFromVertex()
///
inline std::size_t Graph::numberOfEdgesFromVertex(const std::size_t vertex) const
{
    return vertices_[vertex].size();
}

/// Get the integer index of an edge that originates from a given vertex.
///
/// \param vertex Integer index of a vertex.
/// \param j Number of the edge; between 0 and numberOfEdgesFromVertex(vertex) - 1.
///
/// \sa numberOfEdgesFromVertex()
///
inline std::size_t Graph::edgeFromVertex(const std::size_t vertex, const std::size_t j) const
{
    return vertices_[vertex][j].edge();
}

/// Insert an additional vertex.
///
/// \return Integer index of the newly inserted vertex.
///
/// \sa insertVertices()
///
inline std::size_t Graph::insertVertex()
{
    vertices_.push_back(Adjacencies());
    return vertices_.size() - 1;
}
     
/// Insert an additional edge.
///
/// \param vertexIndex0 Integer index of the first vertex in the edge.
/// \param vertexIndex1 Integer index of the second vertex in the edge.
/// \return Integer index of the newly inserted edge.
///
inline std::size_t Graph::insertEdge(const std::size_t vertexIndex0, const std::size_t vertexIndex1)
{
    assert(vertexIndex0 < numberOfVertices());
    assert(vertexIndex1 < numberOfVertices());
    
    std::pair<bool, std::size_t> p = findEdge(vertexIndex0, vertexIndex1);
    if(p.first) { // edge already exists
        return p.second;
    }
    else {
        edges_.push_back(Edge(vertexIndex0, vertexIndex1));
        std::size_t edgeIndex = edges_.size() - 1;
        insertAdjacenciesForEdge(edgeIndex);
        return edgeIndex;
    }
}
            
int Graph::find_closed_node(point3D p)
{
//    std::vector<float> dists;
//    for (int i = 0; i < m_vVertexs->size(); i++)
//    {
//        point3D p1 = m_vVertexs->at(i).getData();
//        dists.push_back(p.distance(p1));
//    }
//    auto m = std::min_element(dists.begin(), dists.end());
//    return m_vVertexs->at(m - dists.begin());
    return 0;
}
