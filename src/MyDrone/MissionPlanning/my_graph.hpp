//
//  my_graph.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/13.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef my_graph_hpp
#define my_graph_hpp

#include "my_point.hpp"

#include <vector>
using namespace std;

class Edge
{
public:
    Edge(const int v0, const int v1);
    int operator[](const std::size_t j) const;
    int& operator[](const std::size_t j);

private:
   int vertexIndices_[2];
};
           
class Adjacency
{
public:
   Adjacency(const int, const int);
   int vertex() const;
   int& vertex();
   int edge() const;
   int& edge();
   bool operator<(const Adjacency&) const;
   bool operator<=(const Adjacency&) const;
   bool operator>(const Adjacency&) const;
   bool operator>=(const Adjacency&) const;
   bool operator==(const Adjacency&) const;
   bool operator!=(const Adjacency&) const;

private:
   int vertex_;
   int edge_;
};
               
typedef vector<Adjacency> Adjacencies;

template<bool T>
class IteratorHelper
:   public Adjacencies::const_iterator
{
private:
    typedef typename Adjacencies::const_iterator Base;

public:
    typedef typename Base::iterator_category iterator_category;
    typedef typename Base::difference_type difference_type;
    typedef const std::size_t value_type;
    typedef value_type* pointer;
    typedef value_type& reference;

    // construction and assignment
    IteratorHelper();
    IteratorHelper(const Base&);
    IteratorHelper(const IteratorHelper<T>&);
    IteratorHelper operator=(const Base&);
    IteratorHelper operator=(const IteratorHelper<T>&);

    // increment and decrement
    IteratorHelper<T>& operator+=(const difference_type);
    IteratorHelper<T>& operator-=(const difference_type);
    IteratorHelper<T>& operator++(); // prefix
    IteratorHelper<T>& operator--(); // prefix
    IteratorHelper<T> operator++(int); // postfix
    IteratorHelper<T> operator--(int); // postfix
    IteratorHelper<T> operator+(const difference_type) const;
    IteratorHelper<T> operator-(const difference_type) const;
    #ifdef _MSC_VER
    difference_type operator-(const IteratorHelper<T>&) const;
    #endif

    // access
    value_type operator*() const;
    value_type operator[](const std::size_t j) const;
private:
    pointer operator->() const;
};

typedef IteratorHelper<true> VertexIterator;
typedef IteratorHelper<false> EdgeIterator;

class Graph
{
public:
    typedef Adjacencies::const_iterator AdjacencyIterator;

    Graph();

    AdjacencyIterator adjacenciesFromVertexBegin(const std::size_t) const;
    AdjacencyIterator adjacenciesFromVertexEnd(const std::size_t) const;

    VertexIterator verticesFromVertexBegin(const std::size_t) const;
    VertexIterator verticesFromVertexEnd(const std::size_t) const;
    
    std::pair<bool, std::size_t> findEdge(const std::size_t, const std::size_t) const;

    // manipulation
    std::size_t insertVertex();
    std::size_t insertEdge(const std::size_t, const std::size_t);
    int find_closed_node(point3D p);

    std::size_t numberOfVertices() const;
    std::size_t numberOfEdgesFromVertex(const std::size_t) const;
    std::size_t edgeFromVertex(const std::size_t, const std::size_t) const;

private:
    void insertAdjacenciesForEdge(const std::size_t);

    std::vector<Adjacencies> vertices_;
    std::vector<Edge> edges_;
};
#endif /* my_graph_hpp */
