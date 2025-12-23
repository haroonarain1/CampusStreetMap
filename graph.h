#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
 private:
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> adjList;
  size_t edges = 0;

 public:
  /// Default constructor
  graph() {
    // TODO_STUDENT
  }

  /// @brief Add the vertex `v` to the graph, must typically be O(1).
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    if(adjList.find(v) != adjList.end()){
      return false;
    }

    adjList[v] = unordered_map<VertexT, WeightT>();
    return true;
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    auto fromEdge = adjList.find(from);
    auto toEdge = adjList.find(to);
    if(fromEdge == adjList.end() || toEdge == adjList.end()){
      return false;
    }
    auto &var = fromEdge->second;
    bool exists = (var.find(to) != var.end());
    var[to] = weight;
    if(!exists){
      edges++;
    }
    return true;
  }

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    auto fromEdge = adjList.find(from);
    if(fromEdge == adjList.end()){
      return false;
    }
    const auto &var = fromEdge->second;
    auto edgeVar = var.find(to);
    if(edgeVar == var.end()){
      return false;
    }
    weight = edgeVar->second;
    return true;
  }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    auto var = adjList.find(v);
    if(var == adjList.end()){
      return S;
    }
    for(const auto& j : var->second){
      S.insert(j.first);
    }
    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
    vector<VertexT> vertVar;
    vertVar.reserve(adjList.size());
    for(const auto& j : adjList){
      vertVar.push_back(j.first);
    }
    return vertVar;
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    // TODO_STUDENT
    return adjList.size();
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    // TODO_STUDENT
    return edges;
  }
};
