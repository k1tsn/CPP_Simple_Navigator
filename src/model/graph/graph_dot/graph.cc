#include "graph.h"

#include <map>
#include <string>
#include <vector>

namespace graph {

namespace GraphDot {

Graph::Graph() {}

Graph::~Graph() {}

void Graph::SetName(const std::string& name) { name_ = name; }

void Graph::SetOrientType(const OrientType& orient_type) {
  orient_type_ = orient_type;
}

void Graph::SetProperties(const Properties& properties) {
  properties_ = properties;
}

void Graph::SetNodeProperties(const Properties& node_properties) {
  node_properties_ = node_properties;
}

void Graph::SetVertexes(const Vertexes& vertexes) { vertexes_ = vertexes; }

void Graph::SetEdges(const Edges& edges) { edges_ = edges; }

void Graph::SetSubgraphs(const Graphs& subgraphs) { subgraphs_ = subgraphs; }

std::string& Graph::GetName() { return name_; }

Graph::OrientType& Graph::GetOrientType() { return orient_type_; }

Graph::Properties& Graph::GetProperties() { return properties_; }

Graph::Properties& Graph::GetNodeProperties() { return node_properties_; }

Graph::Vertexes& Graph::GetVertexes() { return vertexes_; }

Graph::Edges& Graph::GetEdges() { return edges_; }

Graph::Graphs& Graph::GetSubgraphs() { return subgraphs_; }

const std::string& Graph::GetName() const { return name_; }

const Graph::OrientType& Graph::GetOrientType() const { return orient_type_; }

const Graph::Properties& Graph::GetProperties() const { return properties_; }

const Graph::Properties& Graph::GetNodeProperties() const {
  return node_properties_;
}

const Graph::Vertexes& Graph::GetVertexes() const { return vertexes_; }

const Graph::Edges& Graph::GetEdges() const { return edges_; }

const Graph::Graphs& Graph::GetSubgraphs() const { return subgraphs_; }

Graph::Vertexes::iterator Graph::FindVertex(const Vertex& vertex) {
  Vertexes::iterator iter;
  bool find = false;
  for (auto i : subgraphs_) {
    iter = i.FindVertex(vertex);
    if (iter != i.GetVertexes().end()) {
      find = true;
      break;
    }
  }

  if (!find) iter = GetVertexes().find(vertex);
  return iter;
}

Graph::Vertexes::const_iterator Graph::FindVertex(const Vertex& vertex) const {
  Vertexes::const_iterator iter;
  bool find = false;
  for (auto i : subgraphs_) {
    iter = i.FindVertex(vertex);
    if (iter != i.GetVertexes().end()) {
      find = true;
      break;
    }
  }

  if (!find) iter = GetVertexes().find(vertex);
  return iter;
}

void Graph::AddVertexOrAppendProperties(const Vertex& vertex,
                                        const Properties& prop) {
  auto iter = FindVertex(vertex);
  if (iter == GetVertexes().end())
    iter = GetVertexes().insert({vertex, Properties()}).first;

  for (auto i : prop) {
    auto prop_iter = iter->second.find(i.first);
    if (prop_iter == iter->second.end())
      iter->second.insert({i.first, i.second});
    else
      prop_iter->second = i.second;
  }
}

}  // namespace GraphDot

}  // namespace graph
