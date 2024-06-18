#ifndef GRAPH_DOT_GRAPH_H_
#define GRAPH_DOT_GRAPH_H_

#include <map>
#include <string>
#include <vector>

namespace graph {

namespace GraphDot {

class Graph {
 public:
  enum OrientType { kTypeOrient, kTypeNonOrient };
  using PropertyName = std::string;
  using PropertyValue = std::string;
  using Properties = std::map<PropertyName, PropertyValue>;
  using Vertex = std::string;
  using Vertexes = std::map<Vertex, Properties>;
  struct Edge {
    Vertex vertex_;
    Vertex dist_vertex_;
    Properties properties_;
  };
  using Edges = std::vector<Edge>;
  using Graphs = std::vector<Graph>;

  Graph();
  Graph(const Graph& other) = default;
  Graph(Graph&& other) noexcept = default;
  Graph& operator=(const Graph& other) = default;
  Graph& operator=(Graph&& other) noexcept = default;
  virtual ~Graph();

  void SetName(const std::string& name);
  void SetOrientType(const OrientType& orient_type);
  void SetProperties(const Properties& properties);
  void SetNodeProperties(const Properties& node_properties);
  void SetVertexes(const Vertexes& vertexes);
  void SetEdges(const Edges& edges);
  void SetSubgraphs(const Graphs& subgraphs);

  std::string& GetName();
  OrientType& GetOrientType();
  Properties& GetProperties();
  Properties& GetNodeProperties();
  Vertexes& GetVertexes();
  Edges& GetEdges();
  Graphs& GetSubgraphs();

  const std::string& GetName() const;
  const OrientType& GetOrientType() const;
  const Properties& GetProperties() const;
  const Properties& GetNodeProperties() const;
  const Vertexes& GetVertexes() const;
  const Edges& GetEdges() const;
  const Graphs& GetSubgraphs() const;

  Vertexes::iterator FindVertex(const Vertex& vertex);
  Vertexes::const_iterator FindVertex(const Vertex& vertex) const;

  void AddVertexOrAppendProperties(const Vertex& vertex,
                                   const Properties& prop = Properties());

 private:
  std::string name_;
  OrientType orient_type_;
  Properties properties_;
  Properties node_properties_;
  Vertexes vertexes_;
  Edges edges_;
  Graphs subgraphs_;
};

}  // namespace GraphDot

}  // namespace graph

#endif  // GRAPH_DOT_GRAPH_H_
