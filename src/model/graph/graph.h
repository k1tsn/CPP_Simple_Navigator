// graph

#ifndef SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_H_
#define SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_H_

#include <string>
#include <vector>

#include "factory_graph_exporter.h"
#include "factory_graph_imp.h"
#include "factory_graph_importer.h"
#include "graph_exporter.h"
#include "graph_imp.h"
#include "matrix.h"
#include "token_parser.h"

namespace s21 {

class Graph {
 public:
  enum ReturnCode {
    kCodeOk,
    kCodeFileDontSave,
    kCodeFileDontOpen,
    kCodeInvalidFile,
    kCodeVertexAlreadyExits,
    kCodeVertexNotExist,
    kCodeEdgeAlreadyExist,
    kCodeEdgeNotExist,
    kCodeInvalidEdge,
  };

  using ImportType = FactoryGraphImporter::ImportType;
  using ExportType = FactoryGraphExporter::ExportType;
  using GraphImpType = FactoryGraphImp::GraphImpType;

  using Edge = GraphImp::Edge;
  using Vertex = GraphImp::Vertex;
  using EdgeWeightType = GraphImp::EdgeWeightType;
  using Edges = GraphImp::Edges;
  using Vertexes = GraphImp::Vertexes;

  Graph();
  Graph(const Graph& other);
  Graph(Graph&& other) noexcept;
  Graph& operator=(const Graph& other);
  Graph& operator=(Graph&& other) noexcept;
  virtual ~Graph();

  /// @brief Import graph from file.
  /// @param type file type.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontOpen if file dont open.
  /// @return kCodeInvalidFile if file is invalid.
  ReturnCode Import(const std::string& filename, ImportType type);

  /// @brief Export graph to file.
  /// @param type file type.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontSave if file dont create.
  ReturnCode Export(const std::string& filename, ExportType type) const;

  /// @brief Import graph from file with type matrix adjancency,
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontOpen if file dont open.
  /// @return kCodeInvalidFile if file is invalid.
  ReturnCode LoadGraphFromFile(const std::string& filename);

  /// @brief Export graph to file with type dot format.
  /// @return kCodeOk if all ok.
  /// @return kCodeFileDontSave if file dont create.
  ReturnCode ExportGraphToDot(const std::string& filename) const;

  /// @brief Add vertex.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexAlreadyExits if vertex already exist.
  ReturnCode AddVertex(Vertex vertex);

  /// @brief Remove vertex.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  ReturnCode RemoveVertex(Vertex vertex);

  /// @brief Add edge.
  /// @param vertex from witch vertex.
  /// @param edge edge.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  /// @return kCodeEdgeAlreadyExist if edge already exist.
  /// @return kCodeInvalidEdge if edge cant be added.
  ReturnCode AddEdge(Vertex vertex, const Edge& edge);

  /// @brief Remove edge.
  /// @param vertex from witch vertex.
  /// @param edge edge.
  /// @return kCodeOk if all ok.
  /// @return kCodeVertexNotExist if vertex not exist.
  /// @return kCodeEdgeNotExist if edge not exist.
  ReturnCode RemoveEdge(Vertex vertex, const Edge& edge);

  /// @brief Get edges of this vertex.
  /// @warning If vertex not exist, return Edges with size().
  Edges GetEdges(Vertex vertex) const;

  /// @brief Get verteces of this graph.
  Vertexes GetVertexes() const;

  /// @brief Clear graph.
  void Clear();

 private:
  void ConstructDefault();
  void CopyOther(const Graph& other);
  void MoveOther(Graph&& other);
  void Destroy();

  ReturnCode GraphImpToReturnCode(GraphImp::ReturnCode code) const;
  ReturnCode GraphExporterToReturnCode(GraphExporter::ReturnCode code) const;
  ReturnCode GraphImporterToReturnCode(GraphImporter::ReturnCode code) const;

  GraphImpType graph_imp_type_;
  GraphImp* graph_imp_;
};

}  // namespace s21

#endif  // SIMPLE_NAVIGATOR_MODEL_GRAPH_GRAPH_H_
