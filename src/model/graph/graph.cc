// graph

#include "graph.h"

#include <string>
#include <utility>

#include "factory_graph_exporter.h"
#include "factory_graph_imp.h"
#include "factory_graph_importer.h"
#include "graph_exporter.h"
#include "graph_imp.h"
#include "graph_importer.h"

namespace graph {

Graph::Graph() { ConstructDefault(); }

Graph::Graph(const Graph& other) { CopyOther(other); }

Graph::Graph(Graph&& other) noexcept { MoveOther(std::move(other)); }

Graph& Graph::operator=(const Graph& other) {
  if (this == &other) return *this;
  Destroy();
  CopyOther(other);
  return *this;
}

Graph& Graph::operator=(Graph&& other) noexcept {
  if (this == &other) return *this;
  Destroy();
  CopyOther(std::move(other));
  return *this;
}

Graph::~Graph() { Destroy(); }

void Graph::ConstructDefault() {
  FactoryGraphImp factory;
  graph_imp_type_ = GraphImpType::kTypeAdjacencyMatrix;
  graph_imp_ = factory.Create(graph_imp_type_);
}

void Graph::CopyOther(const Graph& other) {
  graph_imp_type_ = other.graph_imp_type_;
  FactoryGraphImp factory;
  graph_imp_ = factory.Create(graph_imp_type_);

  Vertexes verteces = other.GetVertexes();
  for (auto i : verteces) {
    AddVertex(i);
  }

  for (auto i : verteces) {
    Edges edges = other.GetEdges(i);
    for (auto j : edges) {
      AddEdge(i, j);
    }
  }
}

void Graph::MoveOther(Graph&& other) {
  graph_imp_type_ = other.graph_imp_type_;
  graph_imp_ = other.graph_imp_;
  other.ConstructDefault();
}

void Graph::Destroy() { delete graph_imp_; }

Graph::ReturnCode Graph::Import(const std::string& filename, ImportType type) {
  FactoryGraphImporter factory;
  GraphImporter* importer = factory.Create(type);
  GraphImporter::ReturnCode code = importer->Import(filename, *this);
  delete importer;
  return GraphImporterToReturnCode(code);
}

Graph::ReturnCode Graph::Export(const std::string& filename,
                                ExportType type) const {
  FactoryGraphExporter factory;
  GraphExporter* exporter = factory.Create(type);
  GraphExporter::ReturnCode code = exporter->Export(*this, filename);
  delete exporter;
  return GraphExporterToReturnCode(code);
}

Graph::ReturnCode Graph::LoadGraphFromFile(const std::string& filename) {
  return Import(filename, ImportType::kTypeAdjacencyMatrix);
}

Graph::ReturnCode Graph::ExportGraphToDot(const std::string& filename) const {
  return Export(filename, ExportType::kTypeDotFormat);
}

Graph::ReturnCode Graph::AddVertex(Vertex vertex) {
  return GraphImpToReturnCode(graph_imp_->AddVertex(vertex));
}
Graph::ReturnCode Graph::RemoveVertex(Vertex vertex) {
  return GraphImpToReturnCode(graph_imp_->RemoveVertex(vertex));
}
Graph::ReturnCode Graph::AddEdge(Vertex vertex, const Edge& edge) {
  return GraphImpToReturnCode(graph_imp_->AddEdge(vertex, edge));
}
Graph::ReturnCode Graph::RemoveEdge(Vertex vertex, const Edge& edge) {
  return GraphImpToReturnCode(graph_imp_->RemoveEdge(vertex, edge));
}

Graph::Edges Graph::GetEdges(Vertex vertex) const {
  return graph_imp_->GetEdges(vertex);
}
Graph::Vertexes Graph::GetVertexes() const { return graph_imp_->GetVertexes(); }

void Graph::Clear() { graph_imp_->Clear(); }

Graph::ReturnCode Graph::GraphImpToReturnCode(GraphImp::ReturnCode code) const {
  switch (code) {
    case GraphImp::ReturnCode::kCodeVertexAlreadyExits:
      return kCodeVertexAlreadyExits;
      break;
    case GraphImp::ReturnCode::kCodeVertexNotExist:
      return kCodeVertexNotExist;
      break;
    case GraphImp::ReturnCode::kCodeEdgeAlreadyExist:
      return kCodeEdgeAlreadyExist;
      break;
    case GraphImp::ReturnCode::kCodeEdgeNotExist:
      return kCodeEdgeNotExist;
      break;
    case GraphImp::ReturnCode::kCodeInvalidEdge:
      return kCodeInvalidEdge;
      break;
    default:
      break;
  }

  return kCodeOk;
}

Graph::ReturnCode Graph::GraphExporterToReturnCode(
    GraphExporter::ReturnCode code) const {
  switch (code) {
    case GraphExporter::ReturnCode::kCodeFileDontSave:
      return kCodeFileDontSave;
      break;
    default:
      break;
  }

  return kCodeOk;
}

Graph::ReturnCode Graph::GraphImporterToReturnCode(
    GraphImporter::ReturnCode code) const {
  switch (code) {
    case GraphImporter::ReturnCode::kCodeFileDontOpen:
      return kCodeFileDontOpen;
      break;
    case GraphImporter::ReturnCode::kCodeInvalidFile:
      return kCodeInvalidFile;
      break;
    default:
      break;
  }

  return kCodeOk;
}

}  // namespace graph
