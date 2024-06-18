// graph importer adjacency matrix

#include "graph_importer_adjacency_matrix.h"

#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>

#include "graph.h"
#include "graph_importer.h"
#include "string_parser.h"

namespace graph {

GraphImporterAdjacencyMatrix::GraphImporterAdjacencyMatrix()
    : GraphImporter() {}

GraphImporterAdjacencyMatrix::~GraphImporterAdjacencyMatrix() {}

GraphImporterAdjacencyMatrix::ReturnCode GraphImporterAdjacencyMatrix::Import(
    const std::string& filename, Graph& graph) {
  graph.Clear();

  std::ifstream file(filename);
  if (file.fail()) return FileDontOpenCatch();

  TokenParser::Settings settings;
  settings.SetSpaceChars(settings.GetSpaceChars() + ",");
  TokenParser::StringParser parser(settings);

  std::string str;
  if (!std::getline(file, str)) return InvalidFileCatch(graph);
  parser.SetStr(&str);
  TokenParser::Token token = parser.NextInt();
  if (token.IsNull() || token.GetInt() < 0) return InvalidFileCatch(graph);

  size_t size = static_cast<size_t>(token.GetInt());
  for (size_t i = 0; i < size; ++i) {
    graph.AddVertex(Graph::Vertex(i + 1));
  }

  for (size_t i = 0; i < size; ++i) {
    if (!std::getline(file, str)) return InvalidFileCatch(graph);
    parser.SetStr(&str);
    parser.SetI(0);

    for (size_t j = 0; j < size; ++j) {
      token = parser.NextInt();
      if (token.IsNull()) return InvalidFileCatch(graph);

      Graph::Edge edge(token.GetInt(), Graph::Vertex(j + 1));
      graph.AddEdge(Graph::Vertex(i + 1), edge);
    }
  }

  return kCodeOk;
}

GraphImporterAdjacencyMatrix::ReturnCode
GraphImporterAdjacencyMatrix::FileDontOpenCatch() {
  return ReturnCode::kCodeFileDontOpen;
}

GraphImporterAdjacencyMatrix::ReturnCode
GraphImporterAdjacencyMatrix::InvalidFileCatch(Graph& graph) {
  graph.Clear();
  return ReturnCode::kCodeInvalidFile;
}

}  // namespace graph