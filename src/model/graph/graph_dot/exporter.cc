#include "exporter.h"

#include <fstream>
#include <string>

#include "graph.h"

namespace graph {

namespace GraphDot {

Exporter::Exporter() {}

Exporter::~Exporter() {}

Exporter::ReturnCode Exporter::Export(const Graph& graph,
                                      const std::string& filename) const {
  std::ofstream file(filename);
  if (file.fail()) return ReturnCode::kCodeFileDontSave;

  const std::string* orient_name = &kStrGraphOrient;
  if (graph.GetOrientType() == Graph::OrientType::kTypeNonOrient)
    orient_name = &kStrGraphNotOrient;

  ExportPrintGraph(graph, orient_name, graph.GetOrientType(), kSpaceCountStart_,
                   file, 0);

  return ReturnCode::kCodeOk;
}

const int Exporter::kSpaceCountStart_ = 0;
const int Exporter::kSpaceCountAdd_ = 2;
const std::string Exporter::kStrSpace_ = " ";
const std::string Exporter::kDefaultGraphName_ = "graphname";
const std::string Exporter::kStrGraphOrient = "digraph";
const std::string Exporter::kStrGraphNotOrient = "graph";
const std::string Exporter::kStrGraphSubGraph_ = "subgraph";
const std::string Exporter::kStrOpenBrace_ = "{";
const std::string Exporter::kStrCloseBrace_ = "}";
const std::string Exporter::kStrOpenSquareBracket_ = "[";
const std::string Exporter::kStrCloseSquareBracket_ = "]";
const std::string Exporter::kStrSemicolon_ = ";";
const std::string Exporter::kStrEq_ = "=";
const std::string Exporter::kStrComma_ = ",";
const std::string Exporter::kStrNode_ = "node";
const std::string Exporter::kStrNonOrientLine_ = "--";
const std::string Exporter::kStrOrientLine_ = "->";

void Exporter::ExportPrintGraph(const Graph& graph,
                                const std::string* orient_name,
                                Graph::OrientType orient_type, int space_count,
                                std::ofstream& file, int graph_idx) {
  std::string spaces = FormSpaceStr(space_count);
  std::string graph_name = graph.GetName();
  if (graph_name == "") graph_name = FormGraphName(space_count, graph_idx);

  file << spaces << *orient_name << kStrSpace_ << graph_name;
  file << kStrSpace_ << kStrOpenBrace_ << std::endl;

  int space_count_next = space_count + kSpaceCountAdd_;
  orient_name = &kStrGraphSubGraph_;

  int graph_i = 0;
  for (auto i : graph.GetSubgraphs()) {
    ExportPrintGraph(i, orient_name, orient_type, space_count_next, file,
                     graph_i);
    ++graph_i;
  }

  ExportPrintProperties(graph, space_count_next, file);
  ExportPrintNodeProperties(graph, space_count_next, file);
  ExportPrintVertexes(graph, space_count_next, file);
  ExportPrintEdges(graph, orient_type, space_count_next, file);

  file << spaces << kStrCloseBrace_ << std::endl;
}

void Exporter::ExportPrintProperties(const Graph& graph, int space_count,
                                     std::ofstream& file) {
  std::string spaces = FormSpaceStr(space_count);
  for (auto i : graph.GetProperties()) {
    file << spaces << FormProperty(i.first, i.second, kStrSpace_);
    file << kStrSemicolon_ << std::endl;
  }
}

void Exporter::ExportPrintNodeProperties(const Graph& graph, int space_count,
                                         std::ofstream& file) {
  if (graph.GetNodeProperties().empty()) return;
  file << FormSpaceStr(space_count) << kStrNode_ << kStrSpace_;
  file << FormPropertiesList(graph.GetNodeProperties());
  file << kStrSemicolon_ << std::endl;
}

void Exporter::ExportPrintVertexes(const Graph& graph, int space_count,
                                   std::ofstream& file) {
  std::string spaces = FormSpaceStr(space_count);

  for (auto i : graph.GetVertexes()) {
    file << spaces << i.first;

    if (!i.second.empty()) {
      file << kStrSpace_ << FormPropertiesList(i.second);
    }
    file << kStrSemicolon_ << std::endl;
  }
}

void Exporter::ExportPrintEdges(const Graph& graph,
                                Graph::OrientType orient_type, int space_count,
                                std::ofstream& file) {
  std::string spaces = FormSpaceStr(space_count);
  const std::string* line = &kStrOrientLine_;
  if (orient_type == Graph::OrientType::kTypeNonOrient)
    line = &kStrNonOrientLine_;

  for (auto i : graph.GetEdges()) {
    file << spaces << i.vertex_ << *line << i.dist_vertex_;
    if (!i.properties_.empty()) {
      file << kStrSpace_ << FormPropertiesList(i.properties_);
    }
    file << kStrSemicolon_ << std::endl;
  }
}

std::string Exporter::FormGraphName(int space_count, int graph_idx) {
  const std::string& base_name = kDefaultGraphName_;
  std::string delim = "_";
  std::string idx_in =
      std::to_string((space_count - kSpaceCountStart_) / kSpaceCountAdd_);
  std::string idx_num = std::to_string(graph_idx);
  return (base_name + delim + idx_in + delim + idx_num);
}

std::string Exporter::FormSpaceStr(int space_count) {
  std::string spaces;
  for (int i = 0; i != space_count; ++i) spaces.append(kStrSpace_);
  return spaces;
}

std::string Exporter::FormProperty(const std::string& name,
                                   const std::string& value,
                                   const std::string& spaces) {
  return name + spaces + kStrEq_ + spaces + value;
}

std::string Exporter::FormPropertiesList(const Graph::Properties& properties) {
  if (properties.empty()) return "";

  std::string properrties_list = kStrOpenSquareBracket_;
  auto i = properties.begin();
  auto i_next = i;
  ++i_next;
  for (; i_next != properties.end(); ++i, ++i_next) {
    properrties_list = properrties_list + FormProperty(i->first, i->second, "");
    properrties_list = properrties_list + kStrComma_ + kStrSpace_;
  }

  properrties_list = properrties_list + FormProperty(i->first, i->second, "");
  properrties_list = properrties_list + kStrCloseSquareBracket_;

  return properrties_list;
}

}  // namespace GraphDot

}  // namespace graph
