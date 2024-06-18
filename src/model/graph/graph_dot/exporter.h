#ifndef GRAPH_DOT_EXPORTER_H_
#define GRAPH_DOT_EXPORTER_H_

#include <fstream>
#include <string>

#include "graph.h"

namespace graph {

namespace GraphDot {

class Exporter {
 public:
  enum ReturnCode { kCodeOk, kCodeFileDontSave };

  Exporter();
  Exporter(const Exporter& other) = default;
  Exporter(Exporter&& other) noexcept = default;
  Exporter& operator=(const Exporter& other) = default;
  Exporter& operator=(Exporter&& other) noexcept = default;
  virtual ~Exporter();

  ReturnCode Export(const Graph& graph, const std::string& filename) const;

 private:
  static const int kSpaceCountStart_;
  static const int kSpaceCountAdd_;
  static const std::string kStrSpace_;
  static const std::string kDefaultGraphName_;
  static const std::string kStrGraphOrient;
  static const std::string kStrGraphNotOrient;
  static const std::string kStrGraphSubGraph_;
  static const std::string kStrOpenBrace_;
  static const std::string kStrCloseBrace_;
  static const std::string kStrOpenSquareBracket_;
  static const std::string kStrCloseSquareBracket_;
  static const std::string kStrSemicolon_;
  static const std::string kStrEq_;
  static const std::string kStrComma_;
  static const std::string kStrNode_;
  static const std::string kStrOrientLine_;
  static const std::string kStrNonOrientLine_;

  static void ExportPrintGraph(const Graph& graph,
                               const std::string* orient_name,
                               Graph::OrientType orient_type, int space_count,
                               std::ofstream& file, int graph_idx);

  static void ExportPrintProperties(const Graph& graph, int space_count,
                                    std::ofstream& file);
  static void ExportPrintNodeProperties(const Graph& graph, int space_count,
                                        std::ofstream& file);
  static void ExportPrintVertexes(const Graph& graph, int space_count,
                                  std::ofstream& file);
  static void ExportPrintEdges(const Graph& graph,
                               Graph::OrientType orient_type, int space_count,
                               std::ofstream& file);

  static std::string FormSpaceStr(int space_count);
  static std::string FormGraphName(int space_count, int graph_idx);
  static std::string FormPropertyWithSpaces(const std::string& name,
                                            const std::string& value);
  static std::string FormProperty(const std::string& name,
                                  const std::string& value,
                                  const std::string& spaces);
  static std::string FormPropertiesList(const Graph::Properties& properties);
};

}  // namespace GraphDot

}  // namespace graph

#endif  // GRAPH_DOT_EXPORTER_H_
