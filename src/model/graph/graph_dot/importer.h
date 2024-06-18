#ifndef GRAPH_DOT_IMPORTER_H_
#define GRAPH_DOT_IMPORTER_H_

#include <string>

#include "file_parser.h"
#include "graph.h"

namespace graph {

namespace GraphDot {

class Importer {
 public:
  enum ReturnCode { kCodeOk, kCodeFileDontOpen, kCodeInvalidFile };

  Importer();
  Importer(const Importer& other) = default;
  Importer(Importer&& other) noexcept = default;
  Importer& operator=(const Importer& other) = default;
  Importer& operator=(Importer&& other) noexcept = default;
  virtual ~Importer();

  ReturnCode Import(const std::string& filename, Graph& graph) const;

 private:
  using Parser = ::TokenParser::FileParser;
  enum Token {
    kTokenDigraph,
    kTokenGraph,
    kTokenSubgraph,
    kTokenNode,
    kTokenOpenBrace,
    kTokenCloseBrace,
    kTokenOpenSquareBracket,
    kTokenClostSquareBracket,
    kTokenSemcolon,
    kTokenComma,
    kTokenEqSign,
    kTokenLineOrient,
    kTokenLineNonOrient,
  };

  static const std::string kStrTokenDigraph_;
  static const std::string kStrTokenGraph_;
  static const std::string kStrTokenSubgraph_;
  static const std::string kStrTokenNode_;
  static const std::string kStrTokenOpenBrace_;
  static const std::string kStrTokenCloseBrace_;
  static const std::string kStrTokenOpenSquareBracket_;
  static const std::string kStrTokenClostSquareBracket_;
  static const std::string kStrTokenSemcolon_;
  static const std::string kStrTokenComma_;
  static const std::string kStrTokenEqSign_;
  static const std::string kStrTokenLineOrient_;
  static const std::string kStrTokenLineNonOrient_;

  static const ::TokenParser::Settings::TokenIds kTokenIds_;
  static const ::TokenParser::Settings::AppropriateQuotes kAppropriateQuotes_;
  static const std::string kStrWordDelimChars_;

  Parser GetParser() const;

  ReturnCode ImportGraph(Parser& parser, Graph& graph) const;

  ReturnCode NextSubgraph(Parser& parser, Graph& graph) const;
  ReturnCode NextNode(Parser& parser, Graph& graph) const;
  ReturnCode NextProperty(Parser& parser, Graph& graph,
                          const std::string& name) const;
  ReturnCode NextVertexProperty(Parser& parser, Graph& graph,
                                const std::string& vertex) const;
  ReturnCode NextVertex(Parser& parser, Graph& graph,
                        const std::string& vertex) const;
  ReturnCode NextEdges(Parser& parser, Graph& graph,
                       const std::string& v1) const;

  ReturnCode NextPropertiesList(Parser& parser,
                                Graph::Properties& properties) const;
};

}  // namespace GraphDot

}  // namespace graph

#endif  // GRAPH_DOT_IMPORTER_H_
