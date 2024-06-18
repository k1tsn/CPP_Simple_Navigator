#include "importer.h"

#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "file_parser.h"
#include "graph.h"

namespace graph {

namespace GraphDot {

Importer::Importer() {}

Importer::~Importer() {}

Importer::ReturnCode Importer::Import(const std::string& filename,
                                      Graph& graph) const {
  Parser parser = GetParser();
  parser.SetFile(filename);
  if (parser.IsEnd()) return ReturnCode::kCodeFileDontOpen;

  TokenParser::Token graph_type = parser.NextId();
  Graph::OrientType type = Graph::OrientType::kTypeNonOrient;
  if (graph_type.IsNull())
    return ReturnCode::kCodeInvalidFile;
  else if (graph_type.GetId() == Token::kTokenGraph)
    type = Graph::OrientType::kTypeNonOrient;
  else if (graph_type.GetId() == Token::kTokenDigraph)
    type = Graph::OrientType::kTypeOrient;
  else
    return ReturnCode::kCodeInvalidFile;

  Graph res_graph;
  res_graph.SetOrientType(type);
  ReturnCode code = ImportGraph(parser, res_graph);

  if (code == kCodeOk) graph = res_graph;
  return code;
}

const std::string Importer::kStrTokenDigraph_ = "digraph";
const std::string Importer::kStrTokenGraph_ = "graph";
const std::string Importer::kStrTokenSubgraph_ = "subgraph";
const std::string Importer::kStrTokenNode_ = "node";
const std::string Importer::kStrTokenOpenBrace_ = "{";
const std::string Importer::kStrTokenCloseBrace_ = "}";
const std::string Importer::kStrTokenOpenSquareBracket_ = "[";
const std::string Importer::kStrTokenClostSquareBracket_ = "]";
const std::string Importer::kStrTokenSemcolon_ = ";";
const std::string Importer::kStrTokenComma_ = ",";
const std::string Importer::kStrTokenEqSign_ = "=";
const std::string Importer::kStrTokenLineOrient_ = "->";
const std::string Importer::kStrTokenLineNonOrient_ = "--";

const TokenParser::Settings::TokenIds Importer::kTokenIds_ = {
    {kTokenDigraph, kStrTokenDigraph_},
    {kTokenGraph, kStrTokenGraph_},
    {kTokenSubgraph, kStrTokenSubgraph_},
    {kTokenNode, kStrTokenNode_},
    {kTokenOpenBrace, kStrTokenOpenBrace_},
    {kTokenCloseBrace, kStrTokenCloseBrace_},
    {kTokenOpenSquareBracket, kStrTokenOpenSquareBracket_},
    {kTokenClostSquareBracket, kStrTokenClostSquareBracket_},
    {kTokenSemcolon, kStrTokenSemcolon_},
    {kTokenComma, kStrTokenComma_},
    {kTokenEqSign, kStrTokenEqSign_},
    {kTokenLineOrient, kStrTokenLineOrient_},
    {kTokenLineNonOrient, kStrTokenLineNonOrient_},
};

const TokenParser::Settings::AppropriateQuotes Importer::kAppropriateQuotes_ = {
    {'"', '"'},
    {'\'', '\''},
};

const std::string Importer::kStrWordDelimChars_ = "\"\'{}[];,=->";

Importer::Parser Importer::GetParser() const {
  Parser parser;
  TokenParser::Settings settings;
  settings.SetTokenIds(kTokenIds_);
  settings.SetAppropriateQuotes(kAppropriateQuotes_);
  settings.SetWordDelim(settings.GetWordDelimChars() + kStrWordDelimChars_);
  settings.SetTokenIdIsFullWord(true);
  settings.SetWordMaySurrondedByQoutes(true);
  parser.SetSettings(std::move(settings));
  return parser;
}

Importer::ReturnCode Importer::ImportGraph(Parser& parser, Graph& graph) const {
  Token valid_orient = Token::kTokenLineOrient;
  if (graph.GetOrientType() == Graph::OrientType::kTypeNonOrient)
    valid_orient = Token::kTokenLineNonOrient;

  graph.SetName(parser.NextWord());
  TokenParser::Token open_brace = parser.NextThisId(kTokenOpenBrace);
  if (open_brace.IsNull()) return ReturnCode::kCodeInvalidFile;

  ReturnCode code = ReturnCode::kCodeOk;
  while (!parser.IsEnd() && code == ReturnCode::kCodeOk) {
    TokenParser::Token token = parser.NextId();
    if (token.IsId()) {
      if (token.GetId() == Token::kTokenSubgraph)
        code = NextSubgraph(parser, graph);
      else if (token.GetId() == Token::kTokenNode)
        code = NextNode(parser, graph);
      else if (token.GetId() == Token::kTokenCloseBrace)
        break;
      else
        code = ReturnCode::kCodeInvalidFile;

    } else {
      std::string word = parser.NextWord();
      TokenParser::Token token = parser.NextId();
      if (word == "" || token.IsNull())
        code = ReturnCode::kCodeInvalidFile;
      else if (token.GetId() == kTokenEqSign)
        code = NextProperty(parser, graph, word);
      else if (token.GetId() == kTokenOpenSquareBracket)
        code = NextVertexProperty(parser, graph, word);
      else if (token.GetId() == kTokenSemcolon)
        code = NextVertex(parser, graph, word);
      else if (token.GetId() == valid_orient)
        code = NextEdges(parser, graph, word);
      else
        code = ReturnCode::kCodeInvalidFile;
    }
  }

  return code;
}

Importer::ReturnCode Importer::NextSubgraph(Parser& parser,
                                            Graph& graph) const {
  Graph subgraph;
  subgraph.SetOrientType(graph.GetOrientType());
  ReturnCode code = ImportGraph(parser, subgraph);
  if (code == ReturnCode::kCodeOk) graph.GetSubgraphs().push_back(subgraph);
  return code;
}

Importer::ReturnCode Importer::NextNode(Parser& parser, Graph& graph) const {
  Graph::Properties prop;
  TokenParser::Token open_bracket =
      parser.NextThisId(Token::kTokenOpenSquareBracket);
  if (open_bracket.IsNull()) return ReturnCode::kCodeInvalidFile;
  ReturnCode code = NextPropertiesList(parser, prop);
  if (code == ReturnCode::kCodeOk) {
    graph.SetNodeProperties(prop);
    TokenParser::Token sem = parser.NextThisId(Token::kTokenSemcolon);
    if (sem.IsNull()) code = ReturnCode::kCodeInvalidFile;
  }
  return code;
}

Importer::ReturnCode Importer::NextProperty(Parser& parser, Graph& graph,
                                            const std::string& name) const {
  std::string value = parser.NextWord();
  TokenParser::Token semicolon = parser.NextThisId(Token::kTokenSemcolon);
  if (semicolon.IsNull()) return ReturnCode::kCodeInvalidFile;
  graph.GetProperties().insert({name, value});
  return ReturnCode::kCodeOk;
}

Importer::ReturnCode Importer::NextVertexProperty(
    Parser& parser, Graph& graph, const std::string& vertex) const {
  Graph::Properties prop;
  ReturnCode code = NextPropertiesList(parser, prop);
  if (code == ReturnCode::kCodeOk) {
    graph.AddVertexOrAppendProperties(vertex, prop);
    TokenParser::Token sem = parser.NextThisId(Token::kTokenSemcolon);
    if (sem.IsNull()) code = ReturnCode::kCodeInvalidFile;
  }
  return code;
}

Importer::ReturnCode Importer::NextVertex(Parser& parser, Graph& graph,
                                          const std::string& vertex) const {
  (void)parser;
  graph.AddVertexOrAppendProperties(vertex);
  return ReturnCode::kCodeOk;
}

Importer::ReturnCode Importer::NextEdges(Parser& parser, Graph& graph,
                                         const std::string& v1) const {
  Token valid_orient = Token::kTokenLineOrient;
  if (graph.GetOrientType() == Graph::OrientType::kTypeNonOrient)
    valid_orient = Token::kTokenLineNonOrient;

  std::vector<Graph::Edge> edges;

  std::string vertex = v1;
  std::string dist_vertex;
  TokenParser::Token token;

  ReturnCode code = ReturnCode::kCodeOk;
  do {
    dist_vertex = parser.NextWord();
    token = parser.NextId();

    if (dist_vertex.empty() || token.IsNull()) {
      return ReturnCode::kCodeInvalidFile;

    } else if (token.GetId() == valid_orient) {
      edges.push_back({vertex, dist_vertex, Graph::Properties()});

    } else if (token.GetId() == Token::kTokenSemcolon) {
      edges.push_back({vertex, dist_vertex, Graph::Properties()});
      break;

    } else if (token.GetId() == Token::kTokenOpenSquareBracket) {
      edges.push_back({vertex, dist_vertex, Graph::Properties()});
      Graph::Properties prop;
      code = NextPropertiesList(parser, prop);
      if (code == ReturnCode::kCodeOk) {
        for (auto& i : edges) i.properties_ = prop;

        TokenParser::Token sem = parser.NextThisId(Token::kTokenSemcolon);
        if (sem.IsNull()) code = ReturnCode::kCodeInvalidFile;
      }
      break;

    } else {
      return ReturnCode::kCodeInvalidFile;
    }

    vertex = dist_vertex;

  } while (code == ReturnCode::kCodeOk);

  if (code == ReturnCode::kCodeOk)
    for (auto i : edges) {
      if (graph.FindVertex(i.vertex_) == graph.GetVertexes().end())
        graph.GetVertexes().insert({i.vertex_, Graph::Properties()});
      graph.GetEdges().push_back(i);
    }
  if (graph.FindVertex(edges.back().dist_vertex_) == graph.GetVertexes().end())
    graph.GetVertexes().insert(
        {edges.back().dist_vertex_, Graph::Properties()});

  return code;
}

Importer::ReturnCode Importer::NextPropertiesList(
    Parser& parser, Graph::Properties& properties) const {
  TokenParser::Token term =
      TokenParser::Token(TokenParser::Token::id_type(kTokenComma));

  while (term.IsId() && term.GetId() == kTokenComma) {
    std::string word = parser.NextWord();
    TokenParser::Token eq = parser.NextThisId(Token::kTokenEqSign);
    std::string value = parser.NextWord();
    term = parser.NextId();

    if (word.empty() || eq.IsNull()) return ReturnCode::kCodeInvalidFile;

    properties.insert({word, value});
  }

  if (!term.IsId() || term.GetId() != Token::kTokenClostSquareBracket)
    return ReturnCode::kCodeInvalidFile;
  return ReturnCode::kCodeOk;
}

}  // namespace GraphDot

}  // namespace graph
