#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../../../src/model/s21_graph/s21_graph.h"
#include "../../../src/model/s21_graph_algorithms/s21_graph_algorithms.h"
#include "matrix.h"

struct SolveSalesmanDataTest {
  std::string filename_;
  int path_;
  s21::GraphAlgorithms::SalesmanAlgorithms type_;
};

class SolveSalesmanTest : public testing::TestWithParam<int> {
 public:
  static int count_;
  static std::vector<SolveSalesmanDataTest> test_data_;
};

int SolveSalesmanTest::count_ = 3;
std::vector<SolveSalesmanDataTest> SolveSalesmanTest::test_data_ = {
    SolveSalesmanDataTest{
        "graphs/10_salesman.txt", 49,
        s21::GraphAlgorithms::SalesmanAlgorithms::kAntAlgorithm},
    SolveSalesmanDataTest{
        "graphs/11_salesman.txt", 45,
        s21::GraphAlgorithms::SalesmanAlgorithms::kAntAlgorithm},
    SolveSalesmanDataTest{
        "graphs/12.txt", 24,
        s21::GraphAlgorithms::SalesmanAlgorithms::kAntAlgorithm}};

INSTANTIATE_TEST_SUITE_P(solve_salesman, SolveSalesmanTest,
                         testing::Range(0, SolveSalesmanTest::count_));

TEST_P(SolveSalesmanTest, Common) {
  int num_test = this->GetParam();
  SolveSalesmanDataTest& data = SolveSalesmanTest::test_data_[num_test];

  s21::Graph graph;
  graph.LoadGraphFromFile(data.filename_);

  s21::GraphAlgorithms alg;

  s21::GraphAlgorithms::TsmResult res =
      alg.SolveTravelingSalesmanProblem(graph, data.type_);

  EXPECT_EQ(res.distance, data.path_);
}
