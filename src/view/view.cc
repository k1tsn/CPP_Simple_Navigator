#include "view.h"

#include <cstdint>
#include <iostream>

#include "matrix.h"

namespace graph_cb {

View::View(Controller* controller) {
  controller_ = controller;
  is_end_ = true;
}

View::~View() {}

void View::Show() {
  while (is_end_) {
    int action = ShowCallToAction();
    switch (action) {
      case TaskType::kLoadFile:
        LoadFile();
        break;

      case TaskType::kDepthSearch:
        DepthFirstSearch();
        break;

      case TaskType::kBreadthSearch:
        BreadthFirstSearch();
        break;

      case TaskType::kSearchBetweenTwoPairs:
        GetShortestPathBetweenVertices();
        break;

      case TaskType::kSearchAllPairs:
        GetShortestPathsBetweenAllVertices();
        break;

      case TaskType::kLeastSpanningTree:
        GetLeastSpanningTree();
        break;

      case TaskType::kSolveSalesman:
        SolveTravelingSalesmanProblem();
        break;

      case TaskType::kEnd:
        is_end_ = false;
        break;

      default:
        std::cout << "Неизвестное действие, попробуйте снова." << std::endl;
        break;
    }
  }
}

int View::ShowCallToAction() {
  std::string str_action;

  std::cout << std::endl << "Что нужно сделать?" << std::endl << std::endl;

  std::cout << "----------------------------------------";
  std::cout << "------------------------------------" << std::endl;

  std::cout << "1. Загрузить исходный граф из файла." << std::endl;
  std::cout << "2. Выполнить обход графа в глубину." << std::endl;
  std::cout << "3. Выполнить обход графа в ширину." << std::endl;

  std::cout << "4. Выполнить поиск кратчайшего пути между ";
  std::cout << "произвольными двумя вершинами." << std::endl;

  std::cout << "5. Выполнить поиск кратчайших путей между всеми ";
  std::cout << "парами вершин в графе." << std::endl;

  std::cout << "6. Выполнить поиск минимального остовного ";
  std::cout << "дерева в графе." << std::endl;

  std::cout << "7. Решить задачу коммивояжера." << std::endl;

  std::cout << "8. Завершить работу." << std::endl;

  std::cout << "----------------------------------------";
  std::cout << "------------------------------------" << std::endl;

  std::cout << std::endl;

  std::cin >> str_action;

  std::cout << "" << std::endl;

  size_t index;
  int action;
  try {
    action = std::stoi(str_action, &index);
  } catch (...) {
    action = -1;
  }
  if (index != str_action.length()) action = -1;

  return action;
}

void View::LoadFile() {
  std::string fileway;
  std::cout << "Укажите путь до файла: ";
  std::cin >> fileway;
  std::cout << std::endl;

  Controller::ReturnCode code = controller_->LoadGraphFromFile(fileway);
  switch (code) {
    case Controller::ReturnCode::kCodeFileDontOpen:
      std::cout << "Файл не открыт." << std::endl;
      break;

    case Controller::ReturnCode::kCodeInvalidFile:
      std::cout << "Файл поврежден." << std::endl;
      break;

    case Controller::ReturnCode::kCodeOk:
      std::cout << "Файл загружен." << std::endl;
      break;

    default:
      std::cout << "Неизвестная ошибка." << std::endl;
      break;
  }
}

void View::DepthFirstSearch() {
  std::cout << "Введите стартовую вершину: ";
  int start_vertex;
  std::cin >> start_vertex;
  std::vector<Vertex> vertex = controller_->DepthFirstSearch(start_vertex);

  if (vertex.empty()) {
    std::cout << "Вершины нет в графе или граф не был загружен." << std::endl;
    return;
  }

  std::cout << "Обход в глубину:" << std::endl;

  for (size_t i = 0; i < vertex.size(); ++i) {
    std::cout << vertex[i] << " ";
  }

  std::cout << std::endl;
}

void View::BreadthFirstSearch() {
  std::cout << "Введите стартовую вершину: ";
  int start_vertex;
  std::cin >> start_vertex;
  std::vector<Vertex> vertex = controller_->BreadthFirstSearch(start_vertex);

  if (vertex.empty()) {
    std::cout << "Вершины нет в графе или граф не был загружен." << std::endl;
    return;
  }

  std::cout << "Обход в ширину:" << std::endl;

  for (size_t i = 0; i < vertex.size(); ++i) {
    std::cout << vertex[i] << " ";
  }

  std::cout << std::endl;
}

void View::GetShortestPathBetweenVertices() {
  std::cout << "Введите две вершины: " << std::endl;
  int vertex1, vertex2;
  std::cin >> vertex1 >> vertex2;
  WayBetweenTwo way =
      controller_->GetShortestPathBetweenVertices(vertex1, vertex2);
  if (way.distance < 0)
    std::cout << "Граф не загружен или точки не найдены в графе" << std::endl;
  else {
    std::cout << "Длина кратчайщего пути: " << way.distance << std::endl;
    std::cout << "Путь: ";
    for (size_t i = 0; i < way.vertices.size(); ++i)
      std::cout << way.vertices[i] << " ";
    std::cout << std::endl;
  }

  std::cout << std::endl;
}

void View::GetShortestPathsBetweenAllVertices() {
  mtlc::Matrix<Controller::EdgeWeightType> adjacency_matrix =
      controller_->GetShortestPathsBetweenAllVertices();
  std::cout << "Кратчайшие пути между всеми вершинами:" << std::endl;
  PrintMatrix(adjacency_matrix);
}

void View::GetLeastSpanningTree() {
  mtlc::Matrix<Controller::EdgeWeightType> spanning_tree =
      controller_->GetLeastSpanningTree();
  std::cout << "Минимальное остовное дерево:" << std::endl;
  PrintMatrix(spanning_tree);
}

void View::SolveTravelingSalesmanProblem() {
  std::cout << "Выберите алгоритм:" << std::endl;
  std::cout << "1. Муравьиный алгоритм." << std::endl;
  int choise;
  SalesmanType type;
  std::cin >> choise;
  switch (choise - 1) {
    case SalesmanType::kAntAlgorithm:
      type = SalesmanType::kAntAlgorithm;
      break;

    default:
      std::cout << "Неизвестная цифра. Попробуйте снова." << std::endl;
      return;
  }

  TsmResult res = controller_->SolveTravelingSalesmanProblem(type);
  std::cout << "Расстояние: " << res.distance << std::endl;
  for (size_t i = 0; i < res.vertices.size(); ++i) {
    std::cout << res.vertices[i] << " ";
  }
  std::cout << std::endl;
}

void View::CompareSalesmanProblem() {
  std::cout << "Сколько раз нужно решить задачу? Введите число: ";
  int count;
  std::cin >> count;
}

void View::PrintMatrix(const mtlc::Matrix<Controller::EdgeWeightType>& matrix) {
  for (size_t i = 0; i < matrix.GetRows(); ++i) {
    for (size_t j = 0; j < matrix.GetRows(); ++j) {
      std::cout << matrix(i, j) << " ";
    }
    std::cout << std::endl;
  }
}

}  // namespace graph_cb
