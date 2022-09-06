#ifndef PROJECT_UNIFICATION_GRAPH_H
#define PROJECT_UNIFICATION_GRAPH_H

#include "Modules/Processing/ProcessingUtils/ProcessingUtils.h"

// std::size_t qHash(const QPointF& point, std::size_t seed = 0) {
//   return qHashMulti(seed, point.x(), point.y());
// }

class Graph {

 private:
  [[nodiscard]] std::vector<int> defineBoundaries(const QPointF& point) const;
  void mapObstacles(const Robots<Robot>& enemies);
  void createNodes(const QPointF& origin, const QPointF& target);
  void createGraph(const QPointF& origin, const QPointF& target);

  QHash<QPoint, bool> obstacles;
  QHash<QPoint, QList<QPoint>> graph;
  const int OBSTACLE_BOUNDARIES = 7;

 public:
  Graph();

  QList<QPointF>
  generateBestPath(const Robots<Robot>& enemies, const QPointF& origin, const QPointF& target);
};

#endif // PROJECT_UNIFICATION_GRAPH_H
