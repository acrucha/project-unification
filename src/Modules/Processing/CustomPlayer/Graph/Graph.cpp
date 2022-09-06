#include "Graph.h"

Graph::Graph() = default;

void Graph::mapObstacles(const Robots<Robot>& enemies) {
  for (auto& e : enemies) {
    QPointF p = e.position();
    std::vector<int> v = defineBoundaries(p);
    for (int i = v[0]; i < v[2]; i++) {
      for (int j = v[1]; j > v[3]; j--) {
        QPoint point(i, j);
        // qInfo() << point << Qt::endl;
        this->obstacles[point] = true;
      }
    }
  }
}

void Graph::createGraph(const QPointF& origin, const QPointF& target) {
}

std::vector<int> Graph::defineBoundaries(const QPointF& point) const {
  int initialX = qCeil(point.x() - OBSTACLE_BOUNDARIES);
  int initialY = qCeil(point.y() + OBSTACLE_BOUNDARIES);
  int endX = qCeil(point.x() + OBSTACLE_BOUNDARIES);
  int endY = qCeil(point.y() - OBSTACLE_BOUNDARIES);

  std::vector<int> v = {initialX, initialY, endX, endY};

  return v;
}

QList<QPointF> Graph::generateBestPath(const Robots<Robot>& enemies,
                                       const QPointF& origin,
                                       const QPointF& target) {

  QList<QPointF> list;
  mapObstacles(enemies);
  return list;
}
