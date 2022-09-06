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

void Graph::createNodes(const QPointF& origin, const QPointF& target) {
  int difX = qCeil(qFabs(origin.x() - target.x())) - 10;

  qInfo() << "DifX = " << difX << Qt::endl;

  QPoint node(qCeil(origin.x()), qCeil(origin.y()));
  int ix = node.x();
  int interval = difX / 3;
  while (difX > 0) {
    for (int iy = -60; iy <= 60; iy += 30) {
      node.ry() = iy;
      if (!obstacles.contains(node)) {
        graph[node] = QList<QPoint>();
      }
    }
    ix += interval;
    node.rx() = ix;
    difX -= interval;
  }
}

void Graph::createGraph(const QPointF& origin, const QPointF& target) {

  createNodes(origin, target);

  qInfo() << "graph:" << graph << Qt::endl;
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
  createGraph(origin, target);
  return list;
}
