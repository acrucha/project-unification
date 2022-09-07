#include "Graph.h"

Graph::Graph() = default;

void Graph::mapObstacles(const Robots<Robot>& enemies) {
  this->obstaclesArea = QList<QRect>();
  for (auto& e : enemies) {
    QPointF p = e.position();
    std::vector<int> v = defineBoundaries(p);
    QRect area(v[0], v[1], OBSTACLE_BOUNDARIES * 2, OBSTACLE_BOUNDARIES * 2);
    qInfo() << "inimigo => " << p << " - area => " << area << Qt::endl;
    this->obstaclesArea.append(area);
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

double Graph::dist(QPoint node, QPoint other) {
  return qSqrt(qPow(other.x() - node.x(), 2) + qPow(other.y() - node.y(), 2));
}

QPolygon Graph::diagonalPolygon(QPoint node, QPoint other, int flag) const {
  int margin = (RECT_HEIGHT / 2);
  QPolygon p;
  p << QPoint(node.x() - margin, node.y() + flag * margin)
    << QPoint(node.x() + margin, node.y() - flag * margin)
    << QPoint(other.x() - margin, other.y() + flag * margin)
    << QPoint(other.x() + margin, other.y() - flag * margin);
  return p;
}

QPolygon Graph::createArea(QPoint node, QPoint other) {
  int margin = (RECT_HEIGHT / 2);
  QPolygon polygon;
  if (node.x() < other.x()) {   // nodes on right side
    if (node.y() < other.y()) { // diagonal top -> flag = 1
      qInfo() << "DIAGONAL TOP" << Qt::endl;
      return diagonalPolygon(node, other, 1);
    } else if (node.y() > other.y()) { // diagonal bottom
      qInfo() << "DIAGONAL BOTTOM" << Qt::endl;
      return diagonalPolygon(node, other, -1);
    } else {
      qInfo() << "NA DIREITA" << Qt::endl;
      qInfo() << node << Qt::endl;
      return QPolygon(QRect(node.x(), node.y() - margin, NEIGHBOUR_AREA, RECT_HEIGHT));
    }
  } else if (node.y() > other.y()) {
    qInfo() << "EM CIMA" << Qt::endl;
    return QPolygon(QRect(node.x() - margin, node.y(), RECT_HEIGHT, NEIGHBOUR_AREA));
  } else {
    qInfo() << "EMBAIXO" << Qt::endl;
    return QPolygon(QRect(other.x() - margin, other.y(), RECT_HEIGHT, NEIGHBOUR_AREA));
  }
  return polygon;
}

bool Graph::thereIsAnObstacle(QPoint node, QPoint other) {
  QPolygon nodeArea = createArea(node, other);

  qInfo() << "|-> polygon = " << nodeArea << Qt::endl;

  for (auto o : obstaclesArea) {
    if (nodeArea.intersects(QPolygon(o))) {
      qInfo() << node << " e " << other << "!!tem um inimigo" << o.topLeft() << Qt::endl;
      return false;
    }
  }
  return true;
}

bool Graph::isNeighbour(QPoint node, QPoint other) {

  int difX = qAbs(node.x() - other.x());
  int difY = qAbs(node.y() - other.y());

  if (node.x() <= other.x() && difX <= NEIGHBOUR_AREA && difY <= NEIGHBOUR_AREA) {
    if (node == other || thereIsAnObstacle(node, other)) {
      return false;
    }
    return true;
  }

  return false;
}

void Graph::createEdges() {
  // create links between all nodes that are neighbours
  auto node = graph.constBegin();
  while (node != graph.constEnd()) {
    auto other = graph.constBegin();
    while (other != graph.constEnd()) {
      // qInfo() << other.key() << " -> " << other.value() << Qt::endl;
      if (isNeighbour(node.key(), other.key())) {
        graph[node.key()].append(other.key());
      }
      ++other;
    }
    ++node;
  }
  node = graph.constBegin();
  while (node != graph.constEnd()) {
    qInfo() << node.key() << " -> " << node.value() << Qt::endl;
    ++node;
  }
}

// bool isGreater(QHash<QPoint, QList<QPoint>>::iterator node,
//                QHash<QPoint, QList<QPoint>>::iterator other) {
//   if (node.key().x() > other.key().x() && node.key().y() < other.key().y()) {
//     return true;
//   } else if (node.key().x() == other.key().x()) {
//     return (node.key().y() < other.key().y());
//   } else if (node.key().y() == other.key().y()) {
//     return node.key().x() > other.key().x();
//   }
//   return false;
// }

void Graph::createGraph(const QPointF& origin, const QPointF& target) {

  createNodes(origin, target);

  createEdges();

  qInfo() << "robot position:" << origin << Qt::endl;
  qInfo() << "target position:" << target << Qt::endl;

  qInfo() << "graph:" << graph << Qt::endl;
}

std::vector<int> Graph::defineBoundaries(const QPointF& point) const {
  int initialX = qFloor(point.x() - OBSTACLE_BOUNDARIES);
  int initialY = qFloor(point.y() + OBSTACLE_BOUNDARIES);
  int endX = qFloor(point.x() + OBSTACLE_BOUNDARIES);
  int endY = qFloor(point.y() - OBSTACLE_BOUNDARIES);

  std::vector<int> v = {initialX, initialY, endX, endY};

  return v;
}

QList<QPointF> Graph::generateBestPath(const Robots<Robot>& enemies,
                                       const QPointF& origin,
                                       const QPointF& target) {
  qInfo() << "enemies:" << enemies << Qt::endl;
  QList<QPointF> list;
  mapObstacles(enemies);
  createGraph(origin, target);
  return list;
}
