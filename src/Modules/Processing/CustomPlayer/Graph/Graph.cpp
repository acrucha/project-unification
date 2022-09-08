#include "Graph.h"

Graph::Graph() {
  initial = QPair<QPoint, double>(QPoint(0, 0), INF);
}

void Graph::mapObstacles(const Robots<Robot>& enemies) {
  this->obstaclesArea = QList<QRect>();
  for (auto& e : enemies) {
    QPointF p = e.position();
    std::vector<int> v = defineBoundaries(p);
    QRect area(QPoint(v[0], v[1]), QPoint(v[2], v[3]));
    qInfo() << p << " - Area: " << area;
    this->obstaclesArea.append(area);
    for (int i = v[0]; i < v[2]; i++) {
      for (int j = v[1]; j > v[3]; j--) {
        QPoint point(i, j);
        this->obstacles[point] = true;
      }
    }
  }
}

void Graph::createNodes(const QPointF& origin, const QPointF& target) {
  int difX = qCeil(qFabs(origin.x() - target.x())) - TARGET_MARGIN;

  QPoint node(qCeil(origin.x()), qCeil(origin.y()));
  int ix = node.x();
  int interval = difX / (INTERVAL_NUMBER - 1);
  while (difX > 0 && ix <= X_MAX) {
    for (int iy = -Y_MAX; iy <= Y_MAX; iy += Y_MAX / 2) {
      node.ry() = iy;
      if (!obstacles.contains(node)) {
        graph[node] = QHash<QPoint, double>();
      }
    }
    ix += interval;
    node.rx() = ix;
    difX -= interval;
  }
}

double Graph::distance(QPoint node, QPoint other) {
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
      return diagonalPolygon(node, other, 1);
    } else if (node.y() > other.y()) { // diagonal bottom
      return diagonalPolygon(node, other, -1);
    } else { // same y
      return QPolygon(QRect(node.x(), node.y() - margin, NEIGHBOUR_AREA, RECT_HEIGHT));
    }
  } else if (node.y() < other.y()) { // other is above
    return QPolygon(QRect(node.x() - margin, node.y(), RECT_HEIGHT, NEIGHBOUR_AREA));
  } else { // other is bellow
    return QPolygon(QRect(other.x() - margin, other.y(), RECT_HEIGHT, NEIGHBOUR_AREA));
  }
  return polygon;
}

bool Graph::thereIsAnObstacle(QPoint node, QPoint other) {
  QPolygon nodeArea = createArea(node, other);

  // qInfo() << node << " e " << other;
  // qInfo() << "polygon = " << nodeArea;

  for (auto o : obstaclesArea) {
    if (nodeArea.intersects(QPolygon(o))) {
      // qInfo() << node << " e " << other << "!!tem um inimigo" << o.topLeft() << Qt::endl;
      return true;
    }
  }
  return false;
}

bool Graph::isEqual(QPoint node, QPoint other) {
  return (node.x() == other.x() && node.y() == other.y());
}

bool Graph::isNeighbour(QPoint node, QPoint other) {

  int difX = qAbs(node.x() - other.x());
  int difY = qAbs(node.y() - other.y());

  if (node.x() <= other.x() && difX <= NEIGHBOUR_AREA && difY <= NEIGHBOUR_AREA) {
    if (isEqual(node, other) || thereIsAnObstacle(node, other)) {
      return false;
    }
    return true;
  }

  return false;
}

void Graph::printGraph() const {
  auto node = graph.constBegin();
  while (node != graph.constEnd()) {
    qInfo() << node.key() << " -> " << node.value() << Qt::endl;
    ++node;
  }
}

void Graph::setInitial(QPoint node) {
  if (isNeighbour(robot, node)) {
    double dist = distance(node, robot);
    if (dist < initial.second) {
      initial.first = node;
      initial.second = dist;
    }
  }
}

void Graph::createEdges() {

  // create links between all nodes that are neighbours
  auto node = graph.constBegin();
  while (node != graph.constEnd()) {
    if (isNeighbour(node.key(), ball)) {
      graph[node.key()][ball] = distance(node.key(), ball);
    }
    setInitial(node.key()); // ver isso
    auto other = graph.constBegin();
    while (other != graph.constEnd()) {
      if (isNeighbour(node.key(), other.key())) {
        graph[node.key()][other.key()] = distance(node.key(), other.key());
      }
      ++other;
    }
    ++node;
  }
}

void Graph::createGraph(const QPointF& origin, const QPointF& target) {

  createNodes(origin, target);

  createEdges();

  // printGraph();
}

std::vector<int> Graph::defineBoundaries(const QPointF& point) const {
  int initialX = qFloor(point.x() - OBSTACLE_BOUNDARIES);
  int initialY = qFloor(point.y() + OBSTACLE_BOUNDARIES);
  int endX = qFloor(point.x() + OBSTACLE_BOUNDARIES);
  int endY = qFloor(point.y() - OBSTACLE_BOUNDARIES);

  std::vector<int> v = {initialX, initialY, endX, endY};

  return v;
}

QList<QPoint> Graph::generateBestPath(const Robots<Robot>& enemies,
                                      const QPointF& origin,
                                      const QPointF& target) {
  robot = QPoint(qCeil(origin.x()), qCeil(origin.y()));
  ball = QPoint(qCeil(target.x()), qCeil(target.y()));
  mapObstacles(enemies);
  createGraph(origin, target);
  Dijkstra dijkstra(initial.first, graph, ball);

  QList<QPoint> list;
  list = dijkstra.bestPath();

  return list;
}
