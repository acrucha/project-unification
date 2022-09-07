#include "Dijkstra.h"
#include <bits/stdc++.h>

using namespace std;

typedef pair<double, pair<int, int>> nodePair;

// bool static compare(const QPair<double, QPoint>& node, const QPair<double, QPoint>& other) {

//   if (node.first != other.first) {
//     return (node.first < other.first);
//   } else {
//     if (node.second.x() == other.second.x()) {
//       return (node.second.y() > other.second.y());
//     } else {
//       return (node.second.x() < other.second.x());
//     }
//   }

//   return true;
// }

Dijkstra::Dijkstra(QPoint i, QHash<QPoint, QHash<QPoint, double>> g, QPoint t) {
  initial = i;
  graph = g;
  target = t;
  distances = QHash<QPoint, double>();
  pred = QHash<QPoint, QPoint>();
}

QList<QPoint> Dijkstra::generatePath() {
  QList<QPoint> path;
  auto p = target;

  while (p != initial) {
    path.append(p);
    // qInfo() << p;
    p = pred[p];
  }

  path.append(p);

  reverse(path.begin(), path.end());

  return path;
}

pair<int, int> toPair(QPoint p) {
  return pair<int, int>(p.x(), p.y());
}

QPoint toPoint(pair<int, int> p) {
  return QPoint(p.first, p.second);
}

QList<QPoint> Dijkstra::bestPath() {
  QList<QPoint> path;

  priority_queue<nodePair, vector<nodePair>, greater<nodePair>> pq;

  auto init = toPair(initial);
  distances[initial] = 0;
  pq.push({distances[initial], init});

  while (!pq.empty()) {
    auto [dist, node] = pq.top();
    pq.pop(); // tirando da fila
    auto nodePoint = toPoint(node);
    if (dist != distances[nodePoint])
      continue;
    // qInfo() << "nodePoint -> " << nodePoint << "/ ns:";
    auto n = graph[nodePoint].constBegin();
    while (n != graph[nodePoint].constEnd()) {
      double newDist = dist + n.value();
      if (!distances.contains(n.key()) || distances[n.key()] > newDist) {
        distances[n.key()] = newDist;
        pq.push({newDist, toPair(n.key())});
        pred[n.key()] = nodePoint;
        // qInfo() << "n -> " << n.key();
      }
      ++n;
    }
  }

  path = generatePath();

  return path;
}