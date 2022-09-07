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
    p = pred[p];
  }

  path.append(p);

  reverse(path.begin(), path.end());

  return path;
}

QList<QPoint> Dijkstra::bestPath() {
  QList<QPoint> path;

  priority_queue<nodePair, vector<nodePair>, greater<nodePair>> pq;

  distances[initial] = 0;
  pq.push({distances[initial], initial});

  while (!pq.empty()) {
    auto [dist, node] = pq.top();
    pq.pop(); // tirando da fila

    if (dist != distances[node])
      continue;

    auto n = graph[node].constBegin();
    while (n != graph[node].constEnd()) {
      auto newDist = dist + n.value();
      if (!distances.contains(n.key()) || distances[n.key()] > newDist) {
        distances[n.key()] = newDist;
        pq.push({newDist, n.key()});
        pred[n.key()] = node;
      }
      ++n;
    }
  }

  path = generatePath();

  return path;
}