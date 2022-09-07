#ifndef PROJECT_UNIFICATION_DIJKTRA_H
#define PROJECT_UNIFICATION_DIJKTRA_H

#include "Modules/Processing/ProcessingUtils/ProcessingUtils.h"

class Dijkstra {
 private:
  QHash<QPoint, double> distances;
  QHash<QPoint, QPoint> pred;

  const int INF = 1000;
  QPoint initial;
  QPoint target;
  QHash<QPoint, QHash<QPoint, double>> graph;

 public:
  Dijkstra(QPoint i, QHash<QPoint, QHash<QPoint, double>> g, QPoint t);
  QList<QPoint> generatePath();
  QList<QPoint> bestPath();
};

#endif // PROJECT_UNIFICATION_DIJKTRA_H
