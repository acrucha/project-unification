#include "CustomPlayer.h"
#include "Graph/Graph.h"

CustomPlayer::CustomPlayer(int index, QThreadPool* threadPool) : Processing(index, threadPool) {
}

void CustomPlayer::buildParameters(Parameters::Handler& parameters) {
}

void CustomPlayer::connectModules(const Modules* modules) {
  connect(modules->vision(),
          &Vision::sendFrame,
          this,
          &CustomPlayer::receiveFrame,
          Qt::DirectConnection);

  connect(modules->vision(),
          &Vision::sendField,
          this,
          &CustomPlayer::receiveField,
          Qt::DirectConnection);
}

void CustomPlayer::init(const Modules* modules) {
  this->map = false;
}

void CustomPlayer::update() {
  shared->field.extract_to(field);
  if (auto f = shared->frame.get_optional_and_reset()) {
    if (auto it = f->allies().findById(index()); it != f->allies().end()) {
      robot = *it;
    }
    frame.emplace(*f);
  }
}

void CustomPlayer::exec() {
  if (!field || !frame || !robot) {
    return;
  }

  // qInfo() << "to pegando ein!" << Qt::endl;
  if (!this->map) {
    Graph g = Graph();

    QList<QPoint> q =
        g.generateBestPath(frame->enemies(), robot->position(), frame->ball().position());

    qInfo() << robot->position() << " to " << frame->ball().position();
    qInfo() << frame->enemies();
    qInfo() << "MELHOR CAMINHO: " << q;

    this->map = true;
  }

  // TODO: here...
  // emit sendCommand(...);
}

void CustomPlayer::receiveField(const Field& field) {
  shared->field = field;
}

void CustomPlayer::receiveFrame(const Frame& frame) {
  shared->frame = frame;
  runInParallel();
}

static_block {
  Factory::processing.insert<CustomPlayer>();
};
