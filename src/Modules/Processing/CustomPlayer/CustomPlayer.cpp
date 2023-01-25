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
  this->stopRobot = false;
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

  if (!this->map) {
    Graph g = Graph();

    path = g.generateBestPath(frame->enemies(), robot->position(), frame->ball().position());

    qInfo() << "MELHOR CAMINHO: " << path;

    currentPoint = path.first();
    path.removeFirst();

    this->map = true;
  }
  if (!path.isEmpty() && this->stopRobot == false) {
    VSSMotion::GoToPoint goToPoint(currentPoint);
    currentPoint = path.first();
    QPointF p = robot->position();
    QRectF area(QPointF(p.x() - BOUND, p.y() + BOUND), QPointF(p.x() + BOUND, p.y() - BOUND));
    if (area.contains(currentPoint)) {
      path.removeFirst();
    }
    auto command = vssNavigation.run(robot.value(), VSSRobotCommand(goToPoint));
    emit sendCommand(command);
  } else {
    this->stopRobot = true;
  }

  if (this->stopRobot) {
    VSSMotion::Stop stop;
    auto command = vssNavigation.run(robot.value(), VSSRobotCommand(stop));
    emit sendCommand(command);
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
