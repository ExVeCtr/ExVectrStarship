#include "starship_sim_task.hpp"

namespace VCTR {
namespace ShipSim {

StarshipSimTask::StarshipSimTask(float mass, Math::Vector<float, 3> inertiaTensor, float tvcThrustLimit_N,
                                 float tvcAngleLimit_Rad, int64_t simulationInterval)
    : Core::Task_Periodic("Simlation task", simulationInterval),
      adapter_(mass, inertiaTensor, tvcThrustLimit_N, tvcAngleLimit_Rad) {
  Core::getSystemScheduler().addTask(*this);
}

StarshipSimTask::~StarshipSimTask() = default;

void StarshipSimTask::setTVCInputTopic(Core::Topic<Math::Vector<float, 4>> &tvcTopic,
                                       const Math::Vector<float, 3> &tvcPosition, float tvcAngleLimit,
                                       float tvcForceLimit) {
  adapter_.configureTvc(tvcAngleLimit, tvcForceLimit, tvcPosition(2));
  tvcSubr_.subscribe(tvcTopic);
}

void StarshipSimTask::update() {
  const int64_t now = Core::NowNs();
  if (tvcSubr_.isDataNew())
    adapter_.setTvcInput(tvcSubr_.getItem());
  if (flapSettingsSubr_.isDataNew()) {
    const auto f = flapSettingsSubr_.getItem();
    adapter_.setFlapInput(f.flapTLAngle_Rad, f.flapTRAngle_Rad, f.flapBLAngle_Rad, f.flapBRAngle_Rad);
  }
  adapter_.update(now);
}

void StarshipSimTask::taskInit() { adapter_.start(Core::NowNs()); }

void StarshipSimTask::taskThread() {
  update();

  const int64_t ts = adapter_.stateTimestamp();
  const Math::Vector<float, 7> &att = adapter_.getAttitudeState();
  const Math::Vector<float, 6> &pos = adapter_.getPositionState();
  Core::Timestamped<Math::Vector<float, 7>> attState;
  attState.timestamp = ts;
  attState.data = att;
  Core::Timestamped<Math::Vector<float, 6>> posState;
  posState.timestamp = ts;
  posState.data = pos;
  Core::Timestamped<Math::Vector<float, 2>> groundState;
  groundState.timestamp = ts;
  groundState.data = {pos(2), pos(5)}; // [vertical velocity, height above the ground]
  attitudeTopic_.publish(attState);
  positionTopic_.publish(posState);
  groundTopic_.publish(groundState);
}

} // namespace ShipSim
} // namespace VCTR
