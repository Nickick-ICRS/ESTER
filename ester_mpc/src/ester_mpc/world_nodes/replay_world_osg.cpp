#include "ester_mpc/world_nodes/replay_world_osg.hpp"

namespace ester_mpc {

ReplayWorldOSG::ReplayWorldOSG(const dart::simulation::WorldPtr &world, const std::shared_ptr<History> &hist)
    :dart::gui::osg::RealTimeWorldNode(world), world_(world), history_(hist), frame_(0)
{

}

void ReplayWorldOSG::customPostStep() {
    moveToNextFrame();
}

void ReplayWorldOSG::moveToNextFrame() {
    if (frame_ == history_->size()) {
        throw simulation_finished();
    }
    auto state = history_->at(frame_++);
    auto robot = world_->getSkeleton("ester");
    robot->setConfiguration(state.cfg);
    for (size_t i = 0; i < robot->getNumBodyNodes(); i++) {
        robot->getBodyNode(i)->setCompositeState(state.aspect_state[i]);
    }
}

} // namespace ester_mpc