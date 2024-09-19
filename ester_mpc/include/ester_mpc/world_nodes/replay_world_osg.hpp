#ifndef __REPLAY_WORLD_OSG_HPP__
#define __REPLAY_WORLD_OSG_HPP__

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>

namespace ester_mpc
{

class simulation_finished : public std::exception {
};

struct State {
    dart::dynamics::Skeleton::Configuration cfg;
    std::vector<dart::common::Composite::State> aspect_state;
};
using History = std::vector<State>;

class ReplayWorldOSG : public dart::gui::osg::RealTimeWorldNode
{
public:
    ReplayWorldOSG(const dart::simulation::WorldPtr &world, const std::shared_ptr<History> &hist);
    virtual ~ReplayWorldOSG() = default;

    void customPostStep() override;

    void moveToNextFrame();

private:
    dart::simulation::WorldPtr world_;
    std::shared_ptr<History> history_;
    size_t frame_;
};

} // namespace ester_mpc

#endif // __REPLAY_WORLD_OSG_HPP__