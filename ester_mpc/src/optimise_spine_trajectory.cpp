#include "ester_mpc/world_nodes/spine_trajectory_optimise_world.hpp"

#include <thread>
#include <mutex>

using namespace ester_mpc;
using namespace ester_common;

// Helper function to increment indices
bool increment(std::vector<int>& indices, const std::vector<int>& sizes) {
    for (int i = indices.size() - 1; i >= 0; --i) {
        if (++indices[i] < sizes[i]) {
            return true;
        }
        indices[i] = 0;
    }
    return false;
}

// Function to generate Cartesian product
std::vector<Eigen::VectorXd> cartesianProduct(const std::vector<std::vector<double>>& vectors) {
    std::vector<Eigen::VectorXd> result;
    std::vector<int> indices(vectors.size(), 0);
    std::vector<int> sizes;

    size_t total_size = 1;
    for (const auto& vec : vectors) {
        sizes.push_back(vec.size());
        total_size *= vec.size();
    }
    result.reserve(total_size);

    do {
        Eigen::VectorXd combination;
        combination.resize(indices.size());
        for (int i = 0; i < indices.size(); ++i) {
            combination(i) = vectors[i][indices[i]];
        }
        result.push_back(combination);
    } while (increment(indices, sizes));

    return result;
}

class EventHandler : public osgGA::GUIEventHandler {
public:
    EventHandler(const osg::ref_ptr<dart::gui::osg::WorldNode> &node) {
        node_ = node;
    };

    bool handle(const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter&) override {
        return false;
    }

private:
    osg::ref_ptr<dart::gui::osg::WorldNode> node_;
};

std::shared_ptr<SpineTrajectoryOptimiseWorld> make_optimiser_fixed(
    const dart::simulation::WorldPtr &world)
{
    SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> stgs;
    stgs[SpineJointId::FRONT_PITCH] = std::make_shared<SpineTrajectoryGenerator>();
    stgs[SpineJointId::FRONT_YAW] = std::make_shared<SpineTrajectoryGenerator>();
    stgs[SpineJointId::REAR_PITCH] = std::make_shared<SpineTrajectoryGenerator>();
    stgs[SpineJointId::REAR_YAW] = std::make_shared<SpineTrajectoryGenerator>();

    std::vector<Eigen::VectorXd> params;
    params.push_back(Eigen::VectorXd());
    return std::make_shared<SpineTrajectoryOptimiseWorld>(world, stgs, params);
}

std::shared_ptr<SpineTrajectoryOptimiseWorld> make_optimiser_stiffness(
    const dart::simulation::WorldPtr &world, const std::vector<Eigen::VectorXd> &params)
{
    SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> stgs;
    stgs[SpineJointId::FRONT_PITCH] = std::make_shared<SpineStiffnessTrajectoryGenerator>();
    stgs[SpineJointId::FRONT_YAW] = std::make_shared<SpineStiffnessTrajectoryGenerator>();
    stgs[SpineJointId::REAR_PITCH] = std::make_shared<SpineStiffnessTrajectoryGenerator>();
    stgs[SpineJointId::REAR_YAW] = std::make_shared<SpineStiffnessTrajectoryGenerator>();

    return std::make_shared<SpineTrajectoryOptimiseWorld>(world, stgs, params);
}

void generate_params_stiffness(std::vector<Eigen::VectorXd> &out_params) {
    // stiffness = A * sin(2*pi*phase + B) + C, damping = D
    // Mirror params for front and back joints
    std::vector<double> A_p {2, 4, 8};
    std::vector<double> B_p {M_PI_4 / 4, M_PI_4 / 2, 3 * M_PI_4 / 4};
    std::vector<double> C_p {2, 4, 8};
    std::vector<double> D_p {0.6};
    std::vector<double> A_y {2, 4};
    std::vector<double> B_y {M_PI};
    std::vector<double> C_y {2, 3, 4};
    std::vector<double> D_y {0.8, 1.0, 1.3, 1.6};

    Eigen::Matrix<double, 16, 1> params;
    out_params.clear();
    double total_perms = A_p.size() * B_p.size() * C_p.size() * D_p.size()
        * A_y.size() * B_y.size() * C_y.size() * D_y.size();
    out_params.reserve(total_perms);
    std::vector<std::vector<double>> perms {A_p, B_p, C_p, D_p, A_y, B_y, C_y, D_y};
    auto perm_params = cartesianProduct(perms);
    for (auto &p : perm_params) {
        Eigen::Matrix<double, 16, 1> param;
        param.head<8>() = p;
        param.tail<8>() = param.head<8>();
        // Out of phase by PI for front and back
        param(9) += M_PI;
        param(13) += M_PI;
        // Rear pitch needs to be stronger coz of weight imbalance
        param(8) *= 2;
        param(10) *= 2;
        out_params.push_back(param);
    }
    // Best params
    out_params.clear();
    Eigen::Matrix<double, 16, 1> best_walking;
    best_walking << 4, M_PI_4/4, 2, 0.6, 2, M_PI, 2, 1.3,
                    8, M_PI + M_PI_4/4, 4, 0.6, 2, 0, 2, 1.3;
    out_params.push_back(best_walking);
    /*
    out_params.clear();
    // Trotting params also used for turning
    Eigen::Matrix<double, 16, 1> best_trotting;
    best_trotting << 2, M_PI_4/4, 4, 0.6, 2, M_PI, 2, 0.8,
                     4, M_PI+M_PI_4/4, 8, 0.6, 2, 2 * M_PI, 2, 0.8;
    out_params.push_back(best_trotting);
     */
}

std::shared_ptr<SpineTrajectoryOptimiseWorld> make_optimiser_time_dep(
    const dart::simulation::WorldPtr &world, const std::vector<Eigen::VectorXd> &params)
{
    SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> stgs;
    stgs[SpineJointId::FRONT_PITCH] = std::make_shared<SpineTimeDependantTrajectoryGenerator>();
    stgs[SpineJointId::FRONT_YAW] = std::make_shared<SpineTimeDependantTrajectoryGenerator>();
    stgs[SpineJointId::REAR_PITCH] = std::make_shared<SpineTimeDependantTrajectoryGenerator>();
    stgs[SpineJointId::REAR_YAW] = std::make_shared<SpineTimeDependantTrajectoryGenerator>();

    return std::make_shared<SpineTrajectoryOptimiseWorld>(world, stgs, params);
}

void generate_params_time_dep(std::vector<Eigen::VectorXd> &out_params) {
    // theta = A * sin(2*pi*phase + B), stiffness = C, damping = D
    // Mirror params for front and back joints
    const double MAG_PITCH = 4 * M_PI / 180.;
    const double MAG_YAW = 2.3 * M_PI / 180.;
    std::vector<double> A_p {MAG_PITCH, MAG_PITCH/2, MAG_PITCH/4, MAG_PITCH/8};
    std::vector<double> B_p {0, M_PI_2, M_PI, 3 * M_PI_2};
    std::vector<double> C_p {16, 32};
    std::vector<double> D_p {0.25, 0.6};
    std::vector<double> A_y {MAG_YAW, MAG_YAW/2, MAG_YAW/4, MAG_YAW/8};
    std::vector<double> B_y {0.84 * M_PI, 1.34 * M_PI, 1.84 * M_PI, 0.34 * M_PI};
    std::vector<double> C_y {2, 4, 8};
    std::vector<double> D_y {0.4, 0.8};

    std::vector<Eigen::VectorXd> all_params;
    out_params.clear();
    double total_perms = A_p.size() * B_p.size() * C_p.size() * D_p.size()
        * A_y.size() * B_y.size() * C_y.size() * D_y.size();
    out_params.reserve(total_perms);
    all_params.reserve(total_perms);
    std::vector<std::vector<double>> perms {A_p, B_p, C_p, D_p, A_y, B_y, C_y, D_y};
    auto perm_params = cartesianProduct(perms);
    for (auto &p : perm_params) {
        Eigen::Matrix<double, 16, 1> param;
        param.head<8>() = p;
        param.tail<8>() = param.head<8>();
        // Out of phase by PI for front and back
        param(9) += M_PI;
        param(13) += M_PI;
        all_params.push_back(param);
    }
    /*
    // Continue from where we left off
    for (size_t i = 0; i < all_params.size(); i++) {
        out_params.push_back(all_params[i]);
    }
     */
    Eigen::Matrix<double, 16, 1> best_walking;
    best_walking << MAG_PITCH / 2, M_PI_2, 32, 0.6, MAG_YAW / 4, 1.84 * M_PI, 2, 0.8,
                    MAG_PITCH / 2, 3 * M_PI_2, 32, 0.6, MAG_YAW / 4, 3.84 * M_PI, 2, 0.8;
    out_params.push_back(best_walking);
    /*
    Eigen::Matrix<double, 16, 1> best_trotting;
    best_trotting << MAG_PITCH / 2, M_PI_2, 32, 0.25, MAG_YAW / 8, 0.34 * M_PI, 8, 0.8,
                     MAG_PITCH / 2, 3 * M_PI_2, 32, 0.25, MAG_YAW / 8, 1.34 * M_PI, 8, 0.8;
    out_params.push_back(best_trotting);
     */
}

void generate_params_time_dep_real(std::vector<Eigen::VectorXd> &out_params) {
    const double MAG_PITCH = 4 * M_PI / 180.;
    const double MAG_YAW = 2.3 * M_PI / 180.;
    Eigen::Matrix<double, 16, 1> best_walking;
    best_walking << MAG_PITCH, 3 * M_PI_2, 32, 0.6, MAG_YAW, 1.34 * M_PI, 8, 0.4,
                    MAG_PITCH, M_PI_2, 32, 0.6, MAG_YAW, 0.34 * M_PI, 8, 0.4;
    out_params.push_back(best_walking);
    /*
    Eigen::Matrix<double, 16, 1> best_trotting;
    best_trotting << MAG_PITCH, M_PI, 32, 0.25, MAG_YAW, 1.34 * M_PI, 8, 0.4, 
                     MAG_PITCH, 0, 32, 0.25, MAG_YAW, 0.34 * M_PI, 8, 0.4;
    out_params.push_back(best_trotting);
    */
}

std::shared_ptr<SpineTrajectoryOptimiseWorld> make_optimiser_foot_dep(
    const dart::simulation::WorldPtr &world, const std::vector<Eigen::VectorXd> &params)
{
    SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> stgs;
    stgs[SpineJointId::FRONT_PITCH] = std::make_shared<SpineFootPosDependantTrajectoryGenerator>();
    stgs[SpineJointId::FRONT_YAW] = std::make_shared<SpineFootPosDependantTrajectoryGenerator>();
    stgs[SpineJointId::REAR_PITCH] = std::make_shared<SpineFootPosDependantTrajectoryGenerator>();
    stgs[SpineJointId::REAR_YAW] = std::make_shared<SpineFootPosDependantTrajectoryGenerator>();

    return std::make_shared<SpineTrajectoryOptimiseWorld>(world, stgs, params);
}

void generate_params_foot_dep(std::vector<Eigen::VectorXd> &out_params) {
    // theta = f(foot positions, A), stiffness = B, damping = C
    // Mirror params for front and back joints
    std::vector<double> A_p {1, 2, 3};
    std::vector<double> B_p {64, 32, 16};
    std::vector<double> C_p {0.1, 0.4, 0.8};
    std::vector<double> A_y {0.25, 0.5, 0.75};
    std::vector<double> B_y {4, 8, 16};
    std::vector<double> C_y {0.4, 0.8, 1.6};

    out_params.clear();
    double total_perms = A_p.size() * B_p.size() * C_p.size() * A_y.size() * B_y.size() * C_y.size();
    out_params.reserve(total_perms);
    std::vector<std::vector<double>> perms {A_p, B_p, C_p, A_y, B_y, C_y};
    auto perm_params = cartesianProduct(perms);
    for (auto &p : perm_params) {
        Eigen::Matrix<double, 12, 1> param;
        param.head<6>() = p;
        param.tail<6>() = p;
        // Extra stiffness for rear spine pitch due to higher inertia
        param(7) *= 2;
        out_params.push_back(param);
    }

    out_params.clear();
    Eigen::Matrix<double, 12, 1> best_walking;
    best_walking << 1, 32, 0.8, 0.25, 4, 1.6, 1, 64, 0.8, 0.25, 4, 1.6;
    out_params.push_back(best_walking);
    /*
    out_params.clear();
    Eigen::Matrix<double, 12, 1> best_trotting;
    best_trotting << 1., 32, 0.1, 0.5, 4, 1.6, 1, 64, 0.1, 0.5, 4, 1.6;
    out_params.push_back(best_trotting);
     */
}

typedef std::function<std::pair<double, Eigen::VectorXd>(const std::vector<Eigen::VectorXd>&)> OPT_FUNC;

class RunThreadedSolvers {
public:
    RunThreadedSolvers(const OPT_FUNC &func, const std::vector<Eigen::VectorXd> &params)
        :f_(func), params_(params)
    {
        assert(params.size());
        shutting_down_ = false;
        work_finished_ = false;
        best_reward_ = std::numeric_limits<double>::infinity();
        best_parameters_ = params[0];
        batch_size_ = 10;
        current_param_it_ = 0;
        size_t n_threads = std::thread::hardware_concurrency() * 3/4;
        if (n_threads == 0) {
            n_threads = 1;
        }
        workers_.resize(n_threads);
        for (size_t i = 0; i < n_threads; i++) {
            workers_[i] = create_solver_thread();
        }
    };

    ~RunThreadedSolvers() {
        shutting_down_ = true;
        for (auto &worker : workers_) {
            worker.join();
        }
    };

    void spin() {
        while (!shutting_down_ && ros::ok() && !work_finished_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        for (auto &worker : workers_) {
            worker.join();
        }
        shutting_down_ = true;
        std::cerr << "Finished! Best parameters: " << best_parameters_.transpose()
            << " (cost: " << best_reward_ << ")" << std::endl;
    };

private:
    std::thread create_solver_thread() {
        return std::thread([this]() {
            auto start = std::chrono::high_resolution_clock::now();
            while(!shutting_down_ && ros::ok()) {
                std::vector<Eigen::VectorXd> params;
                params.reserve(batch_size_);
                {
                    std::scoped_lock lock{mut_};
                    if (current_param_it_ >= params_.size()) {
                        // Finished all work
                        work_finished_ = true;
                        return;
                    }
                    for (size_t i = 0; i < batch_size_ && current_param_it_ < params_.size(); i++) {
                        params.push_back(params_[current_param_it_++]);
                    }
                }
                std::pair<double, Eigen::VectorXd> result = f_(params);
                {
                    std::scoped_lock lock{result_mut_};
                    if (result.first < best_reward_) {
                        best_reward_ = result.first;
                        best_parameters_ = result.second;
                    }
                }
                {
                    std::scoped_lock lock{mut_};
                    auto now = std::chrono::high_resolution_clock::now();
                    double time_per_it = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count() / current_param_it_;
                    size_t total_its = params_.size();
                    size_t its_left = total_its - current_param_it_;
                    std::cerr << "================================" << std::endl;
                    std::cerr << "Current parameter iteration: " << current_param_it_ << "/" << total_its << std::endl;
                    std::cerr << "Current best parameters: " << best_parameters_.transpose()
                        << " (cost: " << best_reward_ << ")" << std::endl;
                    std::cerr << "Est. time remaining: " << time_per_it * its_left << " s." << std::endl;
                    std::cerr << "================================" << std::endl;
                }
            }
        });
    };

    OPT_FUNC f_;
    std::mutex mut_;
    std::mutex result_mut_;
    std::vector<Eigen::VectorXd> params_;
    std::vector<std::thread> workers_;
    size_t batch_size_;
    std::atomic<size_t> current_param_it_;
    std::atomic<bool> shutting_down_;
    std::atomic<bool> work_finished_;
    Eigen::VectorXd best_parameters_;
    double best_reward_;
};

class CameraUpdateCallback : public dart::gui::osg::ViewerAttachment {
public:
    CameraUpdateCallback(const std::function<void()> &cb) :dart::gui::osg::ViewerAttachment(), f_(cb) {};
    virtual ~CameraUpdateCallback() = default;
    void refresh() override { f_(); };
private:
    const std::function<void()> f_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "optimise_spine_trajectory_parameters");
    [[maybe_unused]] ros::NodeHandle nh;

    bool visualise = true;

    std::vector<Eigen::VectorXd> params;

    if (visualise) {
        generate_params_time_dep_real(params);
        dart::gui::osg::ImGuiViewer viewer;
        dart::simulation::WorldPtr world = std::make_shared<dart::simulation::World>();
        std::shared_ptr<SpineTrajectoryOptimiseWorld> world_optimiser = make_optimiser_time_dep(world, params);
        //std::shared_ptr<SpineTrajectoryOptimiseWorld> world_optimiser = make_optimiser_fixed(world);
        Eigen::Isometry3d robot_tf = world->getSkeleton("ester")->getBodyNode("spine_center_link")->getTransform();
        auto hist = world_optimiser->record();

        osg::ref_ptr<ReplayWorldOSG> world_node =
            new ReplayWorldOSG(world, hist);

        viewer.addWorldNode(world_node);
        viewer.addEventHandler(new EventHandler(world_node));
        viewer.setUpViewInWindow(0, 0, 640, 480);
        osg::Vec3 eye(1.68f, 1.57f, 1.22f);
        osg::Vec3 up(-0.24f, -0.25f, 0.94f);
        osg::Vec3 center(robot_tf.translation().x(), robot_tf.translation().y(), robot_tf.translation().z());
        viewer.getCameraManipulator()->setHomePosition(
            eye, osg::Vec3(0.00f,  0.00f, 0.00f), up);
        viewer.setCameraManipulator(viewer.getCameraManipulator());
        auto update_view_tracking = [&]() {
            Eigen::Isometry3d robot_tf = world->getSkeleton("ester")->getBodyNode("spine_center_link")->getTransform();
            osg::Vec3 center(robot_tf.translation().x(), robot_tf.translation().y(), robot_tf.translation().z());
            osg::Vec3 norm_cent = center - eye;
            norm_cent.normalize();
            double b = sqrt(1. / (1. - pow(norm_cent.z(), 2)));
            double a = - b * norm_cent.z();
            up = osg::Vec3(a * norm_cent.x(), a * norm_cent.y(), a * norm_cent.z() + b);
            if (up.z() < 0) {
                up = -up;
            }
            viewer.getCameraManipulator()->setHomePosition(
                eye, center, up);
            viewer.setCameraManipulator(viewer.getCameraManipulator());
        };
        auto update_view_top_down = [&]() {
            Eigen::Isometry3d robot_tf = world->getSkeleton("ester")->getBodyNode("spine_center_link")->getTransform();
            center.x() = robot_tf.translation().x();
            //center.y() = 0;
            // For rotational gait
            center.y() = robot_tf.translation().y();
            center.z() = robot_tf.translation().z();
            eye = center;
            eye.z() += 1.8;
            // For rotational gait
            eye.z() += 1.8;
            up = osg::Vec3(1, 0, 0);
            viewer.getCameraManipulator()->setHomePosition(
                eye, center, up);
            viewer.setCameraManipulator(viewer.getCameraManipulator());
        };
        auto update_view_side_view = [&]() {
            Eigen::Isometry3d robot_tf = world->getSkeleton("ester")->getBodyNode("spine_center_link")->getTransform();
            center.x() = robot_tf.translation().x();
            center.y() = robot_tf.translation().y();
            eye = center;
            eye.y() += 2.2;
            up = osg::Vec3(0, 0, 1);
            viewer.getCameraManipulator()->setHomePosition(
                eye, center, up);
            viewer.setCameraManipulator(viewer.getCameraManipulator());
        };
        viewer.addAttachment(new CameraUpdateCallback(update_view_tracking));
        //viewer.addAttachment(new CameraUpdateCallback(update_view_top_down));
        //viewer.addAttachment(new CameraUpdateCallback(update_view_side_view));
        //viewer.record(std::string("/home/nick/experiments/time_dep_gait_recording"));
        viewer.simulate(true);
        try {
            viewer.run();
        }
        catch(simulation_finished) {
            // Expected
        }
    }
    else {
        generate_params_stiffness(params);
        auto start = std::chrono::high_resolution_clock::now();
        RunThreadedSolvers solver([] (const std::vector<Eigen::VectorXd> &batch) {
            dart::simulation::WorldPtr world = std::make_shared<dart::simulation::World>();
            std::shared_ptr<SpineTrajectoryOptimiseWorld> world_optimiser = make_optimiser_stiffness(world, batch);
            world_optimiser->solve();
            return std::make_pair(world_optimiser->get_best_parameters_score(), world_optimiser->get_best_parameters());
        }, params);
        solver.spin();
        auto end = std::chrono::high_resolution_clock::now();
        std::cerr << "TOTAL TIME SOLVING: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " SECONDS." << std::endl;
    }
    return EXIT_SUCCESS;
}
