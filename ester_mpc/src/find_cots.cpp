#include "ester_mpc/world_nodes/spine_trajectory_cot_world.hpp"

#include <thread>
#include <mutex>

using namespace ester_mpc;
using namespace ester_common;

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

std::shared_ptr<SpineTrajectoryCotWorld> make_world_fixed(
    const dart::simulation::WorldPtr &world, const Eigen::VectorXd &params,
    const std::vector<std::pair<double, double>> &vels, GaitType gait, bool write)
{
    SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> stgs;
    stgs[SpineJointId::FRONT_PITCH] = std::make_shared<SpineTrajectoryGenerator>();
    stgs[SpineJointId::FRONT_YAW] = std::make_shared<SpineTrajectoryGenerator>();
    stgs[SpineJointId::REAR_PITCH] = std::make_shared<SpineTrajectoryGenerator>();
    stgs[SpineJointId::REAR_YAW] = std::make_shared<SpineTrajectoryGenerator>();

    return std::make_shared<SpineTrajectoryCotWorld>(world, stgs, params, vels, gait, write);
}

std::shared_ptr<SpineTrajectoryCotWorld> make_world_stiffness(
    const dart::simulation::WorldPtr &world, const Eigen::VectorXd &params,
    const std::vector<std::pair<double, double>> &vels, GaitType gait, bool write)
{
    SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> stgs;
    stgs[SpineJointId::FRONT_PITCH] = std::make_shared<SpineStiffnessTrajectoryGenerator>();
    stgs[SpineJointId::FRONT_YAW] = std::make_shared<SpineStiffnessTrajectoryGenerator>();
    stgs[SpineJointId::REAR_PITCH] = std::make_shared<SpineStiffnessTrajectoryGenerator>();
    stgs[SpineJointId::REAR_YAW] = std::make_shared<SpineStiffnessTrajectoryGenerator>();

    return std::make_shared<SpineTrajectoryCotWorld>(world, stgs, params, vels, gait, write);
}

Eigen::VectorXd generate_params_stiffness_walking() {
    // stiffness = A * sin(2*pi*phase + B) + C, damping = D
    Eigen::Matrix<double, 16, 1> params;
    params << 4, M_PI_4/4, 2, 0.6, 2, M_PI, 2, 1.3,
              8, M_PI + M_PI_4/4, 4, 0.6, 2, 0, 2, 1.3;
    return params;
}

Eigen::VectorXd generate_params_stiffness_trotting_turning() {
    // Trotting params also used for turning
    Eigen::Matrix<double, 16, 1> params;
    params << 2, M_PI_4/4, 4, 0.6, 2, M_PI, 2, 0.8,
              4, M_PI+M_PI_4/4, 8, 0.6, 2, 2 * M_PI, 2, 0.8;
    return params;
}

std::shared_ptr<SpineTrajectoryCotWorld> make_world_time_dep(
    const dart::simulation::WorldPtr &world, const Eigen::VectorXd &params,
    const std::vector<std::pair<double, double>> &vels, GaitType gait, bool write,
    bool is_real_params)
{
    SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> stgs;
    stgs[SpineJointId::FRONT_PITCH] = std::make_shared<SpineTimeDependantTrajectoryGenerator>();
    stgs[SpineJointId::FRONT_YAW] = std::make_shared<SpineTimeDependantTrajectoryGenerator>();
    stgs[SpineJointId::REAR_PITCH] = std::make_shared<SpineTimeDependantTrajectoryGenerator>();
    stgs[SpineJointId::REAR_YAW] = std::make_shared<SpineTimeDependantTrajectoryGenerator>();

    std::string extras = "opt_";
    if(is_real_params) {
        extras = "real_dog_";
    }
    return std::make_shared<SpineTrajectoryCotWorld>(world, stgs, params, vels, gait, write, extras);
}

Eigen::VectorXd generate_params_time_dep_walking_turning() {
    // theta = A * sin(2*pi*phase + B), stiffness = C, damping = D
    // Mirror params for front and back joints
    const double MAG_PITCH = 4 * M_PI / 180.;
    const double MAG_YAW = 2.3 * M_PI / 180.;
    Eigen::Matrix<double, 16, 1> params;
    params << MAG_PITCH / 2, M_PI_2, 32, 0.6, MAG_YAW / 4, 1.84 * M_PI, 2, 0.8,
              MAG_PITCH / 2, 3 * M_PI_2, 32, 0.6, MAG_YAW / 4, 3.84 * M_PI, 2, 0.8;
    return params;
}

Eigen::VectorXd generate_params_time_dep_trotting() {
    const double MAG_PITCH = 4 * M_PI / 180.;
    const double MAG_YAW = 2.3 * M_PI / 180.;
    Eigen::Matrix<double, 16, 1> params;
    params << MAG_PITCH / 2, M_PI_2, 32, 0.25, MAG_YAW / 8, 0.34 * M_PI, 8, 0.8,
              MAG_PITCH / 2, 3 * M_PI_2, 32, 0.25, MAG_YAW / 8, 1.34 * M_PI, 8, 0.8;
    return params;
}

Eigen::VectorXd generate_params_time_dep_real_walking_turning() {
    const double MAG_PITCH = 4 * M_PI / 180.;
    const double MAG_YAW = 2.3 * M_PI / 180.;
    Eigen::Matrix<double, 16, 1> params;
    params << MAG_PITCH, 3 * M_PI_2, 32, 0.6, MAG_YAW, 1.34 * M_PI, 8, 0.4,
              MAG_PITCH, M_PI_2, 32, 0.6, MAG_YAW, 0.34 * M_PI, 8, 0.4;
    return params;
}

Eigen::VectorXd generate_params_time_dep_real_trotting() {
    const double MAG_PITCH = 4 * M_PI / 180.;
    const double MAG_YAW = 2.3 * M_PI / 180.;
    Eigen::Matrix<double, 16, 1> params;
    params << MAG_PITCH, M_PI, 32, 0.25, MAG_YAW, 1.34 * M_PI, 8, 0.4, 
              MAG_PITCH, 0, 32, 0.25, MAG_YAW, 0.34 * M_PI, 8, 0.4;
    return params;
}

std::shared_ptr<SpineTrajectoryCotWorld> make_world_foot_dep(
    const dart::simulation::WorldPtr &world, const Eigen::VectorXd &params,
    const std::vector<std::pair<double, double>> &vels, GaitType gait, bool write)
{
    SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> stgs;
    stgs[SpineJointId::FRONT_PITCH] = std::make_shared<SpineFootPosDependantTrajectoryGenerator>();
    stgs[SpineJointId::FRONT_YAW] = std::make_shared<SpineFootPosDependantTrajectoryGenerator>();
    stgs[SpineJointId::REAR_PITCH] = std::make_shared<SpineFootPosDependantTrajectoryGenerator>();
    stgs[SpineJointId::REAR_YAW] = std::make_shared<SpineFootPosDependantTrajectoryGenerator>();

    return std::make_shared<SpineTrajectoryCotWorld>(world, stgs, params, vels, gait, write);
}

Eigen::VectorXd generate_params_foot_dep_walking_turning() {
    // theta = f(foot positions, A), stiffness = B, damping = C
    Eigen::Matrix<double, 12, 1> params;
    params << 1, 32, 0.8, 0.25, 4, 1.6, 1, 64, 0.8, 0.25, 4, 1.6;
    return params;
}

Eigen::VectorXd generate_params_foot_dep_trotting() {
    Eigen::Matrix<double, 12, 1> params;
    params << 1., 32, 0.1, 0.5, 4, 1.6, 1, 64, 0.1, 0.5, 4, 1.6;
    return params;
}

typedef std::function<std::shared_ptr<SpineTrajectoryCotWorld>(
    const dart::simulation::WorldPtr &, const Eigen::VectorXd &,
    const std::vector<std::pair<double, double>> &, GaitType, bool)> WORLD_GEN_FUNC;

class RunExperimentThreaded {
public:
    RunExperimentThreaded(bool visualise, bool write_to_file) : vis_(visualise), write_to_file_(write_to_file)
    {
        shutting_down_ = false;
        work_finished_ = false;
        build_experiment();
        current_experiment_it_ = 0;
        size_t n_threads = std::thread::hardware_concurrency() * 3/4;
        if (n_threads > num_experiments_) {
            n_threads = num_experiments_ - 1;
        }
        if (visualise) {
            n_threads = 0;
        }

        std::cerr << "Running " << n_threads << " tasks in parallel." << std::endl;
        workers_.resize(n_threads);
        for (size_t i = 0; i < n_threads; i++) {
            workers_[i] = std::thread([this]() {work_func();});
        }
    };

    ~RunExperimentThreaded() {
        shutting_down_ = true;
        for (auto &worker : workers_) {
            worker.join();
        }
    };

    void run() {
        std::cerr << "Starting running at " << get_timestamp() << std::endl;
        work_func();
        for (auto &worker : workers_) {
            worker.join();
        }
        shutting_down_ = true;
        std::cerr << "Finished! Experiment parameters and CoTs follow:";
        for (size_t i = 0; i < num_experiments_; i++) {
            std::cerr << std::endl << experiment_names_[i] << ": lin vel, ang vel, cot";
            for (size_t j = 0; j < cots_[i].size(); j++) {
                std::cerr << std::endl << "\t" << vels_[i][j].first << ", " << vels_[i][j].second
                    << ", " << cots_[i][j];
            }
        }
        std::cerr << std::endl;
        write_cots_to_file();
    };

private:
    void work_func() {
        auto start = std::chrono::high_resolution_clock::now();
        while(!shutting_down_ && ros::ok()) {
            size_t idx;
            {
                std::scoped_lock lock{mut_};
                if (current_experiment_it_ >= num_experiments_) {
                    // Finished all work
                    work_finished_ = true;
                    return;
                }
                idx = current_experiment_it_++;
            }
            auto res = dispatch_func(idx);
            cots_[idx] = res.first;
            cots_legs_only_[idx] = res.second;
            {
                std::scoped_lock lock{mut_};
                auto now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> secs = now - start;
                std::cerr << "================================" << std::endl;
                std::cerr << "Finished experiment: " << idx << "/" << num_experiments_
                    << " in " << secs.count() << " seconds" << std::endl;
                std::cerr << "================================" << std::endl;
            }
        }
    }

    std::pair<std::vector<double>, std::vector<double>> dispatch_func(size_t idx)
    {
        dart::simulation::WorldPtr world = std::make_shared<dart::simulation::World>();
        Eigen::VectorXd params = params_[idx];
        const std::vector<std::pair<double, double>> &vels = vels_[idx];
        GaitType gait = gaits_[idx];
        std::shared_ptr<SpineTrajectoryCotWorld> world_manager =
            world_gens_[idx](world, params, vels, gait, write_to_file_);
        if (vis_) {
            SpineTrajectoryCotVisualiser vis(world_manager);
            vis.run();
        }
        else {
            world_manager->run();
        }
        return std::make_pair(world_manager->get_cots(), world_manager->get_cots_legs_only());
    }

    std::string get_timestamp() {
        auto now = std::chrono::system_clock::now();
        std::time_t nowt = std::chrono::system_clock::to_time_t(now);
        std::tm* t = std::gmtime(&nowt);
        std::stringstream ss;
        ss << std::put_time(t, "%Y_%m_%d_%I_%M");
        return ss.str();
    }

    void write_cots_to_file() {
        std::string dir = std::string(getenv("HOME")) + "/experiments/cot_experiments/";
        std::string timestamp = get_timestamp();
        std::string file_path_cots = dir + "cot_data_" + timestamp + ".csv";
        std::string file_path_cots_legs_only = dir + "cot_data_leg_only_" + timestamp + ".csv";

        auto do_write = [this](std::string filepath, std::vector<std::vector<double>> &data) {
            std::ofstream of(filepath, std::ofstream::out | std::ofstream::trunc);
            if (!of) {
                std::cerr << "Failed to open file: " << filepath << std::endl;
                return;
            }
            // Find the longest data series, and write column headers
            size_t n_cols = data.size();
            size_t n_rows = 0;
            for (size_t i = 0; i < n_cols; i++) {
                of << experiment_names_[i] << " lin vel," << experiment_names_[i] << " cot,";
                n_rows = std::max(n_rows, data[i].size());
            }
            // Fill one row at a time
            for (size_t row = 0; row < n_rows; row++) {
                of << std::endl;
                for (size_t col = 0; col < n_cols; col++) {
                    auto &cots = data[col];
                    if (cots.size() < row) {
                        // No more data for this experiment
                        of << ",,";
                    }
                    else if (cots[row] < 0) {
                        // Simulation was unstable
                        of << ",,";
                    }
                    else {
                        // Ignore angular velocities
                        of << vels_[col][row].first << ","
                           << cots[row] << ",";
                    }
                }
            }
        };

        do_write(file_path_cots, cots_);
        do_write(file_path_cots_legs_only, cots_legs_only_);
    }

    std::mutex mut_;
    std::vector<std::thread> workers_;
    std::atomic<size_t> current_experiment_it_;
    std::atomic<bool> shutting_down_;
    std::atomic<bool> work_finished_;

    size_t num_experiments_;
    std::vector<std::vector<double>> cots_;
    std::vector<std::vector<double>> cots_legs_only_;
    std::vector<std::vector<std::pair<double, double>>> vels_;
    std::vector<GaitType> gaits_;
    std::vector<Eigen::VectorXd> params_;
    std::vector<std::string> experiment_names_;
    std::vector<WORLD_GEN_FUNC> world_gens_;

    bool vis_;
    bool write_to_file_;

    void build_experiment() {
        std::vector<std::pair<double, double>> walking_vels, trotting_vels, turning_vels;
        walking_vels = {
            {0.1, 0.},
            {0.15, 0.},
            {0.2, 0.},
            {0.25, 0.},
            {0.3, 0.},
            {0.35, 0.},
            {0.4, 0.},
            {0.45, 0.},
            {0.5, 0.},
            {0.55, 0.},
            {0.6, 0.},
            {0.65, 0.},
            {0.7, 0.},
            {0.75, 0.},
            {0.8, 0.},
            {0.85, 0.},
            {0.9, 0.},
            {0.95, 0.},
            {1., 0.},
        };
        trotting_vels = {
            {0.1, 0.},
            {0.15, 0.},
            {0.2, 0.},
            {0.25, 0.},
            {0.3, 0.},
            {0.35, 0.},
            {0.4, 0.},
            {0.45, 0.},
            {0.5, 0.},
            {0.55, 0.},
            {0.6, 0.},
            {0.65, 0.},
            {0.7, 0.},
            {0.75, 0.},
            {0.8, 0.},
            {0.85, 0.},
            {0.9, 0.},
            {0.95, 0.},
            {1.0, 0.},
            {1.05, 0.},
            {1.1, 0.},
            {1.15, 0.},
            {1.2, 0.},
            {1.25, 0.},
            {1.3, 0.},
            {1.35, 0.},
            {1.4, 0.},
            {1.45, 0.},
            {1.5, 0.},
            {1.55, 0.},
            {1.6, 0.},
            {1.65, 0.},
            {1.7, 0.},
            {1.75, 0.},
            {1.8, 0.},
        };
        turning_vels = {
            {0.3, 0.2},
            {0.3, 0.3},
            {0.3, 0.4},
            {0.3, 0.5},
            {0.3, 0.6},
            {0.3, 0.7},
            {0.3, 0.8},
            {0.3, 0.9},
            {0.3, 1.0},
            {0.3, 1.1},
            {0.3, 1.2},
        };
        for (size_t i = 0; i < 5; i++) {
            gaits_.push_back(GaitType::WALKING);
            vels_.push_back(walking_vels);
        }
        for (size_t i = 0; i < 5; i++) {
            gaits_.push_back(GaitType::TROTTING);
            vels_.push_back(trotting_vels);
        }
        for (size_t i = 0; i < 5; i++) {
            gaits_.push_back(GaitType::WALKING);
            vels_.push_back(turning_vels);
        }
        world_gens_ = {
            make_world_stiffness,
            make_world_foot_dep,
            std::bind(make_world_time_dep, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, false),
            std::bind(make_world_time_dep, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, true),
            make_world_fixed,
            make_world_stiffness,
            make_world_foot_dep,
            std::bind(make_world_time_dep, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, false),
            std::bind(make_world_time_dep, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, true),
            make_world_fixed,
            make_world_stiffness,
            make_world_foot_dep,
            std::bind(make_world_time_dep, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, false),
            std::bind(make_world_time_dep, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, true),
            make_world_fixed,
        };
        params_ = {
            // Walking
            generate_params_stiffness_walking(),
            generate_params_foot_dep_walking_turning(),
            generate_params_time_dep_walking_turning(),
            generate_params_time_dep_real_walking_turning(),
            Eigen::VectorXd(),
            // Trotting
            generate_params_stiffness_trotting_turning(),
            generate_params_foot_dep_trotting(),
            generate_params_time_dep_trotting(),
            generate_params_time_dep_real_trotting(),
            Eigen::VectorXd(),
            // Turning
            generate_params_stiffness_trotting_turning(),
            generate_params_foot_dep_walking_turning(),
            generate_params_time_dep_walking_turning(),
            generate_params_time_dep_real_walking_turning(),
            Eigen::VectorXd(),
        };
        experiment_names_ = {
            "stiffness_walking",
            "foot_dep_walking",
            "time_opt_walking",
            "time_real_walking",
            "fixed_walking",
            "stiffness_trotting",
            "foot_dep_trotting",
            "time_opt_trotting",
            "time_real_trotting",
            "fixed_trotting",
            "stiffness_turning",
            "foot_dep_turning",
            "time_opt_turning",
            "time_real_turning",
            "fixed_turning",
        };

        num_experiments_ = vels_.size();
        cots_.resize(num_experiments_);
        cots_legs_only_.resize(num_experiments_);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "find_CoT");
    [[maybe_unused]] ros::NodeHandle nh;

    auto start = std::chrono::high_resolution_clock::now();
    bool visualise = false;
    bool write_to_file = false;
    ros::param::get("~visualise", visualise);
    ros::param::get("~write_to_file", write_to_file);
    RunExperimentThreaded dispatcher(visualise, write_to_file);
    dispatcher.run();
    auto end = std::chrono::high_resolution_clock::now();
    std::cerr << "TOTAL TIME RUNNING EXPERIMENT: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " SECONDS." << std::endl;
    return EXIT_SUCCESS;
}
