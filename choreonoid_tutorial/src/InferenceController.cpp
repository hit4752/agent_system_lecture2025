#include <cnoid/MessageView>
#include <cnoid/SimpleController>
#include <cnoid/ValueTree>
#include <cnoid/YAMLReader>

#include <torch/torch.h>
#include <torch/script.h>

#include <random>  // C++標準乱数

using namespace cnoid;

class InferenceController1 : public SimpleController
{
    Body* ioBody;
    double dt;
    int inference_interval_steps;

    Vector3 global_gravity;
    VectorXd last_action;
    VectorXd default_dof_pos;
    std::vector<int> motor_dofs;
    std::vector<std::string> motor_dof_names;

    torch::jit::script::Module model;

    // Config values
    int num_actions = 1;
    double action_scale = 1.0;
    double ang_vel_scale = 1.0;
    double lin_vel_scale = 1.0;
    double dof_pos_scale = 1.0;
    double dof_vel_scale = 1.0;
    std::vector<double> command_scale = {1.0, 1.0, 1.0};

    // Command resampling
    std::vector<double> command{0.0, 0.0, 0.0};
    Listing* lin_vel_x_range;
    Listing* lin_vel_y_range;
    Listing* ang_vel_range;
    int resample_interval_steps = 50; // 例：1秒ごと (dt=0.02なら50step)
    int step_count = 0;

    // 乱数生成器
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist_lin_x;
    std::uniform_real_distribution<double> dist_lin_y;
    std::uniform_real_distribution<double> dist_ang;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        dt = io->timeStep();
        ioBody = io->body();

        inference_interval_steps = static_cast<int>(std::round(0.02 / dt)); // genesis dt = 0.02 [sec]
        std::ostringstream oss;
        oss << "inference_interval_steps: " << inference_interval_steps;
        MessageView::instance()->putln(oss.str());

        global_gravity = Vector3(0.0, 0.0, -1.0);

        for(auto joint : ioBody->joints()) {
            joint->setActuationMode(JointTorque);
            io->enableOutput(joint, JointTorque);
            io->enableInput(joint, JointAngle | JointVelocity);
        }
        io->enableInput(ioBody->rootLink(), LinkPosition | LinkTwist);

        // --- Load config ---
        YAMLReader reader;
        auto root = reader.loadDocument("/home/k-kojima/genesis_ws/logs/go2-walking/cfgs.yaml")->toMapping();

        auto env_cfg = root->findMapping("env_cfg");
        auto obs_cfg = root->findMapping("obs_cfg");
        auto command_cfg = root->findMapping("command_cfg");

        num_actions = env_cfg->get("num_actions", 1);
        last_action = VectorXd::Zero(num_actions);
        default_dof_pos = VectorXd::Zero(num_actions);

        auto dof_names = env_cfg->findListing("dof_names");
        motor_dof_names.clear();
        for(int i=0; i<dof_names->size(); ++i){
            motor_dof_names.push_back(dof_names->at(i)->toString());
        }

        auto default_angles = env_cfg->findMapping("default_joint_angles");
        for(int i=0; i<motor_dof_names.size(); ++i){
            std::string name = motor_dof_names[i];
            default_dof_pos[i] = default_angles->get(name, 0.0);
        }

        action_scale = env_cfg->get("action_scale", 1.0);

        ang_vel_scale = obs_cfg->findMapping("obs_scales")->get("ang_vel", 1.0);
        lin_vel_scale = obs_cfg->findMapping("obs_scales")->get("lin_vel", 1.0);
        dof_pos_scale = obs_cfg->findMapping("obs_scales")->get("dof_pos", 1.0);
        dof_vel_scale = obs_cfg->findMapping("obs_scales")->get("dof_vel", 1.0);

        command_scale[0] = lin_vel_scale;
        command_scale[1] = lin_vel_scale;
        command_scale[2] = ang_vel_scale;

        lin_vel_x_range = command_cfg->findListing("lin_vel_x_range");
        lin_vel_y_range = command_cfg->findListing("lin_vel_y_range");
        ang_vel_range = command_cfg->findListing("ang_vel_range");

        // モータDOFのindex取得
        for(const auto& name : motor_dof_names){
            auto joint = ioBody->joint(name);
            if(joint){
                motor_dofs.push_back(joint->jointId());
            } else {
                std::cerr << "Joint " << name << " not found." << std::endl;
            }
        }

        // 乱数初期化
        rng.seed(std::random_device{}());
        dist_lin_x = std::uniform_real_distribution<double>(lin_vel_x_range->at(0)->toDouble(), lin_vel_x_range->at(1)->toDouble());
        dist_lin_y = std::uniform_real_distribution<double>(lin_vel_y_range->at(0)->toDouble(), lin_vel_y_range->at(1)->toDouble());
        dist_ang = std::uniform_real_distribution<double>(ang_vel_range->at(0)->toDouble(), ang_vel_range->at(1)->toDouble());

        // --- Load model ---
        // model = torch::jit::load("/home/k-kojima/genesis_ws/logs/go2-walking/policy_traced.pt")
        // model.to(torch::kCUDA);
        model = torch::jit::load("/home/k-kojima/genesis_ws/logs/go2-walking/policy_traced.pt", torch::kCPU);
        model.to(torch::kCPU);
        model.eval();

        return true;
    }

    bool inference(VectorXd& target_dof_pos, const Vector3d& angular_velocity, const Vector3d& projected_gravity, const VectorXd& joint_pos, const VectorXd& joint_vel) {
        try {
            // observation vector
            std::vector<float> obs_vec;
            for(int i=0; i<3; ++i) obs_vec.push_back(angular_velocity[i] * ang_vel_scale);
            for(int i=0; i<3; ++i) obs_vec.push_back(projected_gravity[i]);
            for(int i=0; i<3; ++i) obs_vec.push_back(command[i] * command_scale[i]);
            for(int i=0; i<num_actions; ++i) obs_vec.push_back((joint_pos[i] - default_dof_pos[i]) * dof_pos_scale);
            for(int i=0; i<num_actions; ++i) obs_vec.push_back(joint_vel[i] * dof_vel_scale);
            for(int i=0; i<num_actions; ++i) obs_vec.push_back(last_action[i]);

            // auto input = torch::from_blob(obs_vec.data(), {1, (long)obs_vec.size()}, torch::kFloat32).to(torch::kCUDA);
            auto input = torch::from_blob(obs_vec.data(), {1, (long)obs_vec.size()}, torch::kFloat32).to(torch::kCPU);

            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(input);

            // inference
            torch::Tensor output = model.forward(inputs).toTensor();
            auto output_cpu = output.to(torch::kCPU);
            auto output_acc = output_cpu.accessor<float, 2>();

            VectorXd action(num_actions);
            for(int i=0; i<num_actions; ++i){
                last_action[i] = output_acc[0][i];
                action[i] = last_action[i];
            }

            target_dof_pos = action * action_scale + default_dof_pos;
        }
        catch (const c10::Error& e) {
            std::cerr << "Inference error: " << e.what() << std::endl;
        }

        return true;
    }

    virtual bool control() override
    {

        if(step_count % resample_interval_steps == 0){
            command[0] = dist_lin_x(rng);
            command[1] = dist_lin_y(rng);
            command[2] = dist_ang(rng);
        }

        const auto rootLink = ioBody->rootLink();
        const Isometry3d root_coord = rootLink->T();
        Vector3 angular_velocity = rootLink->w();
        Vector3 projected_gravity = root_coord.linear().transpose() * global_gravity;

        if (step_count == 0) {
            // std::vector<double> joint_pos(num_actions), joint_vel(num_actions);
            VectorXd joint_pos(num_actions), joint_vel(num_actions);
            for(int i=0; i<num_actions; ++i){
                auto joint = ioBody->joint(motor_dofs[i]);
                joint_pos[i] = joint->q();
                joint_vel[i] = joint->dq();
            }

            VectorXd target_dof_pos;
            inference(target_dof_pos, angular_velocity, projected_gravity, joint_pos, joint_vel);

            // static const double P_gain = 100.0;
            // static const double D_gain = 10.0;
            static const double P_gain = 20.0;
            static const double D_gain = 0.5;

            for(int i=0; i<num_actions; ++i) {
                auto joint = ioBody->joint(motor_dofs[i]);
                double q = joint->q();
                double dq = joint->dq();
                double u = P_gain * (target_dof_pos[i] - q) - D_gain * dq;
                joint->u() = u;
            }
        }
        step_count = (step_count + 1) % inference_interval_steps;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(InferenceController1)
