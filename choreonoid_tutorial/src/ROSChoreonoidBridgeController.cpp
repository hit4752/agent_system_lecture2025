#include <cnoid/SimpleController>
#include <cnoid/EigenUtil>
// ROS
#include <ros/ros.h>
#include <choreonoid_tutorial/RobotObservation.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace cnoid;

class RosChoreonoidBridgeController1 : public SimpleController
{
    // ROS
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::unique_ptr<ros::Rate> rate;
    // trajectory_msgs::JointTrajectory command_msg;
    choreonoid_tutorial::RobotObservation observation_msg;

    int last_received_sequence;
    int sequence_number;

    ros::Time timestamp_received, timestamp_sent;

    // interfaces for a simulated body
    Body* ioBody;

    // data buffer
    std::vector<double> q_ref;
    std::vector<double> dq_ref;

    // control timestep
    double dt;

public:
    void callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
        if (msg->header.stamp == timestamp_sent) {
            timestamp_received = msg->header.stamp;  // 最新のシーケンス番号を保存
            ROS_INFO("Received new data: %f", timestamp_received.toSec());
        } else {
            ROS_INFO("Received duplicate or out-of-order data: %f", msg->header.stamp.toSec());
        }
    }

    virtual bool configure(SimpleControllerConfig* config) override
    {
        if(!ros::isInitialized()){
            config->os() << config->controllerName()
                         << " Call ros::init() for initializing choreonoid as ROS node" << std::endl;
            int fake_argc = 0;
            char** fake_argv = nullptr;
            ros::init(fake_argc, fake_argv, "choreonoid");
        }

        nh.reset(new ros::NodeHandle);

        pub = nh->advertise<choreonoid_tutorial::RobotObservation>("server2client", 1);
        sub = nh->subscribe("client2server", 1, &RosChoreonoidBridgeController1::callback, this);

        rate = std::make_unique<ros::Rate>(200); // 200Hzで通信

        // initialize counter
        last_received_sequence = -1;
        sequence_number = 0;

        return true;
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        // initializes variables
        dt = io->timeStep();
        ioBody = io->body();

        q_ref.clear();
        q_ref.reserve(ioBody->numJoints());
        dq_ref.clear();
        dq_ref.reserve(ioBody->numJoints());

        // enable joints
        for (auto joint : ioBody->joints()) {
            joint->setActuationMode(JointTorque);
            // io->enableIO(joint);
            io->enableOutput(joint, JointTorque);
            io->enableInput(joint, JointAngle | JointVelocity);

            const double q = joint->q();
            q_ref.push_back(q);
            dq_ref.push_back(0); // dq reference is 0 temporally
        }
        // enable rootLink
        io->enableInput(ioBody->rootLink(), LinkPosition | LinkTwist);

        observation_msg.joint_states.name.resize(ioBody->numJoints());
        observation_msg.joint_states.position.resize(ioBody->numJoints());
        observation_msg.joint_states.velocity.resize(ioBody->numJoints());

        for (int i = 0; i < ioBody->numJoints(); ++i){
            auto joint = ioBody->joint(i);
            observation_msg.joint_states.name[i] = joint->name();
            observation_msg.joint_states.position[i] = joint->q();
            observation_msg.joint_states.velocity[i] = joint->dq();
        }

        timestamp_received = ros::Time::now();
        timestamp_sent = ros::Time::now();

        return true;
    }

    virtual bool control() override
    {
        const Isometry3 root_coord = ioBody->rootLink()->T();

        while (ros::ok() && timestamp_sent > timestamp_received) {
            observation_msg.header.stamp = timestamp_sent;

            pub.publish(observation_msg);
            ROS_INFO("Sent: %f", timestamp_sent.toSec());

            ros::spinOnce();
            rate->sleep();
        }

        if (timestamp_sent == timestamp_received) {
            timestamp_sent = ros::Time::now();
            for (int i = 0; i < ioBody->numJoints(); ++i) {
                auto joint = ioBody->joint(i);
                observation_msg.joint_states.name[i] = joint->name();
                observation_msg.joint_states.position[i] = joint->q();
                observation_msg.joint_states.velocity[i] = joint->dq();
            }

            // orientation
            cnoid::Quaternion quaternion(root_coord.rotation());
            observation_msg.imu.orientation.x = quaternion.x();
            observation_msg.imu.orientation.y = quaternion.y();
            observation_msg.imu.orientation.z = quaternion.z();
            observation_msg.imu.orientation.w = quaternion.w();

            // angular velocity
            cnoid::Vector3 w = ioBody->rootLink()->w();
            observation_msg.imu.angular_velocity.x = w.x();
            observation_msg.imu.angular_velocity.y = w.y();
            observation_msg.imu.angular_velocity.z = w.z();

            // Vector3 pos = root_coord.translation();
            // ROS_INFO("pos: %f %f %f", pos[0], pos[1], pos[2]);
            // Vector3 rpy = rpyFromRot(root_coord.rotation());
            // ROS_INFO("rpy: %f %f %f", rpy[0], rpy[1], rpy[2]);
            // ROS_INFO("w: %f %f %f", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        }

        // PD gains
        static const double pgain = 200.0;
        static const double dgain = 50.0;

        for (int i = 0; i < ioBody->numJoints(); ++i) {
            auto joint = ioBody->joint(i);
            // PD control
            const double u = (q_ref[i] - joint->q()) * pgain + (dq_ref[i] - joint->dq()) * dgain;
            ioBody->joint(i)->u() = u;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RosChoreonoidBridgeController1)
