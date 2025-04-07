#include <cnoid/SimpleController>

using namespace cnoid;

class StandController1 : public SimpleController
{
    // interfaces for a simulated body
    Body* ioBody;

    // data buffer
    std::vector<double> q_prev;
    std::vector<double> q_ref;
    std::vector<double> dq_ref;

    // control timestep
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        // initializes variables
        dt = io->timeStep();
        ioBody = io->body();

        q_prev.clear();
        q_prev.reserve(ioBody->numJoints());
        q_ref.clear();
        q_ref.reserve(ioBody->numJoints());
        dq_ref.clear();
        dq_ref.reserve(ioBody->numJoints());

        // drives joints with torque input
        for (auto joint : ioBody->joints()) {
            joint->setActuationMode(JointTorque);
            io->enableIO(joint);

            const double q = joint->q();
            q_prev.push_back(q);
            q_ref.push_back(q);
            dq_ref.push_back(0); // dq reference is 0 temporally
        }

        return true;
    }

    virtual bool control() override
    {
        // PD gains
        static const double pgain = 200.0;
        static const double dgain = 50.0;

        for (int i = 0; i < ioBody->numJoints(); ++i) {
            const double q = ioBody->joint(i)->q();
            const double dq = (q - q_prev[i]) / dt;

            // PD control
            const double u = (q_ref[i] - q) * pgain + (dq_ref[i] - dq) * dgain;
            ioBody->joint(i)->u() = u;

            // record joint positions
            q_prev[i] = q;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(StandController1)
