#pragma once

#include "Simulator.h"
#include <vector>
#include "collisionDetect.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

const double pi_half{std::acos(0.)};

struct box
{
    box(const Vec3& center, const Vec3& size, const Quat& rotation, const Vec3& linear_velocity,
        const Vec3& angular_velocity, const double& mass) :
		center_position(center),
		size(size),
		rotation(rotation.unit()),
		linear_velocity(linear_velocity),
	    angular_velocity(angular_velocity),
		mass(mass),
		inertia_tensor_(compute_initial_inertia(size, mass)),
		angular_momentum(inertia_tensor_ * angular_velocity),
		torque(0.),
		forces(0.)
    {
    }

    Vec3 center_position;
    Vec3 size;
    Quat rotation;
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    Vec3 angular_momentum;
    Vec3 torque;
    Vec3 forces;
    double mass;

    Mat4 get_transform() const;

    void simulate_step(float timeStep);

private:
    // inverse inertia tensor
    Mat4 inertia_tensor_;

    static Mat4 compute_initial_inertia(Vec3 size, double mass);

	void update_inertia();
};

class RigidBodySystemSimulator : public Simulator
{
public:
    // Construtors
    RigidBodySystemSimulator();

    // Functions
    const char* getTestCasesStr() override;
    void initUI(DrawingUtilitiesClass* DUC) override;
    void reset() override;
    void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
    void notifyCaseChanged(int testCase) override;
    void externalForcesCalculations(float timeElapsed) override;
    void simulateTimestep(float timeStep) override;
    void onClick(int x, int y) override;
    void onMouse(int x, int y) override;

    // ExtraFunctions
    int getNumberOfRigidBodies() const;
    Vec3 getPositionOfRigidBody(int i);
    Vec3 getLinearVelocityOfRigidBody(int i);
    Vec3 getAngularVelocityOfRigidBody(int i);
    void applyForceOnBody(int i, Vec3 loc, Vec3 force);
    void addRigidBody(Vec3 position, Vec3 size, int mass);
    void setOrientationOf(int i, const Quat& orientation);
    void setVelocityOf(int i, const Vec3& velocity);

private:
    // Attributes
    // add your RigidBodySystem data members, for e.g.,
    // RigidBodySystem * m_pRigidBodySystem; 
    Vec3 m_externalForce;

    // UI Attributes
    Point2D m_mouse;
    Point2D m_trackmouse;
    Point2D m_oldtrackmouse;

    std::vector<box> bodies_;
    bool only_first_;
    bool is_first_;

    void set_up_simple();
    void set_up_two_body();
    void set_up_complex();

    void print_solution();
};
