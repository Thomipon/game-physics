#pragma once

#include "Simulator.h"
#include <vector>
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

const double pi_half{std::acos(0.)};

struct box
{
    box(const Vec3& center, const Vec3& size, const Quat& rotation, const Vec3& linear_velocity,
        const Vec3& angular_velocity, const double& mass)
        : center_position(center),
          size(size),
          rotation(rotation.unit()),
          linear_velocity(linear_velocity),
          angular_velocity(angular_velocity),
          mass(mass),
          inertia_tensor_(compute_inertia(size, mass))
    {
    }

    Vec3 center_position;
    Vec3 size;
    Quat rotation;
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    double mass;

    Mat4 get_transform() const;

    Mat4 get_inertia() const;

private:
    Mat4 inertia_tensor_;

    static Mat4 compute_inertia(Vec3 size, double mass);
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

    void set_up_simple();
    void set_up_two_body();
    void set_up_complex();
};
