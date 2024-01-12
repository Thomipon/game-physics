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
        const Vec3& angular_velocity, const double& mass, const bool is_static = false) :
        center_position(center),
        size(size),
        rotation(rotation.unit()),
        linear_velocity(linear_velocity),
        angular_velocity(angular_velocity),
        torque(0.),
        forces(0.),
        spring_force(0.),
        spring_torque(0.),
        mass(mass),
        is_static(is_static),
        initial_inv_inertia_(compute_initial_inertia(size, mass)),
        inv_inertia_tensor(get_rotated_inertia(initial_inv_inertia_, rotation.unit())),
        angular_momentum(inv_inertia_tensor.transformVector(angular_velocity))
    {
    }

    Vec3 center_position;
    Vec3 size;
    Quat rotation;
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    Vec3 torque;
    Vec3 forces;
    Vec3 spring_force;
    Vec3 spring_torque;
    double mass;
    bool is_static;

private:
    Mat4 initial_inv_inertia_;

public:
    Mat4 inv_inertia_tensor;
    Vec3 angular_momentum;

    Mat4 get_transform() const;

    Vec3 spring_acceleration(const Vec3& spring_force, const Vec3& linear_velocity) const;
    void simulate_step(float timeStep);
    Vec3 get_point_velocity(const Vec3& position) const;
    void apply_impulse(const Vec3& impulse_normal, const Vec3& position);

private:
    static Mat4 compute_initial_inertia(const Vec3& size, double mass);

    static Mat4 get_rotated_inertia(const Mat4& initial_inv_inertia, const Quat& rotation);
};

struct plane
{
public:
    plane(const Vec3& base, const Vec3& normal)
        : base_(base),
          normal_(normal)
    {
        normalize(normal_);

        Vec3 rot = cross(Vec3{0., 1., 0.}, normal_);
        normalize(rot);
        transform_ = box{
            base_, Vec3{1000., .1, 1000.}, Quat{rot, std::acos(dot(Vec3{0., 1., 0.}, normal_))}, Vec3{0.}, Vec3{0.}, 0.
        }.get_transform();
    }

    void collide_box(box& object, const Vec3& position) const;
    Mat4 get_transform() const;

private:
    Vec3 base_;
    Vec3 normal_;
    Mat4 transform_;
};

struct spring
{
public:
    spring(const int point1, const int point2, const float initial_length, const double stiffness)
        : point1(point1),
          point2(point2),
          initial_length(initial_length),
          stiffness(stiffness)
    {
    }

    int point1;
    int point2;
    float initial_length;
    double stiffness;
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
    static CollisionInfo check_collision_safe(Mat4& body_a, Mat4& body_b);
    void simulateTimestep(float timeStep) override;
    void onClick(int x, int y) override;
    void onMouse(int x, int y) override;

    // ExtraFunctions
    int getNumberOfRigidBodies() const;
    Vec3 getPositionOfRigidBody(int i);
    Vec3 getLinearVelocityOfRigidBody(int i);
    Vec3 getAngularVelocityOfRigidBody(int i);
    void applyForceOnBody(int i, const Vec3& loc, const Vec3& force);
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
    std::vector<spring> springs_;
    std::vector<plane> planes_;

    void set_up_simple();
    void set_up_two_body();
    void set_up_complex();
    void set_up_full();

    void print_solution();
    void collide_bodies(int a, int b, const Vec3& collision_point, const Vec3& normal);

    void applySpringForceOnBody(int i, const Vec3& loc, const Vec3& force);
    void compute_spring_force(const spring& spring);
    void compute_spring_forces();

    void interact();
    void click_on_box(const Vec3& clickPosition, const Vec3& direction);
};
