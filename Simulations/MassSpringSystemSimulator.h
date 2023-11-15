#pragma once

#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

enum class integration_method
{
    euler = 0,
    leapfrog = 1,
    midpoint = 2
};

struct mass_point
{
public:
    mass_point(const Vec3& position, const Vec3& velocity, const bool is_fixed)
        : position(position),
          velocity(velocity),
          is_fixed(is_fixed),
          force(0., 0., 0.)
    {
    }

    Vec3 position;
    Vec3 velocity;

    bool is_fixed;
    Vec3 force;

    void process_collision(const struct plane& plane, float radius);
};

struct plane
{
    plane() = default;

    plane(const Vec3& position, const Vec3& normal)
        : position(position),
          normal(normal)
    {
    }

public:
    Vec3 position{0., -1., 0.};
    Vec3 normal{0., 1., 0.};
};

struct spring
{
public:
    spring(const int point1, const int point2, const float initial_length)
        : point1(point1),
          point2(point2),
          initial_length(initial_length)
    {
    }

    int point1;
    int point2;
    float initial_length;
};

class MassSpringSystemSimulator : public Simulator
{
public:
    // Constructors
    MassSpringSystemSimulator() = default;

    // UI Functions
    const char* getTestCasesStr() override;
    void initUI(DrawingUtilitiesClass* DUC) override;
    void reset() override;
    void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
    void notifyCaseChanged(int testCase) override;
    void externalForcesCalculations(float timeElapsed) override;
    void resolve_collision();
    void simulateTimestep(float timeStep) override;
    void onClick(int x, int y) override;
    void onMouse(int x, int y) override;

    // Specific Functions
    void setMass(float mass);
    void setStiffness(float stiffness);
    void setDampingFactor(float damping);
    int addMassPoint(Vec3 position, Vec3 velocity, bool isFixed);
    void addSpring(int masspoint1, int masspoint2, float initial_length);
    int getNumberOfMassPoints() const;
    int getNumberOfSprings() const;
    Vec3 getPositionOfMassPoint(int index);
    Vec3 getVelocityOfMassPoint(int index);
    void applyExternalForce(const Vec3& force);

    // Do Not Change
    void setIntegrator(int integrator)
    {
        m_iIntegrator = static_cast<integration_method>(integrator);
    }

private:
    // Data Attributes
    float m_fMass;
    float m_fStiffness;
    float m_fDamping;
    integration_method m_iIntegrator;

    // UI Attributes
    Vec3 m_externalForce;
    Point2D m_mouse;
    Point2D m_trackmouse;
    Point2D m_oldtrackmouse;

    std::vector<mass_point> mass_points_;
    std::vector<spring> springs_;
    bool is_first_frame_;

    void set_up_simple_case();
    void set_up_complex_case();

    bool only_first_;

    plane collision_plane_{};
    float radius_{.1};

    void print_state() const;

    void compute_spring_force(const spring& spring);
    void compute_all_forces();
    Vec3 compute_acceleration(const Vec3& force, const Vec3& velocity) const;
    void simulate_euler(float time_step);
    void simulate_midpoint(float time_step);
    void simulate_leapfrog(float time_step);
};
