﻿#include "MassSpringSystemSimulator.h"
#include <array>

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "2 point 1 step, 2 point simulation, complex simulation";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;

    TwAddVarRW(DUC->g_pTweakBar, "IntegrationMethod",
               TwDefineEnumFromString("Integration Method", "euler, leapfrog, midpoint"), &m_iIntegrator, "");
}

void MassSpringSystemSimulator::reset()
{
    m_mouse = m_trackmouse = m_oldtrackmouse = Point2D{};
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
    for (const auto& mass_point : mass_points_)
    {
        DUC->drawSphere(mass_point.position, Vec3{1, 1, 1});
    }
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (testCase)
    {
    case 0:
        only_first_ = true;
        set_up_simple_case();
        break;
    case 1:
        only_first_ = false;
        set_up_simple_case();
        break;
    case 2:
        only_first_ = false;
        set_up_complex_case();
        break;
    default:
        throw std::runtime_error{"Illegal test case"};
    }
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
    std::vector<Vec3> acceleration(getNumberOfMassPoints());
    std::vector<Vec3> positions(getNumberOfMassPoints());
    for(int i = 0; i < getNumberOfMassPoints(); i++)
    {
        positions[i] = getPositionOfMassPoint(i);
    }
    calculateAcceleration(positions, acceleration);

    switch(m_iIntegrator)
    {
    case integration_method::euler:
		for(int i = 0; i < getNumberOfMassPoints(); i++)
		{
            mass_points_[i].position += timeStep * mass_points_[i].velocity;
			mass_points_[i].velocity += timeStep * acceleration[i];
		}
        break;

    case integration_method::midpoint:
        for (int i = 0; i < getNumberOfMassPoints(); i++)
        {
            positions[i] += 0.5 * timeStep * mass_points_[i].velocity;
            mass_points_[i].position += timeStep * (getVelocityOfMassPoint(i) + 0.5 * timeStep * acceleration[i]);
        }
        calculateAcceleration(positions, acceleration);
        for (int i = 0; i < getNumberOfMassPoints(); i++)
        {
            mass_points_[i].velocity += timeStep * acceleration[i];
        }
        break;

    case integration_method::leapfrog:
        break;
    default: break;
    }    
}

void MassSpringSystemSimulator::calculateAcceleration(std::vector<Vec3>& poisitions, std::vector<Vec3>& acceleration) const
{
	for (spring s : springs_)
        {
            float current_length = norm(poisitions[s.point1] - poisitions[s.point2]);
            Vec3 force = -m_fStiffness * (current_length - s.initial_length) * (poisitions[s.point1] - poisitions[s.point2]) / current_length;
            acceleration[s.point1] += force / m_fMass;
            acceleration[s.point2] -= force / m_fMass;
        }
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(const float mass)
{
    m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(const float stiffness)
{
    m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(const float damping)
{
    m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
    mass_points_.emplace_back(position, velocity, isFixed);
    return mass_points_.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initial_length)
{
    springs_.emplace_back(masspoint1, masspoint2, initial_length);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() const
{
    return mass_points_.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() const
{
    return springs_.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(const int index)
{
    return mass_points_.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(const int index)
{
    return mass_points_.at(index).velocity;
}

void MassSpringSystemSimulator::applyExternalForce(const Vec3& force)
{
    m_externalForce = force;
}

void MassSpringSystemSimulator::set_up_simple_case()
{
    mass_points_.clear();
    mass_points_.reserve(2);

    const auto mp1{addMassPoint(Vec3{0., 0., 0.}, Vec3{-1., 0., 0.}, false)};
    const auto mp2{addMassPoint(Vec3{0., 2., 0.}, Vec3{1., 0., 0.}, false)};

    setMass(10);
    setStiffness(40);

    springs_.clear();
    addSpring(mp1, mp2, 1.);

    applyExternalForce(Vec3{0., 0., 0.});
    setDampingFactor(1);
}

void MassSpringSystemSimulator::set_up_complex_case()
{
    constexpr int num_spheres = 10;

    mass_points_.clear();
    mass_points_.reserve(num_spheres);

    std::array<int, num_spheres> mass_points{
        addMassPoint(Vec3{0., 0., 0.}, Vec3{-1., 0., 0.}, false),
        addMassPoint(Vec3{0., 2., 0.}, Vec3{1., 0., 0.}, false),
        addMassPoint(Vec3{0., -2., 0.}, Vec3{1., 1., 0.}, false),
        addMassPoint(Vec3{1., 0., 0.}, Vec3{1., -1., 0.}, false),
        addMassPoint(Vec3{0., 2., -1.}, Vec3{0., 1., 0.}, false),
        addMassPoint(Vec3{2., 0., 0.}, Vec3{-1., -1., 0.}, false),
        addMassPoint(Vec3{3., 2., 0.}, Vec3{1., 0., 1.}, false),
        addMassPoint(Vec3{0., 1., 0.}, Vec3{1., 0., -1.}, false),
        addMassPoint(Vec3{0., 2., -2.}, Vec3{1., 0., .5}, false),
        addMassPoint(Vec3{-3., 3., 0.}, Vec3{.5, 0., 0.}, false)
    };

    setMass(10);
    setStiffness(40);

    springs_.clear();
    for (int i = 0; i < 9; ++i)
    {
        addSpring(mass_points[i], mass_points[i + 1], i / 2.);
    }
    addSpring(mass_points[9], mass_points[0], 5.);

    applyExternalForce(Vec3{0., 0., 0.});
    setDampingFactor(1);
}
