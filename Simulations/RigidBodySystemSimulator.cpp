﻿#include "RigidBodySystemSimulator.h"

Mat4 box::get_transform() const
{
    const Mat4 scale_mat{
        size.x, 0., 0., 0.,
        0., size.y, 0., 0.,
        0., 0., size.z, 0.,
        0., 0., 0., 1.
    };

    const Mat4 translation_mat{
        1., 0., 0., center_position.x,
        0., 1., 0., center_position.y,
        0., 0., 1., center_position.z,
        0., 0., 0., 1.
    };
    return scale_mat * rotation.getRotMat() * translation_mat;
}

RigidBodySystemSimulator::RigidBodySystemSimulator()
    : Simulator(), m_mouse(), m_trackmouse(), m_oldtrackmouse(), only_first_(false)
{
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Single Step,Single Body,Two Bodies,Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    DUC->setUpLighting(Vec3{}, Vec3{.4}, 100, 0.6 * Vec3{0.97, 0.86, 1});
    for (const auto& body : bodies_)
    {
        DUC->drawRigidBody(body.get_transform());
    }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    only_first_ = testCase == 0;
    switch (testCase)
    {
    case 0:
    case 1:
        set_up_simple();
        break;
    case 2:
        set_up_two_body();
        break;
    case 3:
        set_up_complex();
        break;
    default:
        throw std::runtime_error{"Illegal testcase"};
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() const
{
    return bodies_.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(const int i)
{
    return bodies_.at(i).center_position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(const int i)
{
    return bodies_.at(i).linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(const int i)
{
    return bodies_.at(i).angular_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    bodies_.emplace_back(position, size, Quat{0., 0., 0.}, Vec3{0.}, Vec3{0.}, mass);
}

void RigidBodySystemSimulator::setOrientationOf(const int i, const Quat& orientation)
{
    bodies_.at(i).rotation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(const int i, const Vec3& velocity)
{
    bodies_.at(i).linear_velocity = velocity;
}

void RigidBodySystemSimulator::set_up_simple()
{
    bodies_.clear();

    bodies_.emplace_back(Vec3{0.}, Vec3{1., .6, .5}, Quat{Vec3{0., 0., 1.}, pi_half}, Vec3{0.}, Vec3{0.}, 2.);
    applyForceOnBody(0, Vec3{.3, .5, .25}, Vec3{1., 1., 0});
}

void RigidBodySystemSimulator::set_up_two_body()
{
}

void RigidBodySystemSimulator::set_up_complex()
{
}
