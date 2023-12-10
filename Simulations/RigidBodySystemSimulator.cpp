#include "RigidBodySystemSimulator.h"

Mat4 box::get_transform() const
{
    const Mat4 scale_mat{
        size.x, 0., 0., 0.,
        0., size.y, 0., 0.,
        0., 0., size.z, 0.,
        0., 0., 0., 1.
    };

    const Mat4 translation_mat{
        1., 0., 0., 0.,
        0., 1., 0., 0.,
        0., 0., 1., 0.,
        center_position.x, center_position.y, center_position.z, 1.
    };
    return scale_mat * rotation.getRotMat() * translation_mat;
}

Mat4 box::get_rotated_inertia(const Mat4& initial_inv_inertia, const Quat& rotation)
{
    const Mat4 rotation_mat{rotation.getRotMat()};
    Mat4 inv_rotation_mat{rotation_mat};
    inv_rotation_mat.transpose();
    return inv_rotation_mat * initial_inv_inertia * rotation_mat;
}

Mat4 box::compute_initial_inertia(Vec3 size, double mass)
{
    constexpr double factor = 1. / 12.;
    const double scaled_mass = factor * mass;
    return Mat4{
        scaled_mass * (size.y * size.y + size.z * size.z), 0., 0., 0.,
        0., scaled_mass * (size.x * size.x + size.z * size.z), 0., 0.,
        0., 0., scaled_mass * (size.x * size.x + size.y * size.y), 0.,
        0., 0., 0., 1.
    }.inverse();
}

void box::simulate_step(float timeStep)
{
    center_position += linear_velocity * timeStep;
    linear_velocity += timeStep * forces / mass;

    rotation += timeStep * .5 * rotation.dot(Quat{ 0., angular_velocity.x, angular_velocity.y, angular_velocity.z });
    rotation = rotation.unit();

    angular_momentum += timeStep * torque;
    inv_inertia_tensor = get_rotated_inertia(initial_inv_inertia_, rotation);
    angular_velocity = inv_inertia_tensor.transformVectorNormal(angular_momentum);
	//forces = 0.;
    //torque = 0.;
}

Vec3 box::get_point_velocity(Vec3 position) const
{
    return linear_velocity + cross(angular_velocity, position);
}

void box::apply_impulse(Vec3 impulse_normal, Vec3 position)
{
    linear_velocity += impulse_normal / mass;
    angular_momentum += cross(position, impulse_normal);
}

RigidBodySystemSimulator::RigidBodySystemSimulator()
    : Simulator(), m_mouse(), m_trackmouse(), m_oldtrackmouse(), only_first_(false), is_first_(true)
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
    is_first_ = true;
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
    if (only_first_ && !is_first_)
        return;

    for (auto& body : bodies_)
    {
        body.simulate_step(timeStep);
    }

    // Collision Detection
    for(int i = 0; i < getNumberOfRigidBodies(); i++)
    {
	    for(int j = i+1; j < getNumberOfRigidBodies(); j++)
	    {
	        Mat4 body_a{bodies_[i].get_transform()};
	        Mat4 body_b{bodies_[j].get_transform()};
            CollisionInfo collision = checkCollisionSAT(body_a, body_b);
            if(collision.isValid)
            {
                collide_bodies(i, j, collision.collisionPointWorld, collision.normalWorld);
            }
	    }
    }

    if (only_first_ && is_first_) {
        print_solution();
    }
	is_first_ = false;
}

void RigidBodySystemSimulator::collide_bodies(int a, int b, Vec3 collision_point, Vec3 normal)
{
    const Vec3 xa = collision_point - bodies_[a].center_position;
    const Vec3 xb = collision_point - bodies_[b].center_position;
    const Vec3 relative_velocity = bodies_[a].get_point_velocity(xa) - bodies_[b].get_point_velocity(xb);

    const Vec3 inertia_a = cross(bodies_[a].inv_inertia_tensor.transformVector(cross(xa, normal)), xa);
    const Vec3 inertia_b = cross(bodies_[b].inv_inertia_tensor.transformVector(cross(xb, normal)), xb);
    const double denominator = 1. / bodies_[a].mass + 1. / bodies_[b].mass + dot((inertia_a + inertia_b) , normal);

    // c = 0
    const double impulse = - dot(relative_velocity, normal) / denominator;

    bodies_[a].apply_impulse(impulse * normal, xa);
    bodies_[b].apply_impulse(-impulse * normal, xb);
}

void RigidBodySystemSimulator::print_solution()
{
    std::cout << "Linear Velocity: " << getLinearVelocityOfRigidBody(0) << " Angular velocity: " << getAngularVelocityOfRigidBody(0) << "\n";
    std::cout << "Velocity of Point (-0.3, -0.5, -0.25): " << bodies_[0].get_point_velocity(Vec3{ -0.3, -0.5, -0.25 } - bodies_[0].center_position) << std::endl;
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
    bodies_[i].forces += force;
    bodies_[i].torque += cross(loc, force);
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
    bodies_.clear();

    bodies_.emplace_back(Vec3{ -1., 0., 0. }, Vec3{ 0.5, 0.5, 0.5 }, Quat{ Vec3{0., 0., 1.}, 0. }, Vec3{ 0.5, 0., 0. }, Vec3{ 0. }, 3.);
    bodies_.emplace_back(Vec3{ 0.5, 0., 0. }, Vec3{ 0.3, 0.3, 0.3 }, Quat{ Vec3{1., 1., 1.}, 0.5 * pi_half }, Vec3{ -0.25, 0., 0. }, Vec3{ 0. }, 1.);
}

void RigidBodySystemSimulator::set_up_complex()
{
    bodies_.clear();

    bodies_.emplace_back(Vec3{ -1., 0., -1. }, Vec3{ 0.5, 0.5, 0.5 }, Quat{ Vec3{0., 0., 1.}, 0. }, Vec3{ 0.5, 0., 0.5 }, Vec3{ 0. }, 3.);
    bodies_.emplace_back(Vec3{ 0., 1., 0. }, Vec3{ 0.3, 0.6, 0.3 }, Quat{ Vec3{1., 0., 0.}, 0.5 * pi_half }, Vec3{ 0, -0.5, 0. }, Vec3{ 0. }, 3.);
    bodies_.emplace_back(Vec3{ 0., 0., 0. }, Vec3{ 0.7, 0.5, 0.5 }, Quat{ Vec3{0., 0., 1.}, 0. }, Vec3{ 0.5, 0., 0. }, Vec3{ 0. }, 3.);
    bodies_.emplace_back(Vec3{ 1., 0., 1. }, Vec3{ 0.2, 0.5, 0.2 }, Quat{ Vec3{1., 0., 1.}, 0.5 * pi_half }, Vec3{ -0.5, 0., -0.5 }, Vec3{ 0. }, 3.);
}
