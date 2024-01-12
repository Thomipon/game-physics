#include "RigidBodySystemSimulator.h"

CollisionInfo RigidBodySystemSimulator::check_collision_safe(Mat4& body_a, Mat4& body_b)
{
    if (std::isnan(XMMatrixDeterminant((body_a * body_b).toDirectXMatrix()).m128_f32[0]))
        return CollisionInfo{false, Vec3{}, Vec3{}, -1.};
    return checkCollisionSAT(body_a, body_b);
}

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

Vec3 box::spring_acceleration(const Vec3& spring_force, const Vec3& linear_velocity) const
{
    return 1. / mass * (spring_force - 0./*std::min(1., norm(spring_force)) */ * linear_velocity);
}

Mat4 box::get_rotated_inertia(const Mat4& initial_inv_inertia, const Quat& rotation)
{
    const Mat4 rotation_mat{rotation.getRotMat()};
    Mat4 inv_rotation_mat{rotation_mat};
    inv_rotation_mat.transpose();
    return inv_rotation_mat * initial_inv_inertia * rotation_mat;
}

void plane::collide_box(box& object, const Vec3& position) const
{
    //object.apply_impulse(normal_ * abs(dot(normal_, object.linear_velocity)) * 1.1 * object.mass, position);
    object.center_position += .001 * normal_;
    object.linear_velocity = .999 * reflectVector(object.linear_velocity, normal_);
    //object.angular_momentum += cross(normal_, position - object.center_position);
    object.angular_momentum *= .9;
}

Mat4 plane::get_transform() const
{
    return transform_;
}

Mat4 box::compute_initial_inertia(const Vec3& size, const double mass)
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
    if (is_static)
        return;

    center_position += linear_velocity * timeStep;
    Vec3 attraction_force{center_position};
    //normalize(attraction_force);
    attraction_force += Vec3{0., 9.81, 0.};
    linear_velocity += timeStep * (forces / mass + spring_acceleration(spring_force,
                                                                       get_point_velocity(center_position)) -
        attraction_force);

    rotation += timeStep * .5 * rotation * Quat{0., angular_velocity.x, angular_velocity.y, angular_velocity.z};
    rotation = rotation.unit();

    angular_momentum += timeStep * (torque + spring_torque);
    inv_inertia_tensor = get_rotated_inertia(initial_inv_inertia_, rotation);
    angular_velocity = inv_inertia_tensor.transformVectorNormal(angular_momentum);
}

Vec3 box::get_point_velocity(const Vec3& position) const
{
    return linear_velocity + cross(angular_velocity, position);
}

void box::apply_impulse(const Vec3& impulse_normal, const Vec3& position)
{
    linear_velocity += impulse_normal / mass;
    angular_momentum += cross(position, impulse_normal);
}

RigidBodySystemSimulator::RigidBodySystemSimulator()
    : Simulator(), m_mouse(), m_trackmouse(), m_oldtrackmouse()
{
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Default";
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
    for (const auto& spring : springs_)
    {
        DUC->beginLine();
        DUC->drawLine(bodies_.at(spring.point1).center_position, Vec3{0., 0., 1.},
                      bodies_.at(spring.point2).center_position, Vec3{0., 1., 1.});
        DUC->endLine();
    }
    DUC->setUpLighting(Vec3{}, Vec3{.4, .6, .6}, 100., Vec3{.2, .2, .2});
    for (const auto& plane : planes_)
    {
        DUC->drawRigidBody(plane.get_transform());
    }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;

    if (testCase == 0)
    {
        set_up_complex();
    }
    else
    {
        throw std::runtime_error{"Illegal testcase"};
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    compute_spring_forces();

    for (auto& body : bodies_)
    {
        body.simulate_step(timeStep);
    }

    // Collision Detection
    for (int i = 0; i < getNumberOfRigidBodies(); i++)
    {
        for (int j = i + 1; j < getNumberOfRigidBodies(); j++)
        {
            Mat4 body_a{bodies_[i].get_transform()};
            Mat4 body_b{bodies_[j].get_transform()};
            CollisionInfo collision = check_collision_safe(body_a, body_b);
            if (collision.isValid)
            {
                collide_bodies(i, j, collision.collisionPointWorld, collision.normalWorld);
            }
        }
        for (const auto& plane : planes_)
        {
            Mat4 body{bodies_[i].get_transform()};
            Mat4 plane_mat{plane.get_transform()};
            CollisionInfo collision{check_collision_safe(body, plane_mat)};
            if (collision.isValid)
            {
                plane.collide_box(bodies_[i], collision.collisionPointWorld);
            }
        }
    }
}

void RigidBodySystemSimulator::collide_bodies(int a, int b, const Vec3& collision_point, const Vec3& normal)
{
    const Vec3 xa = collision_point - bodies_[a].center_position;
    const Vec3 xb = collision_point - bodies_[b].center_position;
    const Vec3 relative_velocity = bodies_[a].get_point_velocity(xa) - bodies_[b].get_point_velocity(xb);

    const Vec3 inertia_a = cross(bodies_[a].inv_inertia_tensor.transformVector(cross(xa, normal)), xa);
    const Vec3 inertia_b = cross(bodies_[b].inv_inertia_tensor.transformVector(cross(xb, normal)), xb);
    const double denominator = 1. / bodies_[a].mass + 1. / bodies_[b].mass + dot((inertia_a + inertia_b), normal);

    // c = 1
    const double impulse = std::max(0., -1.5 * dot(relative_velocity, normal) / denominator);

    const Vec3 center_of_mass{
        (bodies_[a].center_position * bodies_[a].mass + bodies_[b].center_position * bodies_[b].mass) / (bodies_[a].mass
            + bodies_[b].mass)
    };
    const Vec3 force_point{collision_point - center_of_mass};
    bodies_[a].center_position += .01 * normal;
    bodies_[b].center_position -= .01 * normal;
    bodies_[a].apply_impulse(impulse * normal, xa);
    bodies_[b].apply_impulse(-impulse * normal, xb);
}

void RigidBodySystemSimulator::applySpringForceOnBody(int i, const Vec3& loc, const Vec3& force)
{
    bodies_[i].spring_force = force;
    bodies_[i].spring_torque = cross(loc - bodies_[i].center_position, force);
}

void RigidBodySystemSimulator::compute_spring_force(const spring& spring)
{
    auto& b1 = bodies_.at(spring.point1);
    auto& b2 = bodies_.at(spring.point2);
    const Vec3 direction{b1.center_position - b2.center_position};
    const double current_length{norm(direction)};
    const Vec3 force{spring.stiffness * (current_length - spring.initial_length) / current_length * direction};
    applySpringForceOnBody(spring.point1, b1.center_position, -force);
    applySpringForceOnBody(spring.point2, b2.center_position, force);
}

void RigidBodySystemSimulator::compute_spring_forces()
{
    for (const auto& spring : springs_)
    {
        compute_spring_force(spring);
    }
}

void RigidBodySystemSimulator::print_solution()
{
    std::cout << "Linear Velocity: " << getLinearVelocityOfRigidBody(0) << " Angular velocity: " <<
        getAngularVelocityOfRigidBody(0) << "\n";
    std::cout << "Velocity of Point (-0.3, -0.5, -0.25): " << bodies_[0].get_point_velocity(
        Vec3{-0.3, -0.5, -0.25} - bodies_[0].center_position) << std::endl;
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;

    interact();
}

void RigidBodySystemSimulator::interact()
{
    const Vec3 click_position{
        2. * m_trackmouse.x / DUC->screenWidth - 1., -2. * m_trackmouse.y / DUC->screenHeight + 1., 0.
    };
    std::cout << "Clicked at " << click_position << std::endl;

    Mat4 p_mat{DUC->g_camera.GetProjMatrix()};
    Mat4 v_mat{DUC->g_camera.GetViewMatrix()};
    Mat4 w_mat{DUC->g_camera.GetWorldMatrix()};
    const Vec3 world_click_position{
        w_mat.inverse().transformVector(
            v_mat.inverse().transformVector(p_mat.inverse().transformVector(click_position)))
    };
    std::cout << "Clicked at " << world_click_position << std::endl;

    auto position{DUC->g_camera.GetEyePt()};
    const Vec3 camera_position{position};
    Vec3 camera_direction{world_click_position - camera_position};
    normalize(camera_direction);

    click_on_box(world_click_position, camera_direction);
}

void RigidBodySystemSimulator::click_on_box(const Vec3& clickPosition, const Vec3& direction)
{
    int nearest = -1;
    float min_distance = INFINITY;
    Vec3 collision_point{0.};
    
    Vec3 rot = cross(Vec3{0., 0., 1.}, direction);
    normalize(rot);
    const Quat ray_direction{rot, std::acos(dot(Vec3{0., 0., 1.}, direction))};

    const box ray{clickPosition, Vec3{0.01, 0.01, 100.0}, ray_direction.unit(), Vec3{0.}, Vec3{0.}, 1};
    //bodies_.emplace_back(ray);

    for (int i = 0; i < getNumberOfRigidBodies(); i++)
    {
        CollisionInfo info = checkCollisionSAT(ray.get_transform(), bodies_[i].get_transform());
        if (info.isValid)
        {
            float distance = norm((info.collisionPointWorld - clickPosition));
            if (distance < min_distance)
            {
                nearest = i;
                min_distance = distance;
                collision_point = info.collisionPointWorld;
            }
        }
    }

    if (nearest > -1)
    {
        std::cout << "Collision with " << nearest << " at " << collision_point << std::endl;
        bodies_[nearest].apply_impulse(10 * direction, collision_point);
    }
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

void RigidBodySystemSimulator::applyForceOnBody(int i, const Vec3& loc, const Vec3& force)
{
    bodies_[i].forces += force;
    bodies_[i].torque += cross(loc - bodies_[i].center_position, force);
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
    applyForceOnBody(0, Vec3{.3, .5, .25}, Vec3{1., 1., 0.});
}

void RigidBodySystemSimulator::set_up_two_body()
{
    bodies_.clear();

    bodies_.emplace_back(Vec3{-1., 0., 0.}, Vec3{0.5, 0.5, 0.5}, Quat{Vec3{0., 0., 1.}, 0.}, Vec3{0.5, 0., 0.},
                         Vec3{0.}, 5.);
    bodies_.emplace_back(Vec3{0.5, 0., 0.}, Vec3{0.3, 0.3, 0.3}, Quat{Vec3{1., 1., 1.}, 0.5 * pi_half},
                         Vec3{-0.25, 0., 0.}, Vec3{0.}, 1.);
}

void RigidBodySystemSimulator::set_up_complex()
{
    bodies_.clear();
    springs_.clear();

    bodies_.emplace_back(Vec3{-1., 0., -1.}, Vec3{0.5, 0.5, 0.5}, Quat{Vec3{0., 0., 1.}, 0.}, Vec3{1., 0., .5},
                         Vec3{0.}, 3.);
    bodies_.emplace_back(Vec3{0., 1., 0.}, Vec3{0.3, 0.6, 0.3}, Quat{Vec3{1., 0., 0.}, 0.5 * pi_half}, Vec3{1., 0., .5},
                         Vec3{0.}, 3.);
    bodies_.emplace_back(Vec3{0., 0., 0.}, Vec3{0.7, 0.5, 0.5}, Quat{Vec3{0., 0., 1.}, 0.}, Vec3{0.5, 0., 0.}, Vec3{0.},
                         3.);
    bodies_.emplace_back(Vec3{1., 0., 1.}, Vec3{0.2, 0.5, 0.2}, Quat{Vec3{1., 0., 1.}, 0.5 * pi_half},
                         Vec3{-0.5, 0., -0.5}, Vec3{0.}, 3.);

    springs_.emplace_back(0, 1, 3.f, 40.f);
    springs_.emplace_back(2, 3, 1.f, 40.f);
    springs_.emplace_back(1, 2, 2.f, 30.f);
    springs_.emplace_back(3, 0, 3.f, 50.f);

    planes_.emplace_back(Vec3{0., -.5, 0.}, Vec3{0., 1., 0.});
}
