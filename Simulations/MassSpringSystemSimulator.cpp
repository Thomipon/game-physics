#include "MassSpringSystemSimulator.h"
#include <array>

void mass_point::process_collision(const plane& plane, const float radius)
{
    if(abs(dot(position - plane.position, plane.normal)) < radius)
    {
        velocity = .99 * reflectVector(velocity, plane.normal);
    }
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "2 point 1 step, 2 point simulation, complex simulation";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;

    TwAddVarRW(DUC->g_pTweakBar, "IntegrationMethod",
               TwDefineEnumFromString("Integration Method", "euler, leapfrog, midpoint"), &m_iIntegrator, "");
    TwAddSeparator(DUC->g_pTweakBar, "Interaction", "");
    TwAddVarRW(DUC->g_pTweakBar, "Index", TW_TYPE_INT32, &yeeted_index_, "");
    TwAddButton(DUC->g_pTweakBar, "YEET!", [](void* clientData)
    {
        if (auto* self = static_cast<MassSpringSystemSimulator*>(clientData)) {
            self->yeet();
        }
        
    }, this, "");
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
        DUC->drawSphere(mass_point.position, Vec3{radius_});
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
        simulateTimestep(.005);
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

void MassSpringSystemSimulator::resolve_collision()
{
    for (auto& mass_point : mass_points_)
    {
        mass_point.process_collision(collision_plane_, radius_);
    }
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
    static bool is_leapfrog{false};
    if (!only_first_ || is_first_frame_)
    {
        if (is_leapfrog != (m_iIntegrator == integration_method::leapfrog))
        {
            if (is_leapfrog)
            {
                prepare_leapfrog(-0.5f * timeStep);
            }
            else
            {
                prepare_leapfrog(0.5f * timeStep);
            }
            is_leapfrog = !is_leapfrog;
        }
        compute_all_forces();

        switch (m_iIntegrator)
        {
        case integration_method::euler:
            simulate_euler(timeStep);
            break;
        case integration_method::leapfrog:
            simulate_leapfrog(timeStep);
            break;
        case integration_method::midpoint:
            simulate_midpoint(timeStep);
            break;
        default:
            throw std::runtime_error{"Illegal integrator!"};
        }

        resolve_collision();
    }
    if (only_first_ && is_first_frame_)
    {
        print_state();
        is_first_frame_ = false;
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

void MassSpringSystemSimulator::yeet()
{
    if (yeeted_index_ < 0 || yeeted_index_ >= getNumberOfMassPoints())
        return;
    mass_points_[yeeted_index_].velocity += 10. * Vec3{0., 1., 0.};
}

void MassSpringSystemSimulator::set_up_simple_case()
{
    is_first_frame_ = true;

    mass_points_.clear();
    mass_points_.reserve(2);

    const auto mp1{addMassPoint(Vec3{0., 0., 0.}, Vec3{-1., 0., 0.}, false)};
    const auto mp2{addMassPoint(Vec3{0., 2., 0.}, Vec3{1., 0., 0.}, false)};

    setMass(10);
    setStiffness(40);

    springs_.clear();
    addSpring(mp1, mp2, 1.);

    applyExternalForce(Vec3{0., 0., 0.});
    setDampingFactor(0.);
}

void MassSpringSystemSimulator::set_up_complex_case()
{
    is_first_frame_ = true;
    
    constexpr int num_spheres = 10;

    mass_points_.clear();
    mass_points_.reserve(num_spheres);

    const std::array<int, num_spheres> mass_points{
        addMassPoint(Vec3{0., 0., 0.}, Vec3{-1., 0., 0.}, false),
        addMassPoint(Vec3{0., 2., 0.}, Vec3{1., 0., 0.}, false),
        addMassPoint(Vec3{0., 4., 0.}, Vec3{1., 1., 0.}, false),
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

    applyExternalForce(Vec3{0., -9.81, 0.});
    setDampingFactor(1.);
}

void MassSpringSystemSimulator::print_state() const
{
    for (int i = 0; i < getNumberOfMassPoints(); ++i)
    {
        std::cout << "Point " << i << ":\t" << mass_points_[i].position << "\t" << mass_points_[i].velocity << "\n";
    }
    std::cout << std::endl;
}

void MassSpringSystemSimulator::compute_spring_force(const spring& spring)
{
    auto& mp1 = mass_points_.at(spring.point1);
    auto& mp2 = mass_points_.at(spring.point2);
    const Vec3 direction{mp1.position - mp2.position};
    const double current_length{norm(direction)};
    Vec3 force{m_fStiffness * (current_length - spring.initial_length) / current_length * direction};
    mp1.force -= force;
    mp2.force += std::move(force);
}

void MassSpringSystemSimulator::compute_all_forces()
{
    for (auto& mass_point : mass_points_)
    {
        mass_point.force = m_externalForce;
    }
    
    for (const auto& spring : springs_)
    {
        compute_spring_force(spring);
    }
}

Vec3 MassSpringSystemSimulator::compute_acceleration(const Vec3& force, const Vec3& velocity) const
{
    return  1./ m_fMass * (force - m_fDamping * velocity);
}

void MassSpringSystemSimulator::simulate_euler(const float time_step)
{
    for (auto& mass_point : mass_points_)
    {
        mass_point.position += time_step * mass_point.velocity;
        mass_point.velocity += time_step * compute_acceleration(mass_point.force, mass_point.velocity);
    }
}

void MassSpringSystemSimulator::simulate_midpoint(const float time_step)
{
    std::vector<std::tuple<Vec3, Vec3>> temp_values(mass_points_.size());
    std::transform(mass_points_.cbegin(), mass_points_.cend(), temp_values.begin(), [](const mass_point& mass_point)
    {
        return std::make_tuple(
            mass_point.position,
            mass_point.velocity);
    });

    for (auto& mass_point : mass_points_)
    {
        mass_point.position += time_step / 2 * mass_point.velocity;
        mass_point.velocity += time_step / 2 * compute_acceleration(mass_point.force, mass_point.velocity);
    }

    compute_all_forces();

    for (int i = 0; i < mass_points_.size(); ++i)
    {
        mass_points_[i].position = std::get<0>(temp_values[i]) + time_step * mass_points_[i].velocity;
        mass_points_[i].velocity = std::get<1>(temp_values[i]) + time_step * compute_acceleration(mass_points_[i].force, mass_points_[i].velocity);
    }
}

void MassSpringSystemSimulator::simulate_leapfrog(const float time_step)
{
    for (auto& mass_point : mass_points_)
    {
        mass_point.velocity += time_step * compute_acceleration(mass_point.force, mass_point.velocity);
        mass_point.position += time_step * mass_point.velocity;
    }
}

void MassSpringSystemSimulator::prepare_leapfrog(const float half_timestep)
{
    compute_all_forces();
    for (auto& mass_point : mass_points_)
    {
        mass_point.velocity += half_timestep *  compute_acceleration(mass_point.force, mass_point.velocity);
    }
}
