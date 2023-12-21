#include "DiffusionSimulator.h"
#include "pcgsolver.h"

void Grid::clear_grid_values()
{
    std::fill(temperatures_.begin(), temperatures_.end(), 0.);
}

void Grid::set_grid_value(const int x, const int y, const double value)
{
    get_grid_value_ref(x, y) = value;
}

double Grid::get_grid_value(const int x, const int y) const
{
    return temperatures_.at(x + y * width);
}

double Grid::get_color_value(const int i, const int j) const
{
    const double temperature{get_grid_value(i, j)};
    constexpr double bias{0.};
    constexpr double scale{200.};
    return std::min(std::max((temperature + bias) / scale, 0.), 1.);
}

double Grid::laplace(const int i, const int j) const
{
    const double x_le{i > 0 ? get_grid_value(i - 1, j) : 0.};
    const double x_ge{i < width - 1 ? get_grid_value(i + 1, j) : 0.};
    const double y_le{j > 0 ? get_grid_value(i, j - 1) : 0.};
    const double y_ge{j < length - 1 ? get_grid_value(i, j + 1) : 0.};
    return x_le + x_ge + y_le + y_ge - 4. * get_grid_value(i, j);
}

double& Grid::get_grid_value_ref(const int x, const int y)
{
    return temperatures_.at(x + y * width);
}

DiffusionSimulator::DiffusionSimulator() : grid_{16, 16}
{
    m_iTestCase = 0;
    m_vfMovableObjectPos = Vec3();
    m_vfMovableObjectFinalPos = Vec3();
    m_vfRotate = Vec3();
    // rest to be implemented
}

const char* DiffusionSimulator::getTestCasesStr()
{
    return "Explicit,Implicit";
}

void DiffusionSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    // to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    m_vfMovableObjectPos = Vec3(0, 0, 0);
    m_vfRotate = Vec3(0, 0, 0);
    //
    // to be implemented
    //
    set_up_scene();

    switch (m_iTestCase)
    {
    case 0:
        std::cout << "Explicit solver!\n";
        break;
    case 1:
        std::cout << "Implicit solver!\n";
        break;
    default:
        std::cout << "Empty Test!\n";
        break;
    }
}

void DiffusionSimulator::set_up_scene()
{
    grid_.clear_grid_values();
    // insert some values
    grid_.set_grid_value(10, 10, 1000.);
    grid_.set_grid_value(11, 10, 1000.);
    grid_.set_grid_value(10, 11, 1000.);
    grid_.set_grid_value(11, 11, 1000.);
    grid_.set_grid_value(1, 1, 10000.);

}

void DiffusionSimulator::diffuseTemperatureExplicit(double timeStep)
{
    std::vector<double> deltas(grid_.width * grid_.length);
    for (int i = 0; i < grid_.width; ++i)
    {
        for (int j = 0; j < grid_.length; ++j)
        {
            deltas[i + j * grid_.width] = grid_.laplace(i, j);
        }
    }
    for (int i = 0; i < grid_.width; ++i)
    {
        for (int j = 0; j < grid_.length; ++j)
        {
            grid_.set_grid_value(i, j, deltas[i + j * grid_.width] * timeStep + grid_.get_grid_value(i, j));
        }
    }

    zero_boundary();
}


void DiffusionSimulator::diffuseTemperatureImplicit(double timeStep)
{
    // solve A T = b

    // This is just an example to show how to work with the PCG solver,
    const int nx = 5;
    const int ny = 5;
    const int nz = 5;
    const int N = nx * ny * nz;

    SparseMatrix<Real> A(N);
    std::vector<Real> b(N);

    // This is the part where you have to assemble the system matrix A and the right-hand side b!

    // perform solve
    Real pcg_target_residual = 1e-05;
    Real pcg_max_iterations = 1000;
    Real ret_pcg_residual = 1e10;
    int ret_pcg_iterations = -1;

    SparsePCGSolver<Real> solver;
    solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

    std::vector<Real> x(N);
    for (int j = 0; j < N; ++j) { x[j] = 0.; }

    // preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
    solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

    // Final step is to extract the grid temperatures from the solution vector x
    // to be implemented
}


void DiffusionSimulator::simulateTimestep(float timeStep)
{
    // update current setup for each frame
    switch (m_iTestCase)
    {
    case 0:
        diffuseTemperatureExplicit(timeStep);
        break;
    case 1:
        diffuseTemperatureImplicit(timeStep);
        break;
    default:
        throw std::exception{"Illegal test case"};
    }
}

void DiffusionSimulator::externalForcesCalculations(float timeElapsed)
{
}

void DiffusionSimulator::drawObjects()
{
    //visualization
    const double step_x = 1. / (grid_.width - 1);
    const double step_y = 1. / (grid_.length - 1);

    for (int i = 0; i < grid_.width; i++)
    {
        for (int j = 0; j < grid_.length; j++)
        {
            const double color{grid_.get_color_value(i, j)};
            DUC->setUpLighting(Vec3(0.), Vec3(1.), 10000000., Vec3(1., color, color));
            this->DUC->drawSphere(Vec3{-0.5 + i * step_x, color, -0.5 + j * step_y}, Vec3{0.05});
        }
    }
}

void DiffusionSimulator::zero_boundary()
{
    for (int i = 0; i < grid_.width; ++i)
    {
        grid_.set_grid_value(i, 0, 0.);
        grid_.set_grid_value(i, grid_.length - 1, 0.);
    }
    for (int i = 0; i < grid_.length; ++i)
    {
        grid_.set_grid_value(0, i, 0.);
        grid_.set_grid_value(grid_.width - 1, i, 0.);
    }
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}
