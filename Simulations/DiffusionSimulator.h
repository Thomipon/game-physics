#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

struct Grid
{
public:
    Grid(const int length, const int width)
        : length{length},
          width{width},
          temperatures_(width * length)
    {
    }

    int length;
    int width;

    void clear_grid_values();
    void set_grid_value(int x, int y, double value);
    double get_grid_value(int x, int y) const;
    double get_color_value(int i, int j) const;
    double laplace(int i, int j) const;

private:
    vector<double> temperatures_;

    double& get_grid_value_ref(int x, int y);
};


class DiffusionSimulator : public Simulator
{
public:
    // Construtors
    DiffusionSimulator();

    // Functions
    const char* getTestCasesStr() override;
    void initUI(DrawingUtilitiesClass* DUC) override;
    void reset() override;
    void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
    void notifyCaseChanged(int testCase) override;
    void simulateTimestep(float timeStep) override;

    void externalForcesCalculations(float timeElapsed) override;
    void onClick(int x, int y) override;
    void onMouse(int x, int y) override;
    // Specific Functions
    void drawObjects();

    void zero_boundary();
    // Feel free to change the signature of these functions, add arguments, etc.
    void diffuseTemperatureExplicit(double timeStep);
    void diffuseTemperatureImplicit(double timeStep);

private:
    // Attributes
    Vec3 m_vfMovableObjectPos;
    Vec3 m_vfMovableObjectFinalPos;
    Vec3 m_vfRotate;
    Point2D m_mouse;
    Point2D m_trackmouse;
    Point2D m_oldtrackmouse;
    Grid grid_;

    void set_up_scene();
};

#endif
