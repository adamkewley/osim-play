#include "Simbody.h"

#include <fstream>

using SimTK::MultibodySystem;
using SimTK::SimbodyMatterSubsystem;
using SimTK::GeneralForceSubsystem;
using SimTK::GeneralContactSubsystem;
using SimTK::CableTrackerSubsystem;
using SimTK::Force;
using SimTK::Vec3;
using SimTK::Inertia;
using SimTK::Body;
using SimTK::MassProperties;
using SimTK::Visualizer;
using SimTK::State;
using SimTK::RungeKuttaMersonIntegrator;
using SimTK::MobilizedBody;
using SimTK::CablePath;
using SimTK::Transform;
using SimTK::CableObstacle;
using SimTK::Rotation;
using SimTK::Pi;
using SimTK::ContactGeometry;
using SimTK::YAxis;
using SimTK::ZAxis;
using SimTK::Constraint;
using SimTK::PeriodicEventReporter;
using SimTK::Real;

struct Position_recorder final : public PeriodicEventReporter {
    MultibodySystem const& mbs;
    MobilizedBody::Slider const& lhs;
    MobilizedBody::Slider const& rhs;
    mutable std::ofstream ofile;  // mutable because SimTK lol

    Position_recorder(Real interval,
                      std::string path,
                      MultibodySystem const& _mbs,
                      MobilizedBody::Slider const& _lhs,
                      MobilizedBody::Slider const& _rhs) :
        PeriodicEventReporter{interval},
        mbs{_mbs},
        lhs{_lhs},
        rhs{_rhs},
        ofile{[&]() {
            auto f = std::ofstream{};
            f.exceptions(std::ofstream::failbit);
            f.open(path);
            return f;
        }()} {

        ofile << "time,lhs,rhs" << std::endl;
    }

    void handleEvent(const State& s) const override {
        ofile << s.getTime() << ","
              << lhs.getQ(s) << ","
              << rhs.getQ(s) << std::endl;
    }
};

static double safe_parse_double(char const* s) {
    char* end;
    double v = strtod(s, &end);
    if (*end != '\0' or v == HUGE_VAL or v == -HUGE_VAL) {
        std::stringstream ss;
        ss << "numeric parsing error: could not parse '" << s << "' as a double";
        throw std::runtime_error{ss.str()};
    }
    return v;
}

static const char ox_arg[] = "--offset-x=";
static const char oy_arg[] = "--offset-y=";
static const char oz_arg[] = "--offset-z=";
static const char record_arg[] = "--record-to=";
static const char no_viz_arg[] = "--no-visualizer";
static const char appname[] = "joris_in-simbody";
static const char help_arg[] = "--help";
static const char usage[] =
R"(usage : joris_in-simbody [--offset-x=<val>] [--offset-y=<val>] [--offset-z=<val>]
                         [--record-to=<path>] [--no-visualizer] [--help])";

int main(int argc, char** argv) {
    auto body_offset = Vec3{0.4, 0.0, 0.0};
    std::string record_to = "";
    bool visualize = true;

    // basic CLI parsing
    {
        char** args = argv + 1;
        for (int nargs = argc - 1; nargs > 0; --nargs, ++args) {
            char const* arg = *args;

            if (strncmp(arg, ox_arg , sizeof(ox_arg)-1) == 0) {
                char const* val = arg + (sizeof(ox_arg)-1);
                double v = safe_parse_double(val);
                body_offset.set(0, v);
            } else if (strncmp(arg, oy_arg, sizeof(oy_arg)-1) == 0) {
                char const* val = arg + (sizeof(oy_arg)-1);
                double v = safe_parse_double(val);
                body_offset.set(1, v);
            } else if (strncmp(arg, oz_arg, sizeof(oz_arg)-1) == 0) {
                char const* val = arg + (sizeof(oz_arg)-1);
                double v = safe_parse_double(val);
                body_offset.set(2, v);
            } else if (strncmp(arg, record_arg, sizeof(record_arg)-1) == 0)  {
                record_to = std::string{arg + (sizeof(record_arg)-1)};
            } else if (strcmp(arg, no_viz_arg) == 0) {
                visualize = false;
            } else if (strcmp(arg, help_arg) == 0) {
                std::cout << usage << std::endl;
                return 0;
            } else {
                std::cerr << appname << ": unrecognized argument '" << arg << "'" << std::endl;
                std::cerr << std::endl;
                std::cerr << usage << std::endl;
                return -1;
            }
        }
    }

    auto system = MultibodySystem{};
    auto matter = SimbodyMatterSubsystem{system};
    auto forces = GeneralForceSubsystem{system};
    auto contact = GeneralContactSubsystem{system};
    auto cables = CableTrackerSubsystem{system};

    auto gravity = Force::UniformGravity{
        forces,
        matter,
        Vec3{0.0, -9.80665, 0.0},
    };

    double body_mass = 30.0;
    double body_side_len = 0.1;
    auto center_of_mass = Vec3{0.0, 0.0, 0.0};
    auto body_inertia = body_mass * Inertia::brick(Vec3{body_side_len / 2.0});
    auto slider_orientation = Rotation{Pi/2.0, ZAxis};

    // left mass
    auto body_left = Body::Rigid{
        MassProperties{body_mass, center_of_mass, body_inertia},
    };
    // decorate masses as cubes
    body_left.addDecoration(
        SimTK::Transform{},
        SimTK::DecorativeBrick{Vec3{body_side_len / 2.0}}.setColor({0.8, 0.1, 0.1})
    );
    auto slider_left = MobilizedBody::Slider{
        matter.Ground(),
        Transform{slider_orientation, -body_offset},
        body_left,
        Transform{slider_orientation, Vec3{0.0, 0.0, 0.0}},
    };
    slider_left.setDefaultQ(0.5);  // simbody equivalent to the coordinate bs
    auto spring_to_left = SimTK::Force::TwoPointLinearSpring{
            forces,
            matter.Ground(),
            Vec3(0),
            slider_left,
            Vec3(0, -body_side_len / 2, 0),
            100,
            0.5
    };



    // right mass
    auto body_right = Body::Rigid{
        MassProperties{body_mass, center_of_mass, body_inertia},
    };
    body_right.addDecoration(
        SimTK::Transform{},
        SimTK::DecorativeBrick{Vec3{body_side_len / 2.0}}.setColor({0.8, 0.1, 0.1})
    );
    auto slider_right = MobilizedBody::Slider{
        matter.Ground(),
        Transform{slider_orientation, body_offset},
        body_right,
        Transform{slider_orientation, Vec3{0.0, 0.0, 0.0}},
    };
    slider_right.setDefaultQ(0.5);  // simbody equivalent to the coordinate bs
    auto spring_to_right = SimTK::Force::TwoPointLinearSpring{
            forces,
            matter.Ground(),
            Vec3(0),
            slider_right,
            Vec3(0, -body_side_len / 2, 0),
            100,
            0.5
    };


    // cable path
    auto cable = CablePath{
        cables,
        slider_left,
        Vec3{0},
        slider_right,
        Vec3{0},
    };
    SimTK::CableSpring cable2(forces, cable, 50., 1.0, 0.1);


    // cable obstacle: a sphere that is fixed to the ground at some offset


    // obstacle in cable path
    double obstacle_radius = 0.08;
    auto obstacle_surface = CableObstacle::Surface{
        cable,
        matter.Ground(),
        Transform{Rotation{}, Vec3{0.0, 1.0, 0.0}},
        ContactGeometry::Cylinder{obstacle_radius},
    };
    // obstacles *require* contact point hints so that the wrapping cable
    // knows how to start wrapping over it
    obstacle_surface.setContactPointHints(
        // lhs
        Vec3{-obstacle_radius, 0.001, 0.0},
        // rhs
        Vec3{+obstacle_radius, 0.001, 0.0}
    );


    // set up visualization to match OpenSim (but without OpenSim) see:
    //     OpenSim: ModelVisualizer.cpp + SimulationUtilities.cpp
    system.setUseUniformBackground(true);

    if (not record_to.empty()) {
        system.addEventReporter(new Position_recorder{
                                    0.01,
                                    record_to,
                                    system,
                                    slider_left,
                                    slider_right
                                });
    }

    if (visualize) {
        auto visualizer = Visualizer{system};
        system.addEventReporter(new Visualizer::Reporter{
                                    visualizer,
                                    0.01
                                });
        visualizer.setShowFrameRate(true);
        auto silo = SimTK::Visualizer::InputSilo{};
        visualizer.addInputListener(&silo);
        auto help =
            SimTK::DecorativeText("Press any key to start a new simulation; ESC to quit.");
        help.setIsScreenText(true);
        visualizer.addDecoration(SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);
        visualizer.setShowSimTime(true);
        visualizer.setMode(Visualizer::RealTime);

        // set up system
        system.realizeTopology();
        State s = system.getDefaultState();
        visualizer.drawFrameNow(s);

        while (true) {
            silo.clear(); // Ignore any previous key presses.
            unsigned key, modifiers;
            silo.waitForKeyHit(key, modifiers);
            if (key == SimTK::Visualizer::InputListener::KeyEsc) {
                break;
            }
            // set up simulation (integrator etc.)
            auto integrator = RungeKuttaMersonIntegrator{system};
            auto time_stepper = SimTK::TimeStepper{system, integrator};
            time_stepper.initialize(s);
            time_stepper.stepTo(10.0);
        }
    } else {
        system.realizeTopology();
        State s = system.getDefaultState();
        auto integrator = RungeKuttaMersonIntegrator{system};
        auto time_stepper = SimTK::TimeStepper{system, integrator};
        time_stepper.initialize(s);
        time_stepper.stepTo(10.0);
    }

    return 0;
}