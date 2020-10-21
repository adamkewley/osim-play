#include "Simbody.h"

#include <fstream>
#include <regex>

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

static const char lhs_arg[] = "--lhs=";
static const char rhs_arg[] = "--rhs=";
static const char geometry_arg[] = "--geometry=";
static const char wrapping_quadrant_arg[] = "--wrapping-quadrant=";
static const char final_time_arg[] = "--final-time=";
static const char record_arg[] = "--record-to=";
static const char no_viz_arg[] = "--no-visualizer";
static const char help_arg[] = "--help";
static const char appname[] = "simbody_wrapping";
static const char usage[] =
R"(usage: simbody_wrapping [--lhs=[<x>,<y>,<z>]] [--rhs=[<x>,<y>,<z>]]
                        [--geometry=<geometry>] [--wrapping-quadrant=<direction>]
                        [--final-time=<secs>] [--record-to=<path>]
                        [--no-visualizer] [--help])";
static const char help[] =
R"(OPTIONS
    --lhs=[<x>,<y>,<z>]
        Coordinates of the left-hand vertical slider (default: [-0.4, 0.0, 0.0])

    --rhs=[<x>,<y>,<z>]
        Coordinates of the right-hand vertical slider (default: [+0.4, 0.0, 0.0])

    --geometry=<geometry>
        Specify wrapping geometry. Allowed <geometry> inputs:

            cylinder[<radius>]
            ellipsoid[<x>,<y>,<z>]

        Defaults to: cylinder[0.08]

    --wrapping-quadrant=<quadrant>
        IGNORED: only used by OpenSim backend

    --final-time=<secs>
        Set the final simulation time in seconds (default: 10)

    --record-to=<path>
        Record a CSV containing the positions of the sliding masses to <path>

    --no-visualizer
        Disable the GUI visualizer

    --help
        Show this help
)";

int main(int argc, char** argv) {
    double final_time = 10.0;
    auto lhs_offset = Vec3{-0.4, 0.0, 0.0};
    auto rhs_offset = Vec3{+0.4, 0.0, 0.0};
    std::string record_to = "";
    bool visualize = true;
    double obstacle_radius = 0.08;
    std::unique_ptr<ContactGeometry> geometry =
        std::make_unique<ContactGeometry::Cylinder>(obstacle_radius);
    auto contact_hint_lhs = Vec3{-obstacle_radius, 0.001, 0.0};
    auto contact_hint_rhs = Vec3{+obstacle_radius, 0.001, 0.0};

    // basic CLI parsing
    {
        char** args = argv + 1;
        for (int nargs = argc - 1; nargs > 0; --nargs, ++args) {
            char const* arg = *args;

            if (strncmp(arg, lhs_arg , sizeof(lhs_arg)-1) == 0) {
                char const* val = arg + (sizeof(lhs_arg)-1);
                std::cmatch m;
                std::regex patt{R"(\[([^,]+),([^,]+),([^\]]+)\])"};

                if (std::regex_match(val, m, patt)) {
                    double x = safe_parse_double(m[1].str().c_str());
                    double y = safe_parse_double(m[2].str().c_str());
                    double z = safe_parse_double(m[3].str().c_str());
                    lhs_offset = Vec3{x, y, z};
                } else {
                    std::cerr << appname << ": invalid coordiate specified for '" << arg << "'" << std::endl;
                    return -1;
                }
            } else if (strncmp(arg, rhs_arg, sizeof(rhs_arg)-1) == 0) {
                char const* val = arg + (sizeof(rhs_arg)-1);
                std::cmatch m;
                std::regex patt{R"(\[([^,]+),([^,]+),([^\]]+)\])"};

                if (std::regex_match(val, m, patt)) {
                    double x = safe_parse_double(m[1].str().c_str());
                    double y = safe_parse_double(m[2].str().c_str());
                    double z = safe_parse_double(m[3].str().c_str());
                    rhs_offset = Vec3{x, y, z};
                } else {
                    std::cerr << appname << ": invalid coordiate specified for '" << arg << "'" << std::endl;
                    return -1;
                }
            } else if (strncmp(arg, geometry_arg, sizeof(geometry_arg)-1) == 0) {
                char const* val = arg + sizeof(geometry_arg)-1;
                std::cmatch m;

                std::regex cylinder_patt{R"(cylinder\[([^\]]+)\])"};
                std::regex ellipsoid_patt{R"(ellipsoid\[([^,]+),([^,]+),([^\]]+)\])"};

                if (std::regex_match(val, m, cylinder_patt)) {
                    double r = safe_parse_double(m[1].str().c_str());

                    geometry = std::make_unique<ContactGeometry::Cylinder>(r);
                    contact_hint_lhs = Vec3{-r, 0.001, 0.0};
                    contact_hint_rhs = Vec3{+r, 0.001, 0.0};
                } else if (std::regex_match(val, m, ellipsoid_patt)) {
                    double x = safe_parse_double(m[1].str().c_str());
                    double y = safe_parse_double(m[2].str().c_str());
                    double z = safe_parse_double(m[3].str().c_str());

                    geometry = std::make_unique<ContactGeometry::Ellipsoid>(Vec3{x, y, z});
                    contact_hint_lhs = Vec3{-x, 0.001, 0.0};
                    contact_hint_rhs = Vec3{+x, 0.001, 0.0};
                } else {
                    std::cerr << appname << ": unrecognized geometry argument for '" << arg << "'" << std::endl;
                    return -1;
                }
            } else if (strncmp(arg, wrapping_quadrant_arg, sizeof(wrapping_quadrant_arg)-1) == 0) {
                std::cerr << appname << " WARNING: " << arg << ": not used by simbody backend";
            } else if (strncmp(arg, final_time_arg, sizeof(final_time_arg)-1) == 0) {
                char const* val = arg + (sizeof(final_time_arg)-1);
                final_time = safe_parse_double(val);
            } else if (strncmp(arg, record_arg, sizeof(record_arg)-1) == 0)  {
                record_to = std::string{arg + (sizeof(record_arg)-1)};
            } else if (strcmp(arg, no_viz_arg) == 0) {
                visualize = false;
            } else if (strcmp(arg, help_arg) == 0) {
                std::cout << usage << std::endl;
                std::cout << help << std::endl;
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
        Transform{slider_orientation, lhs_offset},
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
        Transform{slider_orientation, rhs_offset},
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

    auto obstacle_surface = CableObstacle::Surface{
        cable,
        matter.Ground(),
        Transform{Rotation{}, Vec3{0.0, 1.0, 0.0}},
        *geometry
    };
    // obstacles *require* contact point hints so that the wrapping cable
    // knows how to start wrapping over it
    obstacle_surface.setContactPointHints(contact_hint_lhs, contact_hint_rhs);


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
