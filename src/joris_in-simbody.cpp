#include "Simbody.h"

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


/* give the left block some motion
auto pm1 = Constraint::PrescribedMotion{
    matter,
    new SimTK::Function::Sinusoid(0.25*Pi, 1.5*Pi, 0.4*Pi),
    slider_left,
    SimTK::MobilizerQIndex(0),
}; */

int main() {
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
    auto body_offset = Vec3{0.4, 0.0, 0.0};

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
    auto visualizer = Visualizer{system};
    visualizer.setShowFrameRate(true);
    system.addEventReporter(new Visualizer::Reporter{visualizer, 0.01});
    auto silo = SimTK::Visualizer::InputSilo{};
    visualizer.addInputListener(&silo);

    auto help =
        SimTK::DecorativeText("Press any key to start a new simulation; ESC to quit.");
    help.setIsScreenText(true);
    visualizer.addDecoration(SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);
    visualizer.setShowSimTime(true);

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

    return 0;
}
