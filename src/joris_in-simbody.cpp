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
using SimTK::Constraint;

int main() {
    auto system = MultibodySystem{};
    auto matter = SimbodyMatterSubsystem{system};
    auto forces = GeneralForceSubsystem{system};
    auto contact = GeneralContactSubsystem{system};
    auto cables = CableTrackerSubsystem{system};

    auto gravity = Force::UniformGravity{
        forces,
        matter,
        Vec3{0.0, -9.80665, 0},
    };

    auto body_mass = 30.0;
    auto center_of_mass = Vec3{0};
    auto body_side_len = 0.1;
    auto body_half_lens = Vec3{body_side_len / 2.0};
    auto body_inertia = body_mass * Inertia::brick(body_half_lens);

    auto slider_orientation = Vec3{0.0, 0.0, Pi / 2.0};


    // left mass
    auto body_left = Body::Rigid{
        MassProperties{body_mass, center_of_mass, body_inertia},
    };
    body_left.addDecoration(
        SimTK::Transform{},
        SimTK::DecorativeBrick{Vec3{body_side_len / 2.0}}
    );
    auto body_left_offset = Vec3{0.4, 0.0, 0.0};
    auto slider_left = MobilizedBody::Slider{
        matter.Ground(),
        body_left_offset,
        body_left,
        slider_orientation,
    };
    auto pm1 = Constraint::PrescribedMotion{
        matter,
        new SimTK::Function::Sinusoid(0.25*Pi, 1.5*Pi, 0.4*Pi),
        slider_left,
        SimTK::MobilizerQIndex(0),
    };

    // right mass
    auto body_right = Body::Rigid{
        MassProperties{body_mass, center_of_mass, body_inertia},
    };
    body_right.addDecoration(
        SimTK::Transform{},
        SimTK::DecorativeBrick{Vec3{body_side_len / 2.0}}
    );
    auto body_right_offset = Vec3{-body_left_offset};
    auto slider_right = MobilizedBody::Slider{
        matter.Ground(),
        body_right_offset,
        body_right,
        Vec3{0.0, 0.0, Pi / 2.0},
    };
    auto pm2 = Constraint::PrescribedMotion{
        matter,
        new SimTK::Function::Sinusoid(0.25*Pi, Pi, -0.25*Pi),
        slider_right,
        SimTK::MobilizerQIndex(0),
    };


    // cable /w cylinder obstacle between the two bodies
    auto cylinder_rigid = Body::Rigid{
        MassProperties{1.0, center_of_mass, body_inertia},
    };
    auto cylinder_mobod = MobilizedBody::Slider{
        matter.Ground(),
        SimTK::Transform{Rotation{Pi/2.0, YAxis}, Vec3{0.0}},
        //SimTK::Transform{Vec3{0.0}},
        cylinder_rigid,
        SimTK::Transform{Vec3{0.0}},
    };
    auto cable = CablePath{
        cables,
        slider_left,
        Vec3{0},
        slider_right,
        Vec3{0},
    };
    auto cylinder_offset = Vec3{0.0, 0.0, 0.0};
    auto cylinder = CableObstacle::Surface{
        cable,
        cylinder_mobod,
        Transform{
            // cylinder seems to be +-Z for top/bottom, rotating 90deg it in
            // X puts the top in +-Y
            Rotation{0.5*Pi, SimTK::XAxis},
            cylinder_offset,
        },
        ContactGeometry::Cylinder{0.08},
    };
    cylinder.setContactPointHints(
        Vec3{
            0.01, // cylinder lhs
            1.0000000000,  // crashes if == 0 for some fucking reason
            0.0  // cylinder top/bottom: no wrapping over that
        },
        Vec3{
            0.01,  // cylinder rhs
            -1.0000000000,  // crashes if == 0 for some fucking reason
            0.0  // cylinder top/bottom: no wrapping over that
        }
    );
    auto pm3 = Constraint::PrescribedMotion{
        matter,
        new SimTK::Function::Sinusoid(0.7*Pi, 0.1*Pi, -0.25*Pi),
        cylinder_mobod,
        SimTK::MobilizerQIndex(0),
    };




/*

    auto body_right = Body::Rigid{
        MassProperties{body_mass, center_of_mass, body_inertia},
    };
    auto body_ground = Body::Rigid{
        MassProperties{1, center_of_mass, body_inertia},
    };
    */


    // set up visualization
    system.setUseUniformBackground(true);
    auto visualizer = Visualizer{system};
    visualizer.setShowFrameRate(true);
    system.addEventReporter(new Visualizer::Reporter{visualizer, 0.01});

    // set up system
    system.realizeTopology();
    State s = system.getDefaultState();

    // set up simulation (integrator etc.)
    auto integrator = RungeKuttaMersonIntegrator{system};
    auto time_stepper = SimTK::TimeStepper{system, integrator};
    time_stepper.initialize(s);
    time_stepper.stepTo(50.0);

    /*






  // the Body class represents physical properties of a body
  // (e.g. mass and moment of inertia)

  // pendulum's physical properties:
  //    mass: 1 Kg
  //    center of mass: [0, 0, 0]
  //    moment of inertia: 1 kg.m^2 (all 3 rotational axes)
  Body::Rigid pendulumBody(MassProperties(1.0, Vec3(0), Inertia(1)));

  // how the body (pendulum) graphically appears:
  //    A sphere of radius 0.1
  pendulumBody.addDecoration(Transform(), DecorativeSphere(0.1));

  // MobilizedBody combines the body's physical properties with
  // mobilities (i.e. state vars describing how it is *allowed* to
  // move). "Mobilizer" is any **joint** that connects a body to its
  // parent in a multibody tree. A pin mobilizer has one generalized
  // coordinate and one generalized speed

  // pendulum1
  //    a mobilized body (pin mobilizer)
  //    connected to matter.Ground() body (the "root" body)
  //    at location [0, 0, 0]
  MobilizedBody::Pin pendulum1(matter.Ground(), Transform(Vec3(0)),
                               pendulumBody, Transform(Vec3(1, -1, 0)));
  MobilizedBody::Pin pendulum2(pendulum1, Transform(Vec3(0)),
                               pendulumBody, Transform(Vec3(1, 1, 0)));
  
  // Set up visualization.
  system.setUseUniformBackground(true);
  Visualizer viz(system);
  system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
  // Initialize the system and state.
  system.realizeTopology();
  State state = system.getDefaultState();

  // set rotational velocity of the 2nd pendulum
  pendulum2.setRate(state, 50.0);
  
  // Simulate it.
  RungeKuttaMersonIntegrator integ(system);
  TimeStepper ts(system, integ);
  ts.initialize(state);
  ts.stepTo(50.0);
  */

  return 0;
}
