#include "Simbody.h"

using namespace SimTK;

int main() {
  // subclass of System, defines functionality for dealing with
  // multi-body systems
  MultibodySystem system;

  // defines all the bodies in the system. A MultibodySystem must
  // always have this.
  SimbodyMatterSubsystem matter(system);

  // to add a variety of forces to a system
  GeneralForceSubsystem forces(system);

  // add gravity to the force subsystem (other forces exist,
  // e.g. springs, dampers, etc.)
  Force::UniformGravity gravity(forces, matter, Vec3(0, -9.8, 0));

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

  return 0;
}
