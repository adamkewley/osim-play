#include <Simbody.h>
#include <OpenSim/OpenSim.h>
#include <string>

struct testCase {
    // General parameters
    bool   SHOW_VISUALIZER     { false };
    double REPORTING_INTERVAL  { 0.01 };
    double FINAL_TIME          { 5.0 };
    int    DISCRETIZATION      { 6 };
    bool   PATH_POINTS         { true };
    // Spring parameters
    double REST_LENGTH         { 1.0 };
    double STIFFNESS           { 500.0 };
    double DISSIPATION         { 0.1 };
    // Muscle parameters
    double OPT_FIBER_LENGTH    { 1.5 };         // was 0.55
    double TENDON_SLACK_LENGTH { 1.0 };         // was 0.50
    double MUSCLE_MAX_FORCE    { 6000.0 };      // was 4000.0
    // Sliding body parameters
    double BODY_MASS           { 10.0 };
    double BODY_MASS_FACTOR    { 1.0 };
    double BODY_SIZE           { 0.1 };
    double BODY_HEIGHT         { 1.08-0.05 };
    double BODY_OFFSET         { 1.0 };
    // Wrapping body parameters
    double CYLINDER_RADIUS     { 0.16 };
    double CYLINDER_HEIGHT     { -CYLINDER_RADIUS };
    int    CYLINDER_COUNT      { 5 };
//    double CYLINDER_HEIGHT     { 0.40 };
    SimTK::Vec3 CYLINDER_ROT   {0.0, 0.0, 0.0};
    std::string WRAP_BODY_TYPE { "ellipsoid" };

    // sine
    double S_AMPLITUDE         { 0.10 };
    double S_OMEGA             { 3.14 };
    double S_PHASE             { 0.00 };
};

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
using SimTK::Vec3;
using SimTK::Inertia;

using OpenSim::Model;
using OpenSim::Ground;
using OpenSim::PinJoint;
using OpenSim::PathSpring;
using OpenSim::SliderJoint;
using OpenSim::Sine;
using OpenSim::WrapCylinder;
using OpenSim::WrapEllipsoid;
using OpenSim::WrapSphere;
using OpenSim::Sphere;

class Simbody_model final {
    MultibodySystem system_;
    SimbodyMatterSubsystem matter_;
    GeneralForceSubsystem forces_;
    GeneralContactSubsystem contact_;
    CableTrackerSubsystem cables_;

    Force::UniformGravity gravity_;
    Inertia inertia_;

    Body::Rigid body_top_;
    MobilizedBody::Pin pin_top_;

    Body::Rigid body_bottom_;
    MobilizedBody::Pin pin_bottom_;

    Body::Rigid body_wrapping_;

public:
    Simbody_model(const testCase& tc) :
        system_{},
        matter_{system_},
        forces_{system_},
        contact_{system_},
        cables_{system_},

        gravity_{forces_, matter_, Vec3{0.0, -9.80665, 0.0}},
        inertia_{tc.BODY_MASS * Inertia::brick(Vec3(tc.BODY_SIZE / 2.))},

        body_top_{MassProperties{tc.BODY_MASS, Vec3{0.0, 0.0, 0.0}, inertia_}},
        pin_top_{
            matter_.Ground(),
            Transform{  // xform from parent
                Rotation{},
                Vec3{0.0, tc.BODY_OFFSET, 0.0},
            },
            body_top_,
            Transform{
                Rotation{},
                Vec3{0.0, 0.0, 0.0}
            }
        },

        body_bottom_{MassProperties{tc.BODY_MASS, Vec3{0.0, 0.0, 0.0}, inertia_}},
        pin_bottom_{
            matter_.Ground(),
            Transform{  // xform from parent
                Rotation{},
                Vec3{0.0, -tc.BODY_OFFSET, 0.0},
            },
            body_top_,
            Transform{
                Rotation{},
                Vec3{0.0, 0.0, 0.0}
            }
        },

        body_wrapping_{MassProperties{tc.BODY_MASS, Vec3{0.0, 0.0, 0.0}, inertia_}}
    {
        // decorate each body as a brick
        for (Body::Rigid* const body : {&body_top_, &body_bottom_, &body_wrapping_}) {
            body->addDecoration(
                SimTK::Transform{},
                SimTK::DecorativeBrick{Vec3{0.1 / 2.0}}.setColor({0.8, 0.1, 0.1}));
        }
    }

    MultibodySystem const& system() const noexcept {
        return system_;
    }
    MultibodySystem& system() noexcept {
        return system_;
    }
};

Model buildWrappingModel(const testCase& tc) {


    // Create a new OpenSim model
    auto model = Model();
    model.setName("EllipsoidWrapping");
    model.setGravity(Vec3(0));
    Ground& ground = model.updGround();

    // Create bodies for the top and bottom connection point
    double bodyMass = tc.BODY_MASS;
    double bodySideLength = tc.BODY_SIZE;
    auto inertia = bodyMass * Inertia::brick(Vec3(bodySideLength / 2.));

    auto bodyTop = new OpenSim::Body("bodyTop",bodyMass,Vec3(0),inertia);
    auto bodyBottom = new OpenSim::Body("bodyBottom",bodyMass,Vec3(0),inertia);
    auto bodyWrapping = new OpenSim::Body("bodyWrapping",bodyMass,Vec3(0),inertia);

    model.addBody(bodyTop);
    model.addBody(bodyBottom);
    model.addBody(bodyWrapping);

    // I should be able to use pin joint as hinges. Now the model works for a
    // cylinder or ellipsoid where the slider direction is perpendicular to the
    // spring spanning direction but this is kinda hacky. Seems like there is something
    // wrong with the PinJoint. Ask Ajay about this!
    auto pinJointOffset = Vec3(0,tc.BODY_OFFSET,0);
    auto pinTop = new PinJoint("pinTop",ground,pinJointOffset,Vec3(0),
                                        *bodyTop,Vec3(0),Vec3(0));
    auto pinBottom = new PinJoint("pinBottom",ground,-pinJointOffset,Vec3(0),
                                              *bodyBottom,Vec3(0),Vec3(0));

    // Add the joints to the model.
    model.addJoint(pinTop);
    model.addJoint(pinBottom);

    // Add the spring or muscle
    auto spring = new PathSpring("spring", tc.REST_LENGTH, tc.STIFFNESS, tc.DISSIPATION);
    spring->updGeometryPath().appendNewPathPoint("origin", *bodyTop, Vec3(0));
    spring->updGeometryPath().appendNewPathPoint("insertion", *bodyBottom, Vec3(0));

    // Add the moving frame on which the wrapping surface will be connected
    auto sliderGround = new SliderJoint("sliderGround",ground,Vec3(0),Vec3(0),
                                        *bodyWrapping,Vec3(0),Vec3(0));

    auto sine = new Sine(tc.S_AMPLITUDE,tc.S_OMEGA,tc.S_PHASE);
    auto& sliderCoord = sliderGround->updCoordinate();
    sliderCoord.setName("sliderX");
    sliderCoord.setPrescribedFunction(*sine);
    sliderCoord.setDefaultIsPrescribed(true);

    // Add the wrapping surfaces 'ellipsoid', 'cylinder', and 'sphere
    auto wrapSurface = new WrapCylinder();
    if (tc.WRAP_BODY_TYPE == "ellipsoid" || tc.WRAP_BODY_TYPE == "ellipse"){
        delete wrapSurface;
        auto wrapSurface = new WrapEllipsoid();
        wrapSurface->set_dimensions(Vec3(tc.CYLINDER_RADIUS,tc.CYLINDER_RADIUS,2));
    } else if (tc.WRAP_BODY_TYPE == "cylinder") {
        delete wrapSurface;
        auto wrapSurface = new WrapCylinder();
        wrapSurface->set_radius(tc.CYLINDER_RADIUS);
        wrapSurface->set_length(2);
    } else if (tc.WRAP_BODY_TYPE == "sphere") {
        delete wrapSurface;
        auto wrapSurface = new WrapSphere();
        wrapSurface->set_radius(tc.CYLINDER_RADIUS);
    } else {
        wrapSurface->set_radius(tc.CYLINDER_RADIUS);
        wrapSurface->set_length(2);
    }

    wrapSurface->set_xyz_body_rotation(tc.CYLINDER_ROT);
    wrapSurface->set_quadrant("+x");
    wrapSurface->setName("wrapSurface");

    bodyWrapping->addWrapObject(wrapSurface);

    spring->updGeometryPath().addPathWrap(*wrapSurface);

    model.addJoint(sliderGround);
    model.addForce(spring);

    // Visualization
    double sphereRadius = 0.04;
    auto bodyTopGeometry = new Sphere(sphereRadius);
    bodyTopGeometry->setColor(Vec3(0.8,0.1,0.1));
    bodyTop->attachGeometry(bodyTopGeometry);

    auto bodyBottomGeometry = new Sphere(sphereRadius);
    bodyBottomGeometry->setColor(Vec3(0.8,0.1,0.1));
    bodyBottom->attachGeometry(bodyBottomGeometry);

    auto bodyWrappingGeometry = new Sphere(sphereRadius);
    bodyWrappingGeometry->setColor(Vec3(0.1,0.8,0.1));
    bodyWrapping->attachGeometry(bodyWrappingGeometry);

    if (tc.SHOW_VISUALIZER) {
        model.setUseVisualizer(true);
    }
    return model;
}

int main(int, char**) {
    double final_time = 10.0;

    testCase tc;
    auto model = Simbody_model{tc};
    auto& system = model.system();
    system.setUseUniformBackground(true);

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
        time_stepper.stepTo(final_time);
    }

    return 0;
}
