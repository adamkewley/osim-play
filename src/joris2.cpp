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

using namespace OpenSim;

Model buildWrappingModel(const testCase& tc) {
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create a new OpenSim model
    auto model = Model();
    model.setName("EllipsoidWrapping");
    model.setGravity(Vec3(0));
    Ground& ground = model.updGround();

    // Create bodies for the top and bottom connection point
    double bodyMass = tc.BODY_MASS;
    double bodySideLength = tc.BODY_SIZE;
    auto inertia = bodyMass * Inertia::brick(Vec3(bodySideLength / 2.));

    auto bodyTop = new Body("bodyTop",bodyMass,Vec3(0),inertia);
    auto bodyBottom = new Body("bodyBottom",bodyMass,Vec3(0),inertia);
    auto bodyWrapping = new Body("bodyWrapping",bodyMass,Vec3(0),inertia);

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
    testCase tc;
    auto m = buildWrappingModel(tc);
    m.finalizeConnections();

    bool visualize = true;

    if (visualize) {
        m.setUseVisualizer(true);
    }

    SimTK::State s = m.initSystem();
    m.equilibrateMuscles(s);

    if (visualize) {
        m.updMatterSubsystem().setShowDefaultGeometry(true);
        SimTK::Visualizer& viz = m.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(SimTK::White);
    }

    simulate(m, s, 10.0);

    return 0;
}
