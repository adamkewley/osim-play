#include <OpenSim/OpenSim.h>

using namespace SimTK;
using namespace OpenSim;

int main(int argc, char** argv) {
    Model model;
    model.setName("bicep_curl");
    model.setUseVisualizer(true);

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* humerus = new OpenSim::Body(
        "humerus",
        1,
        Vec3(0),
        Inertia(0));

    auto* radius  = new OpenSim::Body(
        "radius",
        1,
        Vec3(0),
        Inertia(0));

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    auto* shoulder = new PinJoint(
        "shoulder",

        model.getGround(),
        Vec3(0),
        Vec3(0),

        *humerus,
        Vec3(0, 1, 0),
        Vec3(0));

    auto* elbow = new PinJoint(
        "elbow",

        *humerus,
        Vec3(0),
        Vec3(0),

        *radius,
        Vec3(0, 1 , 0),
        Vec3(0));

    // Add a muscle that flexes the elbow.

    auto* biceps = new Millard2012EquilibriumMuscle(
        "biceps",
        200,
        0.6,
        0.55,
        0);

    auto* wc = new WrapCylinder{};
    wc->setName("pulley1");
    wc->set_radius(0.4);
    wc->set_length(0.1);
    wc->set_translation(Vec3{-0.1, -0.9, 0});

    model.updGround().addWrapObject(wc);

    GeometryPath& p = biceps->updGeometryPath();
    //p.addPathWrap(*wc);
    p.appendNewPathPoint("origin",    *humerus, Vec3(0, 0.8, 0));
    auto* tmp = p.appendNewPathPoint("tmp",    *humerus, Vec3(0, 0.8, 0));
    // this isn't done programatically in OpenSim anywhere. All of it is done
    // via the osim files.
    /*ConditionalPathPoint cpp{};
    cpp.setRangeMin(1);
    cpp.setRangeMax(2);
    cpp.setLocation(Vec3(0, 0, 0));
    cpp.setParentFrame(*humerus);
    cpp.setName("this_is_a_terrible_api");
    p.replacePathPoint(tmp, &cpp);
    */
    p.appendNewPathPoint("insertion", *radius,  Vec3(0, 0.7, 0));

    // Add a controller that specifies the excitation of the muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*biceps);
    // Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
    brain->prescribeControlForActuator("biceps",
            new StepFunction(1, 3, 0.1, 1));

    // Add components to the model.
    model.addBody(humerus);
    model.addBody(radius);
    model.addJoint(shoulder);
    model.addJoint(elbow);
    model.addForce(biceps);
    model.addController(brain);

    // Add a console reporter to print the muscle fiber force and elbow angle.
    /*
    ConsoleReporter* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(1.0);
    reporter->addToReport(biceps->getOutput("fiber_force"));
    reporter->addToReport(
        elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
        "elbow_angle");
    model.addComponent(reporter);
    */

    // Add display geometry.
    Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    bodyGeometry.setColor(Gray);
    // Attach an ellipsoid to a frame located at the center of each body.
    PhysicalOffsetFrame* humerusCenter = new PhysicalOffsetFrame(
        "humerusCenter", *humerus, Transform(Vec3(0, 0.5, 0)));
    humerus->addComponent(humerusCenter);
    humerusCenter->attachGeometry(bodyGeometry.clone());
    PhysicalOffsetFrame* radiusCenter = new PhysicalOffsetFrame(
        "radiusCenter", *radius, Transform(Vec3(0, 0.5, 0)));
    radius->addComponent(radiusCenter);
    radiusCenter->attachGeometry(bodyGeometry.clone());


    // Configure the model.
    State& state = model.initSystem();
    // Fix the shoulder at its default angle and begin with the elbow flexed.
    shoulder->getCoordinate().setLocked(state, true);
    elbow->getCoordinate().setValue(state, 0.5 * Pi);
    model.equilibrateMuscles(state);

    // Configure the visualizer.
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(White);

    // Simulate.
    simulate(model, state, 10.0);

    return 0;
};
