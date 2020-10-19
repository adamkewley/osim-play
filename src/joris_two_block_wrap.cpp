/* -------------------------------------------------------------------------- *
 *                     test_suite.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Test_suite that checks analytical solutions of muscle lengths over         *
 * wrapping surfaces with the numerical solutions of OpenSim. Especially      *
 * important for unconventional wrapping (over a rotated cylinder for example *
 *                                                                            *
 * Author(s): Joris Verhagen                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>

#include <fstream>

using namespace OpenSim;
using SimTK::PeriodicEventReporter;
using SimTK::MultibodySystem;
using SimTK::MobilizedBody;
using SimTK::Real;
using SimTK::State;

struct Position_analysis final : public Analysis {
    SliderJoint const& lhs;
    SliderJoint const& rhs;
    std::string path;
    std::ofstream ofile;

    Position_analysis(std::string _path,
                      SliderJoint const& _lhs,
                      SliderJoint const& _rhs) :
        lhs{_lhs},
        rhs{_rhs},
        path{std::move(_path)},
        ofile{[&]() {
            auto f = std::ofstream{};
            f.exceptions(std::ofstream::failbit);
            f.open(path);
            return f;
        }()} {

        ofile << "time,lhs,rhs" << std::endl;
    }

    int step(const SimTK::State& s, int stepNumber) override {
        ofile << s.getTime() << ","
              << lhs.getCoordinate().getValue(s) << ","
              << rhs.getCoordinate().getValue(s) << std::endl;
        return 0;
    }

    Position_analysis* clone() const {
        return new Position_analysis{path, lhs, rhs};
    }

    const std::string& getConcreteClassName() const {
        static std::string name = "Position_analysis";
        return name;
    }
};

int main(int argc, char** arv) {
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create a new OpenSim model on earth.
    auto model = Model();
    model.setName("Normal_Wrapping");
    model.setGravity(Vec3(0, -9.80665, 0));

    // BODIES
    double bodyMass = 30.0;
    double bodySideLength = 0.1;
    auto bodyInertia = bodyMass * Inertia::brick(Vec3(bodySideLength / 2.));
    auto bodyLeft = new Body("bodyLeft", bodyMass, Vec3(0), bodyInertia);
    auto bodyRight = new Body("bodyRight", bodyMass, Vec3(0), bodyInertia);
    // Create static ground body for wrapping surface frame
    auto bodyGround = new Body("bodyGround", 1, Vec3(0), bodyInertia);

    model.addBody(bodyLeft);
    model.addBody(bodyRight);
    model.addBody(bodyGround);

    // Attach the pelvis to ground with a vertical slider joint, and attach the
    // pelvis, thigh, and shank bodies to each other with pin joints.
    Vec3 sliderOrientation(0, 0, SimTK::Pi / 2.);
    Vec3 bodyOffset(0.4, 0, 0);
    auto sliderLeft = new SliderJoint("sliderLeft", model.getGround(), bodyOffset,
                                      sliderOrientation, *bodyLeft, Vec3(0), sliderOrientation);
    auto sliderRight = new SliderJoint("sliderRight", model.getGround(), -bodyOffset,
                                       sliderOrientation, *bodyRight, Vec3(0), sliderOrientation);
    auto weldGround = new WeldJoint("weldGround", model.getGround(), *bodyGround);

    // Add the joints to the model.
    model.addJoint(sliderLeft);
    model.addJoint(sliderRight);
    model.addJoint(weldGround);

    // Set the coordinate names and default values. Note that we need "auto&"
    // here so that we get a reference to the Coordinate rather than a copy.
    auto &sliderCoordLeft =
            sliderLeft->updCoordinate(SliderJoint::Coord::TranslationX);
    sliderCoordLeft.setName("yCoordSliderLeft");
    sliderCoordLeft.setDefaultValue(0.5);
    auto &sliderCoordRight =
            sliderRight->updCoordinate(SliderJoint::Coord::TranslationX);
    sliderCoordRight.setName("yCoordSliderRight");
    sliderCoordRight.setDefaultValue(0.5);



    // MUSCLES AND SPRINGS
    double mclFmax = 4000., mclOptFibLen = 0.55, mclTendonSlackLen = 0.5,
            mclPennAng = 0.;
    /*
    auto muscle = new Thelen2003Muscle("muscle", mclFmax, mclOptFibLen,
                                       mclTendonSlackLen, mclPennAng);
    muscle->addNewPathPoint("origin", *bodyLeft, Vec3(0, bodySideLength / 2, 0));
    muscle->addNewPathPoint("insertion", *bodyRight, Vec3(0, bodySideLength / 2, 0));
    */
    auto spring = new PathSpring("path_spring", 1.0, 50, 0.1);
    spring->updGeometryPath().appendNewPathPoint("origin", *bodyLeft, Vec3(0, bodySideLength / 2, 0));
    spring->updGeometryPath().appendNewPathPoint("insertion", *bodyRight, Vec3(0, bodySideLength / 2, 0));

    auto springToLeft = new PointToPointSpring(model.getGround(), Vec3(0),
                                               *bodyLeft, Vec3(0, -bodySideLength / 2, 0), 100, 0.5);
    springToLeft->setName("springToLeft");
    auto springToRight = new PointToPointSpring(model.getGround(), Vec3(0),
                                                *bodyRight, Vec3(0, -bodySideLength / 2, 0), 100, 0.5);
    springToRight->setName("springToRight");


    model.addForce(springToLeft);
    model.addForce(springToRight);



    // WRAPPING SURFACE
    auto wrappingFrame = new PhysicalOffsetFrame("wrappingFrame", model.getGround(),
                                                 SimTK::Transform(Vec3(0, 1.0, 0)));
    // Add the wrapping surface
    // auto wrapSurface = new ak_WrapCylinder();
    auto wrapSurface = new WrapCylinder();
//    auto wrapSurface = new WrapEllipsoid();
    wrapSurface->setAllPropertiesUseDefault(true);
    wrapSurface->set_radius(0.08);
    wrapSurface->set_length(1);
//    wrapSurface->set_dimensions(Vec3(tc.CYLINDER_RADIUS,tc.CYLINDER_RADIUS,1));
    wrapSurface->set_xyz_body_rotation(Vec3(0.0, 0.0, 0.0));
    wrapSurface->set_quadrant("+y");
    wrapSurface->setName("wrapSurface");
    wrappingFrame->addWrapObject(wrapSurface);
    bodyGround->addComponent(wrappingFrame);

    // Configure the vastus muscle to wrap over the patella.
    spring->updGeometryPath().addPathWrap(*wrapSurface);

    //auto spring = new PathSpring("path_spring", 1.0, 50, 0.1);
    //spring->updGeometryPath() = *muscle->getGeometryPath().clone();
//        spring->finalizeConnections(model);
//    model.addForce(muscle);
    model.addForce(spring);



    // CONTROLLER
    /*
    auto brain = new PrescribedController();
    brain->setActuators(model.updActuators());
    double t[5] = {0.0, 1.0, 2.0, 3.0, 4.0}, x[5] = {0.0, 1.0, 0.0, 0.5, 0.0};
    auto controlFunction = new PiecewiseConstantFunction(5, t, x);
    brain->prescribeControlForActuator("muscle", controlFunction);
    model.addController(brain);*/

    // Attach geometry to the bodies and enable the visualizer.
    auto bodyLeftGeometry = new Brick(Vec3(bodySideLength / 2.));
    bodyLeftGeometry->setColor(Vec3(0.8, 0.1, 0.1));
    bodyLeft->attachGeometry(bodyLeftGeometry);

    auto bodyRightGeometry = new Brick(Vec3(bodySideLength / 2.));
    bodyRightGeometry->setColor(Vec3(0.8, 0.1, 0.1));
    bodyRight->attachGeometry(bodyRightGeometry);

    model.setUseVisualizer(true);

    // Configure the model.
    SimTK::State& state = model.initSystem();

    // Fix the shoulder at its default angle and begin with the elbow flexed.
    //shoulder->getCoordinate().setLocked(state, true);
    model.equilibrateMuscles(state);

    auto* analysis = new Position_analysis{
            "/tmp/opensim_version.csv",
            *sliderLeft,
            *sliderRight,
    };
    model.addAnalysis(analysis);

    // Configure the visualizer.
    {
        model.updMatterSubsystem().setShowDefaultGeometry(true);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(SimTK::White);
        viz.setMode(SimTK::Visualizer::RealTime);
    }

    // Simulate.
    simulate(model, state, 10.0);

    return 0;
}
