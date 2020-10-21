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
static const char appname[] = "opensim_wrapping";
static const char usage[] =
R"(usage: opensim_wrapping [--lhs=[<x>,<y>,<z>]] [--rhs=[<x>,<y>,<z>]]
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
        Set the wrapping quadrant for the wrapping geometry (default: +y)

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
    using SimTK::Vec3;
    using SimTK::Inertia;

    auto lhs_offset = Vec3{-0.4, 0.0, 0.0};
    auto rhs_offset = Vec3{+0.4, 0.0, 0.0};
    std::string record_to = "";
    bool visualize = true;
    std::unique_ptr<WrapObject> wrapSurface = nullptr;
    std::string wrapping_quadrant = "+y";
    double final_time = 10;

    // default wrap surf
    {
        auto c = new WrapCylinder();
        c->setAllPropertiesUseDefault(true);
        c->set_radius(0.08);
        c->set_length(1);
        c->set_xyz_body_rotation(Vec3(0.0, 0.0, 0.0));
        c->setName("wrapSurface");

        wrapSurface.reset(c);
    }

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

                    auto c = new WrapCylinder{};
                    c->setAllPropertiesUseDefault(true);
                    c->set_radius(r);
                    c->set_length(1);
                    c->set_xyz_body_rotation(Vec3(0.0, 0.0, 0.0));
                    c->setName("wrapSurface");

                    wrapSurface.reset(c);
                } else if (std::regex_match(val, m, ellipsoid_patt)) {
                    double x = safe_parse_double(m[1].str().c_str());
                    double y = safe_parse_double(m[2].str().c_str());
                    double z = safe_parse_double(m[3].str().c_str());

                    auto e = new WrapEllipsoid{};
                    e->setAllPropertiesUseDefault(true);
                    e->set_dimensions(Vec3{x, y, z});
                    e->set_xyz_body_rotation(Vec3(0.0, 0.0, 0.0));
                    e->setName("wrapSurface");

                    wrapSurface.reset(e);
                } else {
                    std::cerr << appname << ": unrecognized geometry argument for '" << arg << "'" << std::endl;
                    return -1;
                }
            } else if (strncmp(arg, wrapping_quadrant_arg, sizeof(wrapping_quadrant_arg)-1) == 0) {
                wrapping_quadrant = std::string{arg + (sizeof(wrapping_quadrant_arg)-1)};
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

    wrapSurface->set_quadrant(wrapping_quadrant);


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
    auto sliderLeft = new SliderJoint("sliderLeft", model.getGround(), lhs_offset,
                                      sliderOrientation, *bodyLeft, Vec3(0), sliderOrientation);
    auto sliderRight = new SliderJoint("sliderRight", model.getGround(), rhs_offset,
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
    auto wrap_ptr = wrapSurface.release();
    wrappingFrame->addWrapObject(wrap_ptr);
    bodyGround->addComponent(wrappingFrame);

    // Configure the vastus muscle to wrap over the patella.
    spring->updGeometryPath().addPathWrap(*wrap_ptr);

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

    if (visualize) {
        model.setUseVisualizer(true);
    }

    // Configure the model.
    SimTK::State& state = model.initSystem();

    // Fix the shoulder at its default angle and begin with the elbow flexed.
    //shoulder->getCoordinate().setLocked(state, true);
    model.equilibrateMuscles(state);

    if (not record_to.empty()) {
        auto* analysis = new Position_analysis{
            record_to,
            *sliderLeft,
            *sliderRight,
        };
        model.addAnalysis(analysis);
    }

    if (visualize) {
        model.updMatterSubsystem().setShowDefaultGeometry(true);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(SimTK::White);
        viz.setMode(SimTK::Visualizer::RealTime);
    }

    simulate(model, state, final_time);

    return 0;
}
