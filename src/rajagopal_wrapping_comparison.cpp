#include <iostream>
#include <cstring>
#include <cmath>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

static const char HELP[] =
R"(usage: fd [--help][--no-visualizer][--disable-wrapping] model final_time
)";

static bool skip_prefix(char const* s, char const* prefix, char const** out) {
    do {
        if (*prefix == '\0') {
            *out = s;
            return true;
        }
    } while (*s++ == *prefix++);
    return false;
}

static bool safe_parse_double(char const* s, double* out) {
    char* end;
    double v = strtod(s, &end);

    if (*end != '\0' or v == HUGE_VAL or v == -HUGE_VAL) {
        return false;  // not a number
    }

    *out = v;
    return true;
}

int main(int argc, char** argv) {
    // skip app name
    --argc;
    ++argv;

    bool visualize = true;
    bool disable_wrapping = false;

    while (argc > 0) {
        char const* arg = argv[0];

        if (arg[0] != '-') {
            break;
        }

        if (!strcmp(arg, "--help")) {
            std::cout << HELP;
            return 0;
        } else if (!strcmp(arg, "--no-visualizer")) {
            visualize = false;
        } else if (!strcmp(arg, "--disable-wrapping")) {
            disable_wrapping = true;
        } else {
            std::cerr << "fd: unknown option: " << arg << std::endl;
            return -1;
        }

        --argc;
        ++argv;
    }

    switch (argc) {
    case 0:
        std::cerr << "fd: missing arguments: model final_time" << std::endl;
        std::cerr << HELP;
        return -1;
    case 1:
        std::cerr << "fd: missing final_time argument" << std::endl;
        std::cerr << HELP;
        return -1;
    case 2:
        break;
    default:
        std::cerr << "fd: invalid number of arguments" << std::endl;
        std::cerr << HELP;
        return -1;
    }

    double final_time;
    if (not safe_parse_double(argv[1], &final_time)) {
        std::cerr << "fd: " << argv[1] << ": invalid final time (not a number)" << std::endl;
        return -1;
    }

    OpenSim::Model model{argv[0]};

    if (disable_wrapping) {
        OpenSim::ComponentList<OpenSim::WrapObjectSet> l =
                model.updComponentList<OpenSim::WrapObjectSet>();
        for (OpenSim::WrapObjectSet& wos : l) {
            for (int i = 0; i < wos.getSize(); ++i) {
                OpenSim::WrapObject& wo = wos[i];
                wo.set_active(false);
            }
        }
    }

    if (visualize) {
        model.setUseVisualizer(true);
    }

    SimTK::State& state = model.initSystem();
    model.equilibrateMuscles(state);

    if (visualize) {
        model.updMatterSubsystem().setShowDefaultGeometry(true);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(SimTK::White);
        //viz.setMode(SimTK::Visualizer::RealTime);
    }

    OpenSim::simulate(model, state, final_time);
    return 0;
}
