#include <iostream>
#include <cstring>
#include <cmath>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

static const char HELP[] =
R"(usage: fd [--help][--no-visualizer][--no-wrapping][--format=<format>]
          [--assembly-accuracy=<accuracy>] model final_time
)";

static bool skip_prefix(char const* s, char const* prefix, char const** out) {
    do {
        if (*prefix == '\0' and (*s == '\0' or *s == '=')) {
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

struct Log_emitter final : public OpenSim::Analysis {
    OpenSim::Model const& model;
    char const* format;

    Log_emitter(OpenSim::Model const& _model, char const* _format) :
        model{_model},
        format{_format} {

        if (!_format) {
            throw std::runtime_error{"nullptr format passed to Log_emitter"};
        }
        std::cerr << "AKINFO: time,prescribeQcalls" << std::endl;
    }

    int step(const SimTK::State& s, int) override {
        std::cerr << "AKINFO: "
                  << s.getTime() << ','
                  << model.getMultibodySystem().getNumPrescribeQCalls() << std::endl;
        return 0;
    }

    Log_emitter* clone() const override {
        return new Log_emitter{model, format};
    }

    const std::string& getConcreteClassName() const override {
        static std::string name = "Log_emitter";
        return name;
    }
};

int main(int argc, char** argv) {
    // skip app name
    --argc;
    ++argv;

    bool visualize = true;
    bool disable_wrapping = false;
    char const* format = nullptr;
    static constexpr double assembly_accuracy_senteniel = -1337.0;
    double assembly_accuracy = assembly_accuracy_senteniel;

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
        } else if (!strcmp(arg, "--no-wrapping")) {
            disable_wrapping = true;
        } else if (skip_prefix(arg, "--format", &arg)) {
            if (*arg == '=') {
                ++arg;
                format = arg;
            } else if (*arg == '\0') {
                if (argc < 2) {
                    std::cerr << "fd: no format string given for --format" << std::endl;
                    std::cerr << HELP;
                    return -1;
                }
                format = argv[1];
                --argc;
                ++argv;
            }
        } else if (skip_prefix(arg, "--assembly-accuracy", &arg)) {
            char const* val = nullptr;

            if (*arg == '=') {
                val = ++arg;
            } else if (*arg == '\0') {
                if (argc < 2) {
                    std::cerr << "fd: no assembly accuracy value given for --assembly-accuract" << std::endl;
                    return -1;
                }
                val = argv[1];
                --argc;
                ++argv;
            }

            if (not safe_parse_double(val, &assembly_accuracy)) {
                std::cerr << "fd: invalid assembly accuracy given for --assembly-accuracy" << std::endl;
                return -1;
            }
        } else {
            std::cerr << "fd: unknown option: " << arg << std::endl;
            std::cerr << HELP;
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

    if (assembly_accuracy != assembly_accuracy_senteniel) {
        model.set_assembly_accuracy(assembly_accuracy);
    }

    if (visualize) {
        model.setUseVisualizer(true);
    }

    SimTK::State& state = model.initSystem();
    model.equilibrateMuscles(state);

    if (format) {
        model.addAnalysis(new Log_emitter{model, format});
    }

    if (visualize) {
        model.updMatterSubsystem().setShowDefaultGeometry(true);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(SimTK::White);
    }

    OpenSim::simulate(model, state, final_time);

    return 0;
}
