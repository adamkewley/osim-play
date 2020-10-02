#include <OpenSim/OpenSim.h>

#include <iostream>

#define PRINT_CLASS( o ) std::cout << "sizeof(" #o ") = " << sizeof(o) << std::endl;

int main(int argc, char** argv) {
    PRINT_CLASS(std::vector<char>);

    std::cout << std::endl;

    PRINT_CLASS(OpenSim::Object);
    PRINT_CLASS(OpenSim::Component);
    PRINT_CLASS(OpenSim::ModelComponent);
    PRINT_CLASS(OpenSim::Model);
    PRINT_CLASS(OpenSim::Point);
    PRINT_CLASS(OpenSim::Muscle);
    PRINT_CLASS(OpenSim::GeometryPath);

    return 0;
}
