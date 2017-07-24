#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChParserOpenSim.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"

#include <functional>
#include <cassert>
#include <cmath>

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::irrlicht;

using namespace irr;
using namespace rapidxml;

int main(int argc, char* argv[]) {
    // Make a system
    ChSystemSMC my_system;

    ChParserOpenSim parser;

    // relative path, needs to change
    parser.parse(my_system, "../../data/biomech/spherical_revolute.osim");
    // parser.parse(my_system, "../../data/biomech/test.osim");

    // Setup Irrlicht
    ChIrrApp application(&my_system, L"ChBodyAuxRef demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, 6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.01);
    // auto bodies = my_system.Get_bodylist();
    // auto links = my_system.Get_linklist();

    while (application.GetDevice()->run()) {
        application.BeginScene();
        // for (int i = 0; i < bodies->size(); ++i) {
        //     auto b = bodies->at(i);
        //     std::cout << b->GetName() << " is at " << b->GetPos().x() << "," << b->GetPos().y() << ","
        //               << b->GetPos().z() << " mass is " << b->GetMass() << std::endl;
        //     std::cout << b->GetRot().e0() << "," << b->GetRot().e1() << "," << b->GetRot().e2() << ","
        //               << b->GetRot().e3() << std::endl;
        // }
        // for (int i = 0; i < links->size(); ++i) {
        //     auto b = links->at(i);
        //     std::cout << b->GetName() << std::endl;
        // }

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }
    return 0;
}
