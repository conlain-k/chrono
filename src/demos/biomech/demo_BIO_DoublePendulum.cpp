#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_thirdparty/rapidxml/rapidxml.hpp"
#include "chrono_thirdparty/rapidxml/rapidxml_print.hpp"
#include "chrono_thirdparty/rapidxml/rapidxml_utils.hpp"

#include <functional>
#include <cassert>

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;
using namespace rapidxml;

double color = .2;

std::map<std::string, std::function<void(xml_node<>*, ChSystem&, std::shared_ptr<ChBody>)>> function_table;

bool parseBody(xml_node<>* bodyNode, ChSystem& my_system) {
    // Node containing all of body information
    xml_node<>* fieldNode = bodyNode->first_node();
    std::cout << "New body " << bodyNode->first_attribute("name")->value() << std::endl;
    auto newBody = std::make_shared<ChBody>();
    newBody->SetName(bodyNode->first_attribute("name")->value());

    // Give it a cylinder for now
    auto body_cyl = std::make_shared<ChCylinderShape>();
    body_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 1.2);
    body_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0.8);
    body_cyl->GetCylinderGeometry().rad = 0.2;
    newBody->AddAsset(body_cyl);
    my_system.Add(newBody);

    // newBody->SetBodyFixed(true);

    // Parse the body, field-by-field
    while (fieldNode != NULL) {
        std::cout << "Field is " << *fieldNode << " name is " << fieldNode->name() << std::endl;

        // Parse in body information
        function_table[fieldNode->name()](fieldNode, my_system, newBody);

        fieldNode = fieldNode->next_sibling();
    }
}

std::vector<std::string> strToVect(const char* string) {
    std::istringstream buf(string);
    std::istream_iterator<std::string> beg(buf), end;

    return std::vector<std::string>(beg, end);
}
void initFunctionTable() {  // Setup lambda table for body parsing
    function_table["mass"] = [](xml_node<>* fieldNode, ChSystem& my_system, std::shared_ptr<ChBody> newBody) {
        if (std::stod(fieldNode->value()) == 0) {
            // Ground-like body, massless => fixed
            std::cout << "Body is fixed" << std::endl;
            newBody->SetBodyFixed(true);
            newBody->SetCollide(false);
            auto body_col = std::make_shared<ChColorAsset>();
            body_col->SetColor(ChColor(0, 0, 0));
            newBody->AddAsset(body_col);

        } else {
            newBody->SetMass(std::stod(fieldNode->value()));

            auto body_col = std::make_shared<ChColorAsset>();
            body_col->SetColor(ChColor(color, 2 * color, 3 * color));
            color += .2;
            newBody->AddAsset(body_col);
        }
    };
    function_table["mass_center"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                       std::shared_ptr<ChBody> newBody) {
        // Set COM in reference frame ?
    };
    function_table["inertia_xx"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBody> newBody) {
        // Set xx inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.x() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_yy"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBody> newBody) {
        // Set yy inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.y() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_zz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBody> newBody) {
        // Set zz inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.z() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_xy"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBody> newBody) {
        // Set xy inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.x() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["inertia_xz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBody> newBody) {
        // Set xz inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.y() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["inertia_yz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBody> newBody) {
        // Set yz inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.z() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["Joint"] = [](xml_node<>* fieldNode, ChSystem& my_system, std::shared_ptr<ChBody> newBody) {
        // Deduce position of child body, make a joint between them
        if (fieldNode->first_node() == NULL) {
            std::cout << "No joints for this body " << std::endl;
            return;
        }

        xml_node<>* jointNode = fieldNode->first_node();
        // Make a joint here
        std::cout << "Making a " << jointNode->name() << " with " << jointNode->first_node("parent_body")->value()
                  << std::endl;

        // function_table[fieldNode->first_node()->name()](fieldNode, my_system, newBody);

        // Get other body for joint
        auto parent = my_system.SearchBody(jointNode->first_node("parent_body")->value());

        if (parent != nullptr) {
            std::cout << "other body found!" << std::endl;
        } else {
            std::cout << "Body not found " << std::endl;
        }

        auto parentPos = parent->GetPos();
        auto parentOrientation = parent->GetRot();

        // Get vectors from file
        std::vector<std::string> elems = strToVect(jointNode->first_node("location_in_parent")->value());
        ChVector<> jointPosInParent = ChVector<>(std::stod(elems[0]), std::stod(elems[1]), std::stod(elems[2]));

        elems = strToVect(jointNode->first_node("location")->value());
        ChVector<> jointPosInChild = ChVector<>(std::stod(elems[0]), std::stod(elems[1]), std::stod(elems[2]));
        std::cout << jointPosInChild.x() << "," << jointPosInChild.y() << "," << jointPosInChild.z() << std::endl;

        // Get quaternions from file
        elems = strToVect(jointNode->first_node("orientation_in_parent")->value());
        ChQuaternion<> jointOrientationInParent;
        jointOrientationInParent.Q_from_NasaAngles(
            ChVector<>(std::stod(elems[2]), std::stod(elems[0]), std::stod(elems[1])));

        elems = strToVect(jointNode->first_node("orientation")->value());
        ChQuaternion<> jointOrientationInChild;
        jointOrientationInChild.Q_from_NasaAngles(
            ChVector<>(std::stod(elems[2]), std::stod(elems[0]), std::stod(elems[1])));

        ChVector<> jointPosGlobal = parentPos + parentOrientation.GetInverse().Rotate(jointPosInParent);

        // Get orientation for child body
        newBody->SetRot(parentOrientation * jointOrientationInParent * jointOrientationInChild.GetInverse());
        // Use orientation to get position for child body
        newBody->SetPos(jointPosGlobal - newBody->TransformPointLocalToParent(jointPosInChild));
        assert(newBody.GetRot.Length() == 1);

        std::cout << "Parent is at global " << parentPos.x() << "," << parentPos.y() << "," << parentPos.z()
                  << std::endl;
        std::cout << "Joint is at parent " << jointPosInParent.x() << "," << jointPosInParent.y() << ","
                  << jointPosInParent.z() << std::endl;
        std::cout << "Joint is at global " << jointPosGlobal.x() << "," << jointPosGlobal.y() << ","
                  << jointPosGlobal.z() << std::endl;
        std::cout << "Joint is at child " << jointPosInChild.x() << "," << jointPosInChild.y() << ","
                  << jointPosInChild.z() << std::endl;
        std::cout << "Putting body " << newBody->GetName() << " at " << newBody->GetPos().x() << ","
                  << newBody->GetPos().y() << "," << newBody->GetPos().z() << std::endl;

        auto joint = std::make_shared<ChLinkLockRevolute>();
        if (newBody->GetNameString() == std::string("rod1")) {
            joint->Initialize(parent, newBody, ChCoordsys<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
            std::cout << "rod1" << std::endl;
        } else {
            joint->Initialize(parent, newBody, ChCoordsys<>(ChVector<>(2, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
            std::cout << "rod2" << std::endl;
        }
        // joint->Initialize(parent, newBody, true, ChCoordsys<>(jointPosInParent, jointOrientationInParent),
        //                   ChCoordsys<>(jointPosInChild, jointOrientationInChild));
        joint->SetNameString(std::string(parent->GetName()) + "_" + std::string(newBody->GetName()));
        // my_system.AddLink(joint);

        // Z X Y
        // auto jointOrientationInParent = std::stod(jointNode->first_node("orientation_in_parent")->value());
        // std::cout << jointOrientationInParent << std::endl;
    };
    // function_table["PinJoint"] = [](xml_node<>* fieldNode, ChSystem& my_system, std::shared_ptr<ChBody>
    // newBody) {
    //
    //     auto joint = std::make_shared<ChLinkLockRevolute>();
    //     joint->Initialize(otherBody, newBody, ChCoordsys<>(ChVector<>(0, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
    //
    // };

    function_table["VisibleObject"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                         std::shared_ptr<ChBody> newBody) {
        // Not needed maybe?
    };
    function_table["WrapObjectSet"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                         std::shared_ptr<ChBody> newBody) {
        // I don't think we need this either
    };
}

int main(int argc, char* argv[]) {
    // Make a system
    ChSystemSMC my_system;

    initFunctionTable();

    // relative path, needs to change
    // rapidxml::file<> file("../../data/biomech/ScapulothorachicJoint_Shoulder.osim");
    rapidxml::file<> file("../../data/biomech/double_pendulum.osim");

    // read in and parse data
    xml_document<> doc;         // character type defaults to char
    doc.parse<0>(file.data());  // 0 means default parse flags

    // Hold list of bodies
    xml_node<>* bodySet = doc.first_node()->first_node("Model")->first_node("BodySet")->first_node("objects");

    // Get gravity from model and set it in system
    {
        std::istringstream gravBuf(doc.first_node()->first_node("Model")->first_node("gravity")->value());
        std::istream_iterator<std::string> beg(gravBuf), end;
        std::vector<std::string> elems(beg, end);
        // my_system.Set_G_acc(ChVector<>(std::stod(elems[0]), std::stod(elems[1]), std::stod(elems[2])));
        my_system.Set_G_acc(ChVector<>(0, 0, 0));
    }

    // Holds list of fields for body
    xml_node<>* bodyNode = bodySet->first_node();

    // Loop through each body
    while (bodyNode != NULL) {
        parseBody(bodyNode, my_system);
        bodyNode = bodyNode->next_sibling();
    }

    ChIrrApp application(&my_system, L"ChBodyAuxRef demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, 6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.001);
    while (application.GetDevice()->run()) {
        application.BeginScene();
        auto bodies = my_system.Get_bodylist();
        auto links = my_system.Get_linklist();
        for (int i = 0; i < bodies->size(); ++i) {
            auto b = bodies->at(i);
            std::cout << b->GetName() << " is at " << b->GetPos().x() << "," << b->GetPos().y() << ","
                      << b->GetPos().z() << " mass is " << b->GetMass() << std::endl;
        }
        for (int i = 0; i < links->size(); ++i) {
            auto b = links->at(i);
            std::cout << b->GetName() << std::endl;
        }

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }
    return 0;
}
