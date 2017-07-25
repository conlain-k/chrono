// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Conlain Kelly
// =============================================================================
//
// Parser utility class for OpenSim input files.
//
// =============================================================================

#include "chrono/utils/ChParserOpenSim.h"
#include "chrono_thirdparty/rapidxml/rapidxml_print.hpp"
#include "chrono_thirdparty/rapidxml/rapidxml_utils.hpp"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChFrame.h"

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"

namespace chrono {
namespace utils {

using namespace rapidxml;

ChParserOpenSim::ChParserOpenSim() {
    initFunctionTable();
}

void ChParserOpenSim::parse(ChSystem& p_system, const std::string& filename, ChParserOpenSim::VisType vis) {
    m_visType = vis;
    rapidxml::file<char> file(filename.c_str());

    xml_document<> doc;         // character type defaults to char
    doc.parse<0>(file.data());  // 0 means default parse flags

    // Hold list of bodies
    xml_node<>* bodySet = doc.first_node()->first_node("Model")->first_node("BodySet")->first_node("objects");

    // Get gravity from model and set it in system
    auto elems = strToDoubleVector(doc.first_node()->first_node("Model")->first_node("gravity")->value());
    p_system.Set_G_acc(ChVector<>(elems[0], elems[1], elems[2]));

    // Holds list of fields for body
    xml_node<>* bodyNode = bodySet->first_node();

    // Loop through each body
    while (bodyNode != NULL) {
        parseBody(bodyNode, p_system);
        bodyNode = bodyNode->next_sibling();
    }
    initVisualizations(bodyNode, p_system);
}

ChSystem* ChParserOpenSim::parse(const std::string& filename,
                                 ChMaterialSurface::ContactMethod contact_method,
                                 ChParserOpenSim::VisType vis) {
    ChSystem* sys = (contact_method == ChMaterialSurface::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                               : static_cast<ChSystem*>(new ChSystemSMC);

    parse(*sys, filename, vis);

    return sys;
}

// -------------------------------------------------------------------------------------------

// Creates ChBody and parses its various properties from its XML child nodes
bool ChParserOpenSim::parseBody(xml_node<>* bodyNode, ChSystem& my_system) {
    // Make a new body and name it for later
    std::cout << "New body " << bodyNode->first_attribute("name")->value() << std::endl;
    // TODO - This should eventually be a ChBodyAuxRef but polymorphism should make that easy
    // I don't want to debug two things at once
    auto newBody = std::make_shared<ChBodyAuxRef>();
    newBody->SetName(bodyNode->first_attribute("name")->value());

    // Give it a cylinder for now
    // auto body_cyl = std::make_shared<ChCylinderShape>();
    // body_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, -.2);
    // body_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, .2);
    // body_cyl->GetCylinderGeometry().rad = 0.2;
    // newBody->AddAsset(body_cyl);
    my_system.Add(newBody);

    // First node in linked list of fields
    xml_node<>* fieldNode = bodyNode->first_node();

    // Parse the body, field-by-field
    while (fieldNode != NULL) {
        // Parse in body information
        function_table[fieldNode->name()](fieldNode, my_system, newBody);

        fieldNode = fieldNode->next_sibling();
    }
    return true;
}

// Get an STL vector from a string, used to make the xml parsing cleaner
std::vector<double> ChParserOpenSim::strToDoubleVector(const char* string) {
    std::istringstream buf(string);
    std::istream_iterator<double> beg(buf), end;
    return std::vector<double>(beg, end);
}

// This is where the magic happens -- it sets up a map of strings to lambdas
// that do the heavy lifting of osim -> chrono parsing
void ChParserOpenSim::initFunctionTable() {
    function_table["mass"] = [](xml_node<>* fieldNode, ChSystem& my_system, std::shared_ptr<ChBodyAuxRef> newBody) {
        if (std::stod(fieldNode->value()) == 0) {
            // Ground-like body, massless => fixed
            newBody->SetBodyFixed(true);
            newBody->SetCollide(false);
            auto body_col = std::make_shared<ChColorAsset>();
            // Ground has special color to identify it
            body_col->SetColor(ChColor(0, 0, 0));
            newBody->AddAsset(body_col);
        } else {
            // auto cyl_1 = std::make_shared<ChCylinderShape>();
            // cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, .5, 0);
            // cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
            // cyl_1->GetCylinderGeometry().rad = 0.1;
            // newBody->AddAsset(cyl_1);
            // Give body mass and color and rod
            newBody->SetMass(std::stod(fieldNode->value()));
            // auto body_col = std::make_shared<ChColorAsset>();
            // body_col->SetColor(ChColor(0, 0, .5f));
            // newBody->AddAsset(body_col);
        }
    };
    function_table["mass_center"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                       std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set COM in reference frame ?
        auto elems = strToDoubleVector(fieldNode->value());
        // Opensim doesn't really use a rotated frame, so unit quaternion
        newBody->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(elems[0], elems[1], elems[2]), ChQuaternion<>(1, 0, 0, 0)));
        // {
        //     auto sphere = std::make_shared<ChSphereShape>();
        //     sphere->GetSphereGeometry().rad = 0.2;
        //     sphere->Pos = ChVector<>(elems[0], elems[1], elems[2]);
        //     // sphere->Pos = ChVector<>(0, 0, 0);
        //     sphere->SetColor(ChColor(0, 0, .5f));
        //     newBody->AddAsset(sphere);
        // }
        {
            auto sphere = std::make_shared<ChSphereShape>();
            sphere->GetSphereGeometry().rad = 0.15;
            sphere->Pos = ChVector<>(0, 0, 0);
            // sphere->Pos = ChVector<>(0, 0, 0);
            sphere->SetColor(ChColor(0, .5f, 0));
            newBody->AddAsset(sphere);
        }
    };
    function_table["inertia_xx"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xx inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.x() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_yy"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set yy inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.y() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_zz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set zz inertia moment
        ChVector<> inertiaXX = newBody->GetInertiaXX();
        inertiaXX.z() = std::stod(fieldNode->value());
        newBody->SetInertiaXX(inertiaXX);
    };
    function_table["inertia_xy"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xy inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.x() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["inertia_xz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set xz inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.y() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["inertia_yz"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                      std::shared_ptr<ChBodyAuxRef> newBody) {
        // Set yz inertia moment
        ChVector<> inertiaXY = newBody->GetInertiaXY();
        inertiaXY.z() = std::stod(fieldNode->value());
        newBody->SetInertiaXY(inertiaXY);
    };
    function_table["Joint"] = [this](xml_node<>* fieldNode, ChSystem& my_system,
                                     std::shared_ptr<ChBodyAuxRef> newBody) {
        // If there are no joints, this is hopefully the ground (or another global parent??)
        if (fieldNode->first_node() == NULL) {
            std::cout << "No joints for this body " << std::endl;
            return;
        }

        // Deduce child body from joint orientation
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
            std::cout << "Parent not found!!!!" << std::endl;
            exit(1);
        }

        // Global to parent
        auto X_G_P = parent->GetFrame_REF_to_abs();

        // Get joint's position in parent frame
        auto elems = ChParserOpenSim::strToDoubleVector(jointNode->first_node("location_in_parent")->value());
        ChVector<> jointPosInParent = ChVector<>(elems[0], elems[1], elems[2]);

        // Get joint's orientation in parent frame
        elems = ChParserOpenSim::strToDoubleVector(jointNode->first_node("orientation_in_parent")->value());
        ChQuaternion<> jointOrientationInParent = Q_from_AngX(elems[0]) * Q_from_AngY(elems[1]) * Q_from_AngZ(elems[2]);

        // Parent to joint in parent
        ChFrame<> X_P_F(jointPosInParent, jointOrientationInParent);

        // Offset location and orientation caused by joint initial configuration, default is identity
        ChVector<> t_F_M(0, 0, 0);
        ChQuaternion<> R_F_M(1, 0, 0, 0);

        // Get offsets, depending on joint type
        if (std::string(jointNode->name()) == std::string("PinJoint")) {
            xml_node<>* coordinates =
                jointNode->first_node("CoordinateSet")->first_node("objects")->first_node("Coordinate");
            double thetaZ = std::stod(coordinates->first_node("default_value")->value());
            R_F_M = Q_from_AngZ(thetaZ);
        } else if ((std::string(jointNode->name()) == std::string("WeldJoint"))) {
            // Do absolutely nothing, they're stuck together
        } else if ((std::string(jointNode->name()) == std::string("UniversalJoint"))) {
            // Do some universal magic here
            // Coords go Z then X rotation
            xml_node<>* coordinates =
                jointNode->first_node("CoordinateSet")->first_node("objects")->first_node("Coordinate");
            double thetaX = std::stod(coordinates->first_node("default_value")->value());
            coordinates = coordinates->next_sibling();
            double thetaY = std::stod(coordinates->first_node("default_value")->value());
            R_F_M = Q_from_AngX(thetaX) * Q_from_AngY(thetaY);
        } else if ((std::string(jointNode->name()) == std::string("BallJoint"))) {
            // X Y Z
            xml_node<>* coordinates =
                jointNode->first_node("CoordinateSet")->first_node("objects")->first_node("Coordinate");
            double thetaX = std::stod(coordinates->first_node("default_value")->value());
            coordinates = coordinates->next_sibling();
            double thetaY = std::stod(coordinates->first_node("default_value")->value());
            coordinates = coordinates->next_sibling();
            double thetaZ = std::stod(coordinates->first_node("default_value")->value());
            R_F_M = Q_from_AngX(thetaX) * Q_from_AngY(thetaY) * Q_from_AngZ(thetaZ);
        } else if ((std::string(jointNode->name()) == std::string("CustomJoint"))) {
            // Cry
        }

        // Joint in parent to joint in child
        ChFrame<> X_F_M(t_F_M, R_F_M);

        // Get joint's position in child frame
        elems = ChParserOpenSim::strToDoubleVector(jointNode->first_node("location")->value());
        ChVector<> jointPosInChild = ChVector<>(elems[0], elems[1], elems[2]);

        // Get joint's orientation in child frame
        elems = ChParserOpenSim::strToDoubleVector(jointNode->first_node("orientation")->value());
        ChQuaternion<> jointOrientationInChild = Q_from_AngX(elems[0]) * Q_from_AngY(elems[1]) * Q_from_AngZ(elems[2]);

        // Joint in child to child
        ChFrame<> X_B_M(jointPosInChild, jointOrientationInChild);

        auto X_G_B = X_G_P * X_P_F * X_F_M * X_B_M.GetInverse();
        // Set body frame, not necessarily centroidal
        newBody->SetFrame_REF_to_abs(X_G_B);
        // newBody->SetCoord(X_G_B.GetCoord());

        // ChVector<> jointPosGlobal = parentPos + parentOrientation.GetInverse().Rotate(jointPosInParent);
        // ChQuaternion<> jointOrientationGlobal = jointOrientationInParent * parentOrientation;
        //
        ChFrame<> jointCoords = X_G_P * X_P_F;
        //
        // newBody->SetCoord(X_G_P * X_P_F * X_F_M * X_M_B);

        // newBody->SetRot(parentOrientation * jointOrientationInParent * R_F_M *
        //                 jointOrientationInChild.GetInverse());
        //
        // newBody->SetPos(jointPosGlobal - newBody->TransformPointLocalToParent(jointPosInChild));

        assert(std::abs(newBody->GetRot().Length() - 1) < 1e-10);

        std::cout << "Parent is at global " << parent->GetPos().x() << "," << parent->GetPos().y() << ","
                  << parent->GetPos().z() << "|" << parent->GetRot().e0() << "," << parent->GetRot().e1() << ","
                  << parent->GetRot().e2() << "," << parent->GetRot().e3() << std::endl;
        std::cout << "Joint is at parent " << jointPosInParent.x() << "," << jointPosInParent.y() << ","
                  << jointPosInParent.z() << "|" << jointOrientationInParent.e0() << ","
                  << jointOrientationInParent.e1() << "," << jointOrientationInParent.e2() << ","
                  << jointOrientationInParent.e3() << std::endl;
        std::cout << "Joint is at global " << jointCoords.GetPos().x() << "," << jointCoords.GetPos().y() << ","
                  << jointCoords.GetPos().z() << "|" << jointCoords.GetRot().e0() << "," << jointCoords.GetRot().e1()
                  << "," << jointCoords.GetRot().e2() << "," << jointCoords.GetRot().e3() << std::endl;
        std::cout << "Joint is at child " << jointPosInChild.x() << "," << jointPosInChild.y() << ","
                  << jointPosInChild.z() << "|" << jointOrientationInChild.e0() << "," << jointOrientationInChild.e1()
                  << "," << jointOrientationInChild.e2() << "," << jointOrientationInChild.e3() << std::endl;
        std::cout << "Putting body " << newBody->GetName() << " at " << newBody->GetPos().x() << ","
                  << newBody->GetPos().y() << "," << newBody->GetPos().z() << std::endl;
        std::cout << "Orientation is " << newBody->GetRot().e0() << "," << newBody->GetRot().e1() << ","
                  << newBody->GetRot().e2() << "," << newBody->GetRot().e3() << std::endl;

        // Make a joint, depending on what it actually is
        if (std::string(jointNode->name()) == std::string("PinJoint")) {
            std::cout << "Pin joint!" << std::endl;
            auto joint = std::make_shared<ChLinkLockRevolute>();
            joint->Initialize(parent, newBody, jointCoords.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            my_system.AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("WeldJoint"))) {
            std::cout << "Weld joint!" << std::endl;
            auto joint = std::make_shared<ChLinkLockLock>();
            joint->Initialize(parent, newBody, jointCoords.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            my_system.AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("UniversalJoint"))) {
            // Do some universal magic here
            std::cout << "Universal joint!" << std::endl;
            auto joint = std::make_shared<ChLinkUniversal>();
            joint->Initialize(parent, newBody, jointCoords);
            joint->SetNameString(jointNode->first_attribute("name")->value());
            my_system.AddLink(joint);
            m_jointList.push_back(joint);

        } else if ((std::string(jointNode->name()) == std::string("BallJoint"))) {
            std::cout << "Ball joint!" << std::endl;
            auto joint = std::make_shared<ChLinkLockSpherical>();
            joint->Initialize(parent, newBody, jointCoords.GetCoord());
            joint->SetNameString(jointNode->first_attribute("name")->value());
            my_system.AddLink(joint);
            m_jointList.push_back(joint);

        } else {
            // Cry
            std::cout << "Unknown Joint type " << jointNode->name() << " between " << parent->GetName() << " and "
                      << newBody->GetName() << "Making spherical standin." << std::endl;
            auto joint = std::make_shared<ChLinkLockSpherical>();
            joint->Initialize(parent, newBody, jointCoords.GetCoord());
            joint->SetNameString(std::string(jointNode->first_attribute("name")->value()) + "_standin");
            my_system.AddLink(joint);
            m_jointList.push_back(joint);
        }

        // std::cout << "putting joint at " << jointPosGlobal.x() << "," << jointPosGlobal.y() << "," <<
        // jointPosGlobal.z()
        //           << "|" << jointOrientationGlobal.e0() << "," << jointOrientationGlobal.e1() << ","
        //           << jointOrientationGlobal.e2() << "," << jointOrientationGlobal.e3() << std::endl;

    };
    // function_table["PinJoint"] = [](xml_node<>* fieldNode, ChSystem& my_system, std::shared_ptr<ChBodyAuxRef>
    // newBody) {
    //
    //     auto joint = std::make_shared<ChLinkLockRevolute>();
    //     joint->Initialize(otherBody, newBody, ChCoordsys<>(ChVector<>(0, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
    //
    // };

    function_table["VisibleObject"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                         std::shared_ptr<ChBodyAuxRef> newBody) {
        // Not needed maybe?
    };
    function_table["WrapObjectSet"] = [](xml_node<>* fieldNode, const ChSystem& my_system,
                                         std::shared_ptr<ChBodyAuxRef> newBody) {
        // I don't think we need this either
    };
}

void ChParserOpenSim::initVisualizations(xml_node<>* node, ChSystem& p_system) {
    std::cout << "Assembling Primitives " << std::endl;
    switch (m_visType) {
        case VisType::PRIMITIVES:
            for (auto link : m_jointList) {
                std::cout << link->GetName() << std::endl;
                auto linkCoords = link->GetLinkAbsoluteCoords();
                std::cout << "Link is at " << linkCoords.pos.x() << "," << linkCoords.pos.y() << ","
                          << linkCoords.pos.z() << std::endl;
                auto parent = dynamic_cast<ChBodyAuxRef*>(link->GetBody1());
                auto child = dynamic_cast<ChBodyAuxRef*>(link->GetBody2());
                {
                    auto parent_joint_cyl = std::make_shared<ChCylinderShape>();
                    parent_joint_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
                    parent_joint_cyl->GetCylinderGeometry().p2 =
                        parent->GetFrame_REF_to_abs().TransformPointParentToLocal(linkCoords.pos);
                    std::cout << "Length is " << parent_joint_cyl->GetCylinderGeometry().p2.Length() << std::endl;
                    parent_joint_cyl->GetCylinderGeometry().rad = 0.1;
                    if (parent_joint_cyl->GetCylinderGeometry().p2.Length() > 1e-5) {
                        parent->AddAsset(parent_joint_cyl);
                    }
                }
                {
                    auto joint_child_cyl = std::make_shared<ChCylinderShape>();
                    joint_child_cyl->GetCylinderGeometry().p1 =
                        child->GetFrame_REF_to_abs().TransformPointParentToLocal(linkCoords.pos);
                    joint_child_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
                    std::cout << "Length is " << joint_child_cyl->GetCylinderGeometry().p1.Length() << std::endl;
                    joint_child_cyl->GetCylinderGeometry().rad = 0.1;
                    if (joint_child_cyl->GetCylinderGeometry().p1.Length() > 1e-5) {
                        child->AddAsset(joint_child_cyl);
                    }
                }
            }
            break;
        case VisType::MESH:
            // Parse node by node
            break;
    }
}

}  // end namespace utils
}  // end namespace chrono
