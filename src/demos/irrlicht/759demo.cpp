//
// =============================================================================
// Authors: Conlain Kelly
// =============================================================================
//
// Used to test collision detection for 759 final project
//
// =============================================================================

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "tritri.hpp"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

// Callback class for contact processing.
class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager(ChSystem* system) : m_system(system) {}

    // Return the current total number of contacts experienced by the specified body.
    unsigned int GetNcontacts(std::shared_ptr<ChBody> body) const {
        auto search = m_bcontacts.find(body.get());
        return (search == m_bcontacts.end()) ? 0 : search->second;
    }

    // Process all contacts at current time.
    // Reset the hash map and invoke the callback for each collision.
    void Process() {
        m_bcontacts.clear();
        m_system->GetContactContainer()->ReportAllContacts(this);
    }

  private:
    // Keep track of the number of contacts experienced by each body.
    // Maintain a hash map with body as key and current number of contacts as value.
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        auto bodyA = static_cast<ChBody*>(modA);
        auto searchA = m_bcontacts.find(bodyA);
        if (searchA == m_bcontacts.end())
            m_bcontacts.insert(std::make_pair(bodyA, 1));
        else
            searchA->second++;

        auto bodyB = static_cast<ChBody*>(modB);
        auto searchB = m_bcontacts.find(bodyB);
        if (searchB == m_bcontacts.end())
            m_bcontacts.insert(std::make_pair(bodyB, 1));
        else
            searchB->second++;

        return true;
    }

    ChSystem* m_system;
    std::unordered_map<ChBody*, unsigned int> m_bcontacts;
};

// A single point in 3D space
typedef double Point[3];

typedef struct {
    const std::vector<ChVector<double>> vertices;
    const std::vector<ChVector<double>> normals;
    const std::vector<ChVector<int>> indices;

} mesh_t;

void printPoint(const Point& p) {
    printf("%f, %f, %f\n", p[0], p[1], p[2]);
}
void copyVectorToPointOffset(const ChVector<>& source, Point& dest, const ChVector<>& offset) {
    dest[0] = source.x() + offset[0];
    dest[1] = source.y() + offset[1];
    dest[2] = source.z() + offset[2];
    // printPoint(source);
    // printPoint(dest);
}

// Copy a point to another, possibly with an offset
void copyPoint(const Point& source, Point& dest, const Point& offset) {
    dest[0] = source[0] + offset[0];
    dest[1] = source[1] + offset[1];
    dest[2] = source[2] + offset[2];
    printPoint(source);
    printPoint(dest);
}

int mesh_t_compute_contacts(const mesh_t& mesh1, const mesh_t& mesh2, const ChVector<>& pos1, const ChVector<>& pos2) {
    int ncontacts = 0;
	printf("distance is %f\n", (pos1 - pos2).Length());
    // For each face in the first mesh
	
    for (int f1 = 0; f1 < mesh1.indices.size(); f1++) {
        auto i1 = mesh1.indices[f1];
        // printf("indices are %d, %d, %d\n", i.x(), i.y(), i.z());
        Point V1, V2, V3;
        // auto V1 = vertices[i.x()];
        copyVectorToPointOffset(mesh1.vertices[i1.x()], V1, pos1);
        copyVectorToPointOffset(mesh1.vertices[i1.y()], V2, pos1);
        copyVectorToPointOffset(mesh1.vertices[i1.z()], V3, pos1);
        for (int f2 = 0; f2 < mesh1.indices.size(); f2++) {
            auto i2 = mesh2.indices[f2];
            // printf("indices are %d, %d, %d\n", i2.x(), i2.y(), i2.z());
            Point U1, U2, U3;
            // auto V1 = vertices[i.x()];
            copyVectorToPointOffset(mesh2.vertices[i2.x()], U1, pos2);
            copyVectorToPointOffset(mesh2.vertices[i2.y()], U2, pos2);
            copyVectorToPointOffset(mesh2.vertices[i2.z()], U3, pos2);
            if (tri_tri_intersect(U1, U2, U3, V1, V2, V3) == 1) {
                ncontacts ++;
                printf("collision occured between mesh1 at face %d, mesh2 at face %d \n", f1, f2);
            }
        }
    }
	return ncontacts;
    // printf("vertex %d is (%f, %f, %f)\n", );
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the system.
    ChSystemNSC system;
    // system.SetSolverType(ChSolver::Type::SOR);
    // system.SetMaxItersSolverSpeed(20);
    // system.SetMaxItersSolverStab(5);
    // collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    // collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);
    system.Set_G_acc({0, 0, 0});

    // Create the Irrlicht application.
    ChIrrApp application(&system, L"Number of collisions", irr::core::dimension2d<irr::u32>(800, 600), false);
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(10, 10, 0));

    ChTriangleMeshConnected bunnymesh;
    bunnymesh.LoadWavefrontMesh("../../../759-final-project/data/bunny.obj", true, true);
    bunnymesh.Transform({0, 0, 0}, ChMatrix33<>(50.0));

    auto vertices = bunnymesh.m_vertices;
    auto normals = bunnymesh.m_normals;
    auto indices = bunnymesh.m_face_v_indices;

    // In this example the two meshes are the same, the offsets are just different
    mesh_t mesh1 = {vertices, normals, indices};
    mesh_t mesh2 = {vertices, normals, indices};
    mesh_t_compute_contacts(mesh1, mesh2, {0, 0, 0}, {20, 0, 0});

    // std::vector<Point[3]> points1;
    // points1.reserve(indices.size());

    // // For each face
    // for (int f = 0; f < indices.size(); f++) {
    //     auto i = indices[f];
    //     printf("indices are %d, %d, %d", i.x(), i.y(), i.z());
    //     // For each vertex in the face
    //     Point* tri1 = points1[f];
    //     copyVectorToPointOffset(vertices[i.x()], tri1[0], {-10, 0, 0});
    //     copyVectorToPointOffset(vertices[i.y()], tri1[1], {-10, 0, 0});
    //     copyVectorToPointOffset(vertices[i.z()], tri1[2], {-10, 0, 0});
    //     printPoint(points1[f][0]);
    // }

    auto bunny1 = std::make_shared<ChBody>();
    bunny1->SetPos(ChVector<>(-5, 0, 0));
    bunny1->SetPos_dt(ChVector<>(2, 0, 0));
    bunny1->SetBodyFixed(false);
    system.AddBody(bunny1);

    bunny1->GetCollisionModel()->ClearModel();
    bunny1->GetCollisionModel()->AddTriangleMesh(bunnymesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
    bunny1->GetCollisionModel()->BuildModel();
    bunny1->SetCollide(true);

    auto bunnyasset = std::make_shared<ChTriangleMeshShape>();
    bunnyasset->SetMesh(bunnymesh);
    bunny1->AddAsset(bunnyasset);

    auto bunny2 = std::make_shared<ChBody>();
    bunny2->SetPos(ChVector<>(5, 0, 0));
    bunny2->SetPos_dt(ChVector<>(-2, 0, 0));

    bunny2->SetBodyFixed(false);
    system.AddBody(bunny2);

    bunny2->GetCollisionModel()->ClearModel();
    bunny2->GetCollisionModel()->AddTriangleMesh(bunnymesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
    bunny2->GetCollisionModel()->BuildModel();
    bunny2->SetCollide(true);

    auto bunnyasset2 = std::make_shared<ChTriangleMeshShape>();
    bunnyasset2->SetMesh(bunnymesh);
    bunny2->AddAsset(bunnyasset2);

    // Complete visualization asset construction.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Create the contact manager.
    ContactManager manager(&system);

    // Simulation loop.
    application.SetStepManage(true);
    application.SetTimestep(0.02);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        // Render scene.
        application.DrawAll();

        // Advance dynamics.
        application.DoStep();

        // Process current collisions and report number of contacts on a few bodies.
        manager.Process();
        int ncontacts = mesh_t_compute_contacts(mesh1, mesh2, bunny1->GetPos(), bunny2->GetPos());
        if (manager.GetNcontacts(bunny1) != 0) {
            printf("ncontacts is %d\n", ncontacts);
            std::cout << "Time: " << system.GetChTime();
            std::cout << "   bunny1: " << manager.GetNcontacts(bunny1);
            std::cout << "   bunny2: " << manager.GetNcontacts(bunny2);
            std::cout << std::endl;
            std::cout << "offset is " << (bunny1->GetPos() - bunny2->GetPos()).x() << std::endl;
        }
        application.EndScene();
    }

    return 0;
}
