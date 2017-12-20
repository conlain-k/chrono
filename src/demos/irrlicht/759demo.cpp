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

#include "stopwatch.hpp"
#include "tritri.hpp"

using std::max;
using std::min;

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

bool useBroadphase = true;

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
    ChBody& mFrame;
    // Store AABB mins (xmin,ymin,zmin)
    double AABBmin[3];
    // Store AABB maxes (xmax,ymax,zmax)
    double AABBmax[3];
    Point* mPoints;

} mesh_t;

void printPoint(const Point& p) {
    printf("%f, %f, %f\n", p[0], p[1], p[2]);
}
void copyVectorToPointOffset(const ChVector<>& source, Point& dest) {
    dest[0] = source.x();
    dest[1] = source.y();
    dest[2] = source.z();
}

// Copy all vertices into points, with frame transform
void copyVerticesToPoints(mesh_t& mesh) {
    ChVector<int> i;
    // Pull into local scope for cache-friendly

    const int size = mesh.indices.size();
    auto frame = mesh.mFrame;
    auto points = mesh.mPoints;
    auto vertices = mesh.vertices;

#pragma omp parallel for private(i)
    for (int f1 = 0; f1 < size; f1++) {
        i = mesh.indices[f1];

        // Copy first vertex into V1
        copyVectorToPointOffset(frame * (vertices[i.x()]), points[i.x()]);
        copyVectorToPointOffset(frame * (vertices[i.y()]), points[i.y()]);
        copyVectorToPointOffset(frame * (vertices[i.z()]), points[i.z()]);
    }
}

// Create AABB for a mesh
void mesh_t_compute_AABB(mesh_t& mesh) {
    double xmin = DBL_MAX, xmax = -DBL_MAX, ymin = DBL_MAX, ymax = -DBL_MAX, zmin = DBL_MAX, zmax = -DBL_MAX;

    // Pull into local scope for cache-friendly
    const int size = mesh.indices.size();
    auto points = mesh.mPoints;

    ChVector<int> i1;
// std::cout << "size is " << size << std::endl;
#pragma omp parallel for reduction(min : xmin, ymin, zmin), reduction(max : xmax, ymax, zmax) private(i1)
    for (int f1 = 0; f1 < size; f1++) {
        auto i1 = mesh.indices[f1];
        // Point P1, P2, P3;
        Point& P1 = points[i1.x()];
        Point& P2 = points[i1.y()];
        Point& P3 = points[i1.z()];
        xmin = min(xmin, min(P1[0], min(P2[0], P3[0])));
        ymin = min(ymin, min(P1[1], min(P2[1], P3[1])));
        zmin = min(zmin, min(P1[2], min(P2[2], P3[2])));
        xmax = max(xmax, max(P1[0], max(P2[0], P3[0])));
        ymax = max(ymax, max(P1[1], max(P2[1], P3[1])));
        zmax = max(zmax, max(P1[2], max(P2[2], P3[2])));
    }
    mesh.AABBmin[0] = xmin;
    mesh.AABBmin[1] = ymin;
    mesh.AABBmin[2] = zmin;
    mesh.AABBmax[0] = xmax;
    mesh.AABBmax[1] = ymax;
    mesh.AABBmax[2] = zmax;
}

// Test if 2 AABBs collide
bool mesh_t_AABB_collide(const mesh_t& m1, const mesh_t& m2) {
    // bool x = (m1.AABBmin[0] <= m2.AABBmax[0] && m1.AABBmax[0] >= m2.AABBmin[0]);
    // bool y = (m1.AABBmin[1] <= m2.AABBmax[1] && m1.AABBmax[1] >= m2.AABBmin[1]);
    // bool z = (m1.AABBmin[2] <= m2.AABBmax[2] && m1.AABBmax[2] >= m2.AABBmin[2]);
    // printf("x,y,z is %d,%d,%d\n", x, y, z);
    return (m1.AABBmin[0] <= m2.AABBmax[0] && m1.AABBmax[0] >= m2.AABBmin[0]) &&
           (m1.AABBmin[1] <= m2.AABBmax[1] && m1.AABBmax[1] >= m2.AABBmin[1]) &&
           (m1.AABBmin[2] <= m2.AABBmax[2] && m1.AABBmax[2] >= m2.AABBmin[2]);
}
bool mesh_t_broadphase(mesh_t& mesh1, mesh_t& mesh2) {
    mesh_t_compute_AABB(mesh1);
    mesh_t_compute_AABB(mesh2);
    bool broad = false;
    if (broad = mesh_t_AABB_collide(mesh1, mesh2)) {
        std::cout << "broadphase is true" << std::endl;
    } else {
        // std::cout << "broadphase is false" << std::endl;
    }
    return broad;
}

int mesh_t_compute_contacts_fast(mesh_t& mesh1, mesh_t& mesh2) {
    copyVerticesToPoints(mesh1);
    copyVerticesToPoints(mesh2);
    bool broad = mesh_t_broadphase(mesh1, mesh2);
    if (useBroadphase && !broad) {
        // std::cout << "exiting early\n";
        return 0;
    }
    int ncontacts = 0;

    Point* m1Points = mesh1.mPoints;
    Point* m2Points = mesh2.mPoints;
    // So much indirection has to hurt the cache
    const int s1 = mesh1.indices.size();
    const int s2 = mesh2.indices.size();
    ChVector<int> i1, i2;

#pragma omp parallel for reduction(+ : ncontacts) private(i1, i2) schedule(dynamic)
    for (int f1 = 0; f1 < s1; f1++) {
        for (int f2 = 0; f2 < s2; f2++) {
            i1 = mesh1.indices[f1];
            i2 = mesh2.indices[f2];
            // if (NoDivTriTriIsect(U1, U2, U3, V1, V2, V3) == 1) {
            if (NoDivTriTriIsect(m2Points[i2.x()], m2Points[i2.y()], m2Points[i2.z()], m1Points[i1.x()],
                                 m1Points[i1.y()], m1Points[i1.z()]) == 1) {
                ncontacts++;
                printf("collision occured between mesh1 at face %d, mesh2 at face %d \n", f1, f2);
            }
        }
    }
    if (!broad && ncontacts != 0) {
        printf("broadphase failed\n");
        exit(1);
    }
    return ncontacts;
}

int mesh_t_compute_contacts_faster(const mesh_t& mesh1,
                                   const mesh_t& mesh2,
                                   const ChFrame<>& frame1,
                                   const ChFrame<>& frame2) {
    int ncontacts = 0;

    // Point V1, V2, V3;
    // Point U1, U2, U3;
    // This might have a race condition
    // #pragma omp parallel for reduction(+ : ncontacts) private(V1, V2, V3, U1, U2, U3, i1, i2) shared(mesh1, mesh2,
    // frame1, frame2) schedule(dynamic)d
    Point* m1Points = mesh1.mPoints;
    Point* m2Points = mesh2.mPoints;

#pragma omp parallel for
    for (int f2 = 0; f2 < mesh2.indices.size(); f2++) {
        auto i2 = mesh2.indices[f2];
        copyVectorToPointOffset(frame2 * (mesh2.vertices[i2.x()]), m2Points[i2.x()]);
        copyVectorToPointOffset(frame2 * (mesh2.vertices[i2.y()]), m2Points[i2.y()]);
        copyVectorToPointOffset(frame2 * (mesh2.vertices[i2.z()]), m2Points[i2.z()]);
    }

    ChVector<int> i1, i2;
#pragma omp parallel for reduction(+ : ncontacts) private(i1, i2) shared(mesh1, mesh2) schedule(dynamic)
    for (int f1 = 0; f1 < mesh1.indices.size(); f1++) {
        auto i1 = mesh1.indices[f1];
        // Another optimization -- use local points for outer loop since copy is only once
        Point V1, V2, V3;
        copyVectorToPointOffset(frame1 * (mesh1.vertices[i1.x()]), V1);
        copyVectorToPointOffset(frame1 * (mesh1.vertices[i1.y()]), V2);
        copyVectorToPointOffset(frame1 * (mesh1.vertices[i1.z()]), V3);
        // #pragma omp parallel for reduction(+ : ncontacts) private(U1, U2, U3, i2)
        for (int f2 = 0; f2 < mesh2.indices.size(); f2++) {
            auto i2 = mesh2.indices[f2];
            // if (NoDivTriTriIsect(U1, U2, U3, V1, V2, V3) == 1) {
            if (NoDivTriTriIsect(m2Points[i2.x()], m2Points[i2.y()], m2Points[i2.z()], m1Points[i1.x()],
                                 m1Points[i1.y()], m1Points[i1.z()]) == 1) {
                ncontacts++;
                printf("collision occured between mesh1 at face %d, mesh2 at face %d \n", f1, f2);
            }
        }
    }
    return ncontacts;
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the system.
    ChSystemNSC system;
    // system.SetSolverType(ChSolver::Type::SOR);
    // system.SetMaxItersSolverSpeed(20);
    // system.SetMaxItersSolverStab(5);
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.001);
    system.Set_G_acc({0, 0, 0});

    // Create the Irrlicht application.
    ChIrrApp application(&system, L"Number of collisions", irr::core::dimension2d<irr::u32>(800, 600), false);
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, 10, 10));

    ChTriangleMeshConnected bunnymesh;
    bunnymesh.LoadWavefrontMesh(GetChronoDataFile("bunny.obj"), true, true);
    bunnymesh.Transform({0, 0, 0}, ChMatrix33<>(50.0));

    ChTriangleMeshConnected bunnymesh2;
    bunnymesh2.LoadWavefrontMesh(GetChronoDataFile("bunny.obj"), true, true);
    bunnymesh2.Transform({0, 0, 0}, ChMatrix33<>(50.0));

    auto bunny1 = std::make_shared<ChBody>();
    bunny1->SetPos(ChVector<>(-5, 0, 0));
    bunny1->SetRot(QUNIT);
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
    // Rotate at strange angle to remove symmetry
    bunny2->SetRot(Q_from_AngY(-.1));
    bunny2->SetPos_dt(ChVector<>(-2, 0, 0));

    bunny2->SetBodyFixed(false);
    system.AddBody(bunny2);

    bunny2->GetCollisionModel()->ClearModel();
    bunny2->GetCollisionModel()->AddTriangleMesh(bunnymesh2, false, false, VNULL, ChMatrix33<>(1), 0.005);
    bunny2->GetCollisionModel()->BuildModel();
    bunny2->SetCollide(true);

    auto bunnyasset2 = std::make_shared<ChTriangleMeshShape>();
    bunnyasset2->SetMesh(bunnymesh2);
    bunny2->AddAsset(bunnyasset2);

    // In this example the two meshes are the same, the offsets are just different
    mesh_t mesh1 = {bunnymesh.m_vertices, bunnymesh.m_normals, bunnymesh.m_face_v_indices, *bunny1};
    mesh1.mPoints = (Point*)malloc(mesh1.vertices.size() * sizeof(Point));
    mesh_t mesh2 = {bunnymesh2.m_vertices, bunnymesh2.m_normals, bunnymesh2.m_face_v_indices, *bunny2};
    mesh2.mPoints = (Point*)malloc(mesh2.vertices.size() * sizeof(Point));

    // Complete visualization asset construction.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Create the contact manager.
    ContactManager manager(&system);

    double timestep = .01;
    // Simulation loop.
    application.SetStepManage(true);
    application.SetTimestep(timestep);

    stopwatch<std::milli, double> sw;

    while (application.GetDevice()->run()) {
        application.BeginScene();

        // Render scene.
        application.DrawAll();

        sw.start();
        int ncontacts = mesh_t_compute_contacts_fast(mesh1, mesh2);
        sw.stop();
        double my_time = sw.count();

        // sw.start();
        // int ncontacts1 = mesh_t_compute_contacts_faster(mesh1, mesh2, ChFrame<>(*bunny1), ChFrame<>(*bunny2));
        // sw.stop();
        // double my_time1 = sw.count();
        //
        // if (ncontacts != ncontacts1) {
        //     std::cerr << "dammit" << std::endl;
        //     exit(1);
        // }

        // Advance dynamics.
        application.DoStep();

        // Process current collisions and report number of contacts on a few bodies.
        manager.Process();
        if (timestep == .05 && (bunny2->GetPos() - bunny1->GetPos()).x() < 7.2) {
            timestep = .005;
            application.SetTimestep(timestep);
        }
        auto narrow = system.GetTimerCollisionNarrow();
        auto broad = system.GetTimerCollisionBroad();
        auto chrono_time = narrow + broad;

        std::cout << "offset is " << (bunny2->GetPos() - bunny1->GetPos()).x() << std::endl;
        std::cout << "time ratio is " << my_time / (chrono_time * 1000) << std::endl;
        if (manager.GetNcontacts(bunny1) != 0 || ncontacts != 0) {
            std::cout << "fast took " << my_time / 1000 << " s !" << std::endl;
            std::cout << "chrono time is " << chrono_time << " s !" << std::endl;
            std::cout << "narrow is " << narrow << " broad is " << broad << std::endl;
            // std::cout << "new took " << my_time1 / 1000 << " s!" << std::endl;
            printf("ncontacts is %d\n", ncontacts);
            std::cout << "Time: " << system.GetChTime();
            std::cout << "   bunny1: " << manager.GetNcontacts(bunny1);
            std::cout << "   bunny2: " << manager.GetNcontacts(bunny2);
            std::cout << std::endl;
        }
        application.EndScene();
    }
    delete[] mesh1.mPoints;
    delete[] mesh2.mPoints;

    return 0;
}