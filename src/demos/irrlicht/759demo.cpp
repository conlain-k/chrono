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
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include <omp.h>
#include <unordered_set>
#include <vector>
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

// Bin along X for now
#define NBINS 20
#define BIN_EPSILON .001
#define USE_BROADPHASE true
#define USE_HISTORY true

// Callback class for contact processing.
class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager(ChSystem* my_system) : m_system(my_system) {}

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

// crappy hash function for a pair so it can be used in a set
// A pair already defines equality, so we don't need that
struct hash_pair_functor {
    inline std::size_t operator()(const std::pair<int, int>& p) const { return p.first * 37 + p.second; }
};

// A single point in 3D space
typedef double Point[3];

struct bbox {
    // Store AABB mins (xmin,ymin,zmin)
    Point min;

    // Store AABB maxes (xmax,ymax,zmax)
    Point max;
};

struct mesh_t {
    // Get obj data from Chrono
    const std::vector<ChVector<double>> vertices;
    const std::vector<ChVector<double>> normals;
    const std::vector<ChVector<int>> indices;
    // Host body
    ChBody& mFrame;
    // Start as null quaternion
    ChQuaternion<> oldRot;
    // Holds bounding info
    bbox AABB;
    // Hold bin dimensions
    bbox bins[NBINS];
    // Hold indices of faces inside each bin
    std::vector<ChVector<int>> binIndices[NBINS];
    // Reverse mapping of bin indices to faces
    std::vector<int> reverseIndices[NBINS];
    // List of copied points
    Point* mPoints;
};

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

bool Point_inside_AABB(const bbox& box, const Point& point) {
    // function isPointInsideAABB(point, box) {
    return (point[0] >= box.min[0] && point[0] <= box.max[0]) && (point[1] >= box.min[1] && point[1] <= box.max[1]) &&
           (point[2] >= box.min[2] && point[2] <= box.max[2]);
}

// Test if 2 AABBs collide
bool AABB_overlap(const bbox& b1, const bbox& b2) {
    // bool x = b1.min[0] <= b2.max[0] && b1.max[0] >= b2.min[0];
    // bool y = b1.min[1] <= b2.max[1] && b1.max[1] >= b2.min[1];
    // bool z = b1.min[2] <= b2.max[2] && b1.max[2] >= b2.min[2];
    // printf("x,y,z is %d,%d,%d\n", x, y, z);
    return (b1.min[0] <= b2.max[0] && b1.max[0] >= b2.min[0]) && (b1.min[1] <= b2.max[1] && b1.max[1] >= b2.min[1]) &&
           (b1.min[2] <= b2.max[2] && b1.max[2] >= b2.min[2]);
}

void mesh_t_bin_vertices(mesh_t& mesh) {
    // Clear the old ones
    for (int i = 0; i < NBINS; i++) {
        mesh.binIndices[i].clear();
        mesh.reverseIndices[i].clear();
    }

    // Pull into local scope for cache-friendly
    const int size = mesh.indices.size();
    auto points = mesh.mPoints;

    ChVector<int> i1;
    // std::cout << "size is " << size << std::endl;
    // #pragma omp parallel for private(i1)
    for (int bin = 0; bin < NBINS; bin++) {
        for (int f1 = 0; f1 < size; f1++) {
            i1 = mesh.indices[f1];
            // Point P1, P2, P3;
            Point& P1 = points[i1.x()];
            Point& P2 = points[i1.y()];
            Point& P3 = points[i1.z()];
            // if point is in bin, add its index
            if (Point_inside_AABB(mesh.bins[bin], P1) || Point_inside_AABB(mesh.bins[bin], P2) ||
                Point_inside_AABB(mesh.bins[bin], P3)) {
                // std::cout << "face" << f1 << " is in bin " << bin << std::endl;
                mesh.binIndices[bin].push_back(i1);
                mesh.reverseIndices[bin].push_back(f1);
            }
        }
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
        i1 = mesh.indices[f1];
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
    mesh.AABB.min[0] = xmin;
    mesh.AABB.min[1] = ymin;
    mesh.AABB.min[2] = zmin;
    mesh.AABB.max[0] = xmax;
    mesh.AABB.max[1] = ymax;
    mesh.AABB.max[2] = zmax;

    // Width of bin (maybe)
    double xwidth = (xmax - xmin) / NBINS;
    double ywidth = (ymax - ymin) / NBINS;
    double zwidth = (zmax - zmin) / NBINS;
    // Only bin along x for now
    // Add a small epsilon so we don't accidentally
    for (int i = 0; i < NBINS; i++) {
        mesh.bins[i].min[0] = xmin + (xwidth * i) - BIN_EPSILON;
        mesh.bins[i].min[1] = ymin - BIN_EPSILON;  // full y height
        mesh.bins[i].min[2] = zmin - BIN_EPSILON;  // full z width
        mesh.bins[i].max[0] = mesh.bins[i].min[0] + xwidth + 2 * BIN_EPSILON;
        mesh.bins[i].max[1] = ymax + BIN_EPSILON;  // full y height
        mesh.bins[i].max[2] = zmax + BIN_EPSILON;  // full z width
    }
}

bool mesh_t_broadphase(mesh_t& mesh1, mesh_t& mesh2) {
    mesh_t_compute_AABB(mesh1);
    mesh_t_compute_AABB(mesh2);

    return AABB_overlap(mesh1.AABB, mesh2.AABB);
}

// Return a list of contacts as <mesh1_face, mesh2_face>
// Uses multithreading but not binning
std::vector<std::pair<int, int>> mesh_t_narrowphase(mesh_t& mesh1, mesh_t& mesh2) {
    int nthreads = omp_get_max_threads();
    // Hold contact pairs for each thread
    auto* contact_pair_bins = new std::vector<std::pair<int, int>>[nthreads];

    // Final vector to return
    std::vector<std::pair<int, int>> contact_pairs_vector;

    Point* m1Points = mesh1.mPoints;
    Point* m2Points = mesh2.mPoints;
    // So much indirection has to hurt the cache
    const int s1 = mesh1.indices.size();
    const int s2 = mesh2.indices.size();
    ChVector<int> i1, i2;

#pragma omp parallel for private(i1, i2) schedule(dynamic)
    for (int f1 = 0; f1 < s1; f1++) {
        for (int f2 = 0; f2 < s2; f2++) {
            i1 = mesh1.indices[f1];
            i2 = mesh2.indices[f2];
            // if (NoDivTriTriIsect(U1, U2, U3, V1, V2, V3) == 1) {
            if (NoDivTriTriIsect(m2Points[i2.x()], m2Points[i2.y()], m2Points[i2.z()], m1Points[i1.x()],
                                 m1Points[i1.y()], m1Points[i1.z()]) == 1) {
                contact_pair_bins[omp_get_thread_num()].push_back(std::pair<int, int>(f1, f2));
            }
        }
    }
    // This is slightly very gross
    for (int b = 0; b < nthreads; b++) {
        contact_pairs_vector.insert(contact_pairs_vector.end(), contact_pair_bins[b].begin(),
                                    contact_pair_bins[b].end());
    }

    // Free alloced mem since this can't be done statically
    delete[] contact_pair_bins;
    return contact_pairs_vector;
}

// Return a list of contacts as <mesh1_face, mesh2_face>
// Uses binning to speed up computation
std::vector<std::pair<int, int>> mesh_t_narrowphase_binning(mesh_t& mesh1, mesh_t& mesh2) {
    int nthreads = omp_get_max_threads();
    // Hold contact pairs for each thread
    auto* contact_pair_bins = new std::vector<std::pair<int, int>>[nthreads];
    // Final vector to return
    std::vector<std::pair<int, int>> contact_pairs_vector;
    // Construct set (all elements unique) so that we only get unique collisions
    std::unordered_set<std::pair<int, int>, hash_pair_functor> contact_pairs_set;

    Point* m1Points = mesh1.mPoints;
    Point* m2Points = mesh2.mPoints;

    // So much indirection has to hurt the cache
    ChVector<int> i1, i2;
#pragma omp parallel for private(i1, i2) schedule(dynamic)
    // Go through bins, then try and only check triangles inside same bin
    for (int b1 = 0; b1 < NBINS; b1++) {
        for (int b2 = 0; b2 < NBINS; b2++) {
            // If the bins overlap, check collision
            if (AABB_overlap(mesh1.bins[b1], mesh2.bins[b2])) {
                // Compute collision
                const int s1 = mesh1.binIndices[b1].size();
                const int s2 = mesh2.binIndices[b2].size();
                for (int f1 = 0; f1 < s1; f1++) {
                    i1 = mesh1.binIndices[b1][f1];
                    for (int f2 = 0; f2 < s2; f2++) {
                        i2 = mesh2.binIndices[b2][f2];
                        // check for collision and add it to this thread's collisions
                        if (NoDivTriTriIsect(m2Points[i2.x()], m2Points[i2.y()], m2Points[i2.z()], m1Points[i1.x()],
                                             m1Points[i1.y()], m1Points[i1.z()]) == 1) {
                            // add to ++;
                            contact_pair_bins[omp_get_thread_num()].push_back(
                                std::pair<int, int>(mesh1.reverseIndices[b1][f1], mesh2.reverseIndices[b2][f2]));
                        }
                    }
                }
            }
        }
    }

    // Add contacts from each thread
    for (int b = 0; b < nthreads; b++) {
        contact_pairs_set.insert(contact_pair_bins[b].begin(), contact_pair_bins[b].end());
    }
    // Store this even though it might not be needed any more
    contact_pairs_vector = std::vector<std::pair<int, int>>(contact_pairs_set.begin(), contact_pairs_set.end());
    // Store old rot for history
    mesh1.oldRot = mesh1.mFrame.GetRot();
    // Clean up memory
    delete[] contact_pair_bins;

    return contact_pairs_vector;
}

std::vector<std::pair<int, int>> mesh_t_compute_contacts(mesh_t& mesh1, mesh_t& mesh2, bool use_binning = true) {
    copyVerticesToPoints(mesh1);
    copyVerticesToPoints(mesh2);
    bool broad = mesh_t_broadphase(mesh1, mesh2);
    // Exit early if no broadphase detected
    if (USE_BROADPHASE && !broad) {
        // std::cout << "exiting early\n";
        return std::vector<std::pair<int, int>>();
    }
    if (use_binning) {
        // If we're using binning, don't rebin if we haven't rotated since bins are the same
        if (USE_HISTORY && mesh1.mFrame.GetRot() == mesh1.oldRot) {
            // std::cout << "no need to redo binning!\n";
        } else {
            mesh_t_bin_vertices(mesh1);
            mesh_t_bin_vertices(mesh2);
        }
        return mesh_t_narrowphase_binning(mesh1, mesh2);
    } else {
        return mesh_t_narrowphase(mesh1, mesh2);
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the my_system.
    ChSystemNSC my_system;
    // my_system.SetSolverType(ChSolver::Type::SOR);
    // my_system.SetMaxItersSolverSpeed(20);
    // my_system.SetMaxItersSolverStab(5);
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.001);
    my_system.Set_G_acc({0, 0, 0});

    // Create the Irrlicht application.
    ChIrrApp application(&my_system, L"Number of collisions", irr::core::dimension2d<irr::u32>(800, 600), false);
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, 10, 10));

    ChTriangleMeshConnected bunnymesh;
    bunnymesh.LoadWavefrontMesh(GetChronoDataFile("bunny.obj"), true, true);
    // Shift down and scale by 40 so that bunnies are reasonable size and COM is decent
    bunnymesh.Transform({0, -3, 0}, ChMatrix33<>(50.0));

    ChTriangleMeshConnected bunnymesh2;
    bunnymesh2.LoadWavefrontMesh(GetChronoDataFile("bunny.obj"), true, true);
    bunnymesh2.Transform({0, -3, 0}, ChMatrix33<>(50.0));

    auto bunny1 = std::make_shared<ChBody>();
    bunny1->SetPos(ChVector<>(-5, 0, 0));
    bunny1->SetRot(Q_from_AngY(CH_C_PI / 6));
    bunny1->SetPos_dt(ChVector<>(2, 0, 0));
    bunny1->SetBodyFixed(false);
    my_system.AddBody(bunny1);

    bunny1->GetCollisionModel()->ClearModel();
    bunny1->GetCollisionModel()->AddTriangleMesh(bunnymesh, false, false, VNULL, ChMatrix33<>(1), 0.001);
    bunny1->GetCollisionModel()->BuildModel();
    bunny1->SetCollide(true);

    auto bunnyasset = std::make_shared<ChTriangleMeshShape>();
    bunnyasset->SetMesh(bunnymesh);
    bunny1->AddAsset(bunnyasset);

    auto bunny2 = std::make_shared<ChBody>();
    bunny2->SetPos(ChVector<>(5, 0, 0));
    // Rotate at strange angle to remove symmetry
    bunny2->SetRot(Q_from_AngY(2 * CH_C_PI / 3));
    bunny2->SetPos_dt(ChVector<>(-2, 0, 0));

    bunny2->SetBodyFixed(false);
    my_system.AddBody(bunny2);

    bunny2->GetCollisionModel()->ClearModel();
    bunny2->GetCollisionModel()->AddTriangleMesh(bunnymesh2, false, false, VNULL, ChMatrix33<>(1), 0.001);
    bunny2->GetCollisionModel()->BuildModel();
    bunny2->SetCollide(true);

    auto bunnyasset2 = std::make_shared<ChTriangleMeshShape>();
    bunnyasset2->SetMesh(bunnymesh2);
    bunny2->AddAsset(bunnyasset2);

    // In this example the two meshes are the same, the offsets are just different
    mesh_t mesh1 = {bunnymesh.m_vertices, bunnymesh.m_normals, bunnymesh.m_face_v_indices, *bunny1};
    mesh1.mPoints = new Point[mesh1.vertices.size()];
    mesh_t mesh2 = {bunnymesh2.m_vertices, bunnymesh2.m_normals, bunnymesh2.m_face_v_indices, *bunny2};
    mesh2.mPoints = new Point[mesh2.vertices.size()];

    // Complete visualization asset construction.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Create the contact manager.
    ContactManager manager(&my_system);

    double timestep = .005;
    // Simulation loop.
    application.SetStepManage(true);
    application.SetTimestep(timestep);

    stopwatch<std::milli, double> sw;

    while (application.GetDevice()->run()) {
        // Check with no binning
        sw.start();
        auto my_contacts = mesh_t_compute_contacts(mesh1, mesh2, false);
        int ncontacts = my_contacts.size();
        sw.stop();
        double my_time = sw.count();

        // Check with binning
        sw.start();
        auto fast_contacts = mesh_t_compute_contacts(mesh1, mesh2, true);
        int ncontacts_binning = fast_contacts.size();
        sw.stop();
        double my_time1 = sw.count();

        // this checks for binning bugs, should never print
        if (ncontacts != ncontacts_binning) {
            std::cerr << "different numbers: " << ncontacts << ", " << ncontacts_binning << std::endl;
            // exit(1);
        }
        application.BeginScene();

        // Render scene.
        application.DrawAll();

        // Advance dynamics.
        application.DoStep();

        // Process current collisions and report number of contacts on a few bodies.
        manager.Process();

        auto narrow = my_system.GetTimerCollisionNarrow();
        auto broad = my_system.GetTimerCollisionBroad();
        auto chrono_time = narrow + broad;
        std::cout << "TIMESTEP ---------------------------------------------------- \n";
        std::cout << "Time: " << my_system.GetChTime() << std::endl;
        std::cout << "ratio without broadphase: " << my_time / (chrono_time * 1000) << std::endl;
        std::cout << "ratio with broadphase: " << my_time1 / (chrono_time * 1000) << std::endl;
        if (manager.GetNcontacts(bunny1) != 0 || ncontacts != 0) {
            std::cout << "chrono time is " << chrono_time << " s !" << std::endl;
            std::cout << "chrono narrow is " << narrow << " broad is " << broad << std::endl;
            std::cout << "without binning, took " << my_time / 1000 << " s !" << std::endl;
            std::cout << "with binning took " << my_time1 / 1000 << " s!" << std::endl;
            printf("ncontacts is %d\n", ncontacts);
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
