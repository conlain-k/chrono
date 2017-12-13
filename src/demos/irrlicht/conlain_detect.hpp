// Uses mesh gotten from here: https://graphics.stanford.edu/~mdfisher/Data/Meshes/bunny.obj
// Uses sample code from the tinyobj loader example
// Load an obj, print some data
// Usage: ./tester data/bunny.obj works rn

#include "chrono/core/ChVector.h"

#define TINYOBJLOADER_IMPLEMENTATION
// #include "tinyobjloader/tiny_obj_loader.h"

// using tinyobj::real_t;

// A single point in 3D space
typedef double Point[3];

void printPoint(const Point& p) {
    printf("%f, %f, %f\n", p[0], p[1], p[2]);
}
void vectorToPointOffset(const ChVector& source, Point& dest, const ChVector& offset) {
    dest[0] = source.x() + offset[0];
    dest[1] = source.y() + offset[1];
    dest[2] = source.z() + offset[2];
    // printPoint(source);
    printPoint(dest);
}

// Copy a point to another, possibly with an offset
void copyPoint(const Point& source, Point& dest, const Point& offset) {
    dest[0] = source[0] + offset[0];
    dest[1] = source[1] + offset[1];
    dest[2] = source[2] + offset[2];
    printPoint(source);
    printPoint(dest);
}

/*
int main(int argc, char const* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: ./solver <filename1> <filename2>" << std::endl;
        exit(1);
    }
    string filename(argv[1]);
    string filename2(argv[2]);
    tinyobj::attrib_t attrib, attrib2;
    std::vector<tinyobj::shape_t> shapes, shapes2;
    std::vector<tinyobj::material_t> materials, materials2;
    std::string err;
    bool triangulate = true;
    bool ret;
    ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str(), NULL, triangulate);

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }
    if (!ret) {
        std::cerr << "couldn't load first bunny!" << std::endl;
    }
    ret = tinyobj::LoadObj(&attrib2, &shapes2, &materials2, &err, filename2.c_str(), "data/", triangulate);
    if (!err.empty()) {
        std::cerr << err << std::endl;
    }
    if (!ret) {
        std::cerr << "couldn't load first bunny!" << std::endl;
    }
    // for ()
    // int tri_tri_intersect(float V0[3],float V1[3],float V2[3],
    //                       float U0[3],float U1[3],float U2[3])
    // PrintInfo(attrib, shapes, materials);
    float V0[] = {0, 0, 0};
    float V1[] = {1, 0, 0};
    float V2[] = {0, 1, 0};
    float U0[] = {0, 0, 0};
    float U1[] = {0, 1, 0};
    float U2[] = {0, 1, 1};
    std::cout << tri_tri_intersect(V0, V1, V2, U0, U1, U2) << std::endl;
    float U3[] = {2, 3, 2};
    float U4[] = {2, 2, 2};
    float U5[] = {3, 2, 2};
    std::cout << tri_tri_intersect(V0, V1, V2, U3, U4, U5) << std::endl;
    // This is a messy c-style cast
    Point* mesh1Points = (Point*)attrib.vertices.data();
    Point* mesh2Points = (Point*)attrib2.vertices.data();
    // for (unsigned int i = 0; i < attrib.vertices.size() / 3; i++) {
    // 	printf("point is %f, %f, %f\n", mesh1Points[i][0], mesh1Points[i][1], mesh1Points[i][2]);
    // 	printf("  v[%ld] = (%f, %f, %f)\n", static_cast<long>(i),
    // 		   static_cast<const double>(attrib.vertices.data()[3 * i + 0]),
    // 		   static_cast<const double>(attrib.vertices.data()[3 * i + 1]),
    // 		   static_cast<const double>(attrib.vertices.data()[3 * i + 2]));
    // }

    // For each shape
    for (size_t i = 0; i < shapes.size(); i++) {
        // For each face
        for (size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) {
            size_t fnum = shapes[i].mesh.num_face_vertices[f];
            size_t index_offset = 0;
            // printf("face %ld\n", static_cast<long>(f));
            // For each vertex in the face
            Point* tri1 = new Point[3];
            for (int v = 0; v < 3; v++) {
                tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + v];
                copyPoint(mesh1Points[idx.vertex_index], tri1[i]);
                // tri1[i][v] = mesh1Points[idx.vertex_index][0];
                // tri1[i][v] = mesh1Points[idx.vertex_index][1];
                // tri1[i][v] = mesh1Points[idx.vertex_index][2];
                index_offset += fnum;
            }
            // printf("point is %f, %f, %f\n", tri1[i][0], tri1[i][1], tri1[i][2]);
            // printf("point is %f, %f, %f\n", mesh1Points[idx.vertex_index][0],
            // 	   mesh1Points[idx.vertex_index][1], mesh1Points[idx.vertex_index][2]);
            // std::cout << "start inner loop" << std::endl;
            for (size_t j = 0; j < shapes.size(); j++) {
                // For each face
                for (size_t face2 = 0; face2 < shapes2[j].mesh.num_face_vertices.size(); face2++) {
                    size_t index_offset2 = 0;
                    size_t fnum2 = shapes2[i].mesh.num_face_vertices[f];

                    // printf("face %ld\n", static_cast<long>(face2));
                    // For each vertex in the face
                    Point tri2[3];
                    for (int v = 0; v < 3; v++) {
                        tinyobj::index_t idx = shapes[i].mesh.indices[index_offset2 + v];
                        copyPoint(mesh2Points[idx.vertex_index], tri2[i]);
                        index_offset2 += fnum2;
                    }
                    if (tri_tri_intersect(tri1[0], tri1[1], tri1[1], tri2[0], tri2[1], tri2[2])) {
                        // std::cout << "face " << f << " on body 1 collides with face " << face2
                        // 		  << " on body 2 " << std::endl;
                        // std::cout << "body 1:" << std::endl;
                        // for (int v = 0; v < 3; v++) {
                        // 	printPoint(tri1[v]);
                        // }
                        // std::cout << "body 2:" << std::endl;
                        // for (int v = 0; v < 3; v++) {
                        // 	printPoint(tri2[v]);
                        // }
                    }
                }
            }
            delete[] tri1;
        }
    }
    printf("size of point is %lu\n", sizeof(Point));
    return 0;
}
*/