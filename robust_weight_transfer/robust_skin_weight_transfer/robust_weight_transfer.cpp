#include <api.hpp>
#include <iostream>
#include <iomanip>

#include <Eigen/Dense>
#include <igl/point_mesh_squared_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/bounding_box_diagonal.h>
#include <fstream>
#include <igl/AABB.h>

extern "C" void funlockfile(FILE *) {}
void flockfile(FILE *) {}

void barycentric_coordinates_tri(const Eigen::MatrixXd &P, const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &C, Eigen::MatrixXd &BC) {
    BC.resize(P.rows(), 3);
    BC.setZero();
}

extern "C" Variant find_closest_point_on_surface(PackedArray<double> p_source, PackedArray<double> p_mesh_vertices, PackedArray<int32_t> p_mesh_triangles) {
    using namespace Eigen;
    using namespace igl;

    std::vector<double> source_data = p_source.fetch();
    std::vector<double> mesh_vertices_data = p_mesh_vertices.fetch();
    std::vector<int32_t> p_mesh_triangles_data = p_mesh_triangles.fetch();

    Map<MatrixXd> points(source_data.data(), source_data.size() / 3, 3);
    Map<MatrixXd> vertices(mesh_vertices_data.data(), mesh_vertices_data.size() / 3, 3);
    Map<MatrixXi> triangles(p_mesh_triangles_data.data(), p_mesh_triangles_data.size() / 3, 3);
    VectorXd squared_distances;
    VectorXi primitive_indices;
    MatrixXd closest_points;
    point_mesh_squared_distance(points, vertices, triangles, squared_distances, primitive_indices, closest_points);

    MatrixXd vertex_1(primitive_indices.size(), 3);
    MatrixXd vertex_2(primitive_indices.size(), 3);
    MatrixXd vertex_3(primitive_indices.size(), 3);
    for (int i = 0; i < primitive_indices.size(); ++i) {
        vertex_1.row(i) = vertices.row(triangles(primitive_indices(i), 0));
        vertex_2.row(i) = vertices.row(triangles(primitive_indices(i), 1));
        vertex_3.row(i) = vertices.row(triangles(primitive_indices(i), 2));
    }

    MatrixXd barycentric_coordinates;
    barycentric_coordinates_tri(closest_points, vertex_1, vertex_2, vertex_3, barycentric_coordinates);

    Dictionary result;
    std::vector<double> smallest_squared_distances(squared_distances.data(), squared_distances.data() + squared_distances.size());
    result["smallest_squared_distances"] = PackedArray<double>(smallest_squared_distances);

    std::vector<int> primitive_indices_vector(primitive_indices.data(), primitive_indices.data() + primitive_indices.size());
    result["primitive_indices"] = PackedArray<int>(primitive_indices_vector);

    std::vector<double> closest_points_vector(closest_points.data(), closest_points.data() + closest_points.size() * closest_points.cols());
    result["closest_points"] = PackedArray<double>(closest_points_vector);

    std::vector<double> barycentric_coordinates_vector(barycentric_coordinates.data(), barycentric_coordinates.data() + barycentric_coordinates.size() * barycentric_coordinates.cols());
    result["barycentric_coordinates"] = PackedArray<double>(barycentric_coordinates_vector);

    return Variant(result);
}

extern "C" Variant create_diamond() {
    using namespace Eigen;

    // Define vertices for a small diamond shape scaled to mm
    double scale_factor = 0.005;  // Scaling down from meters to millimeters
    MatrixXd vertices(6, 3);
    vertices << 0, 0, 1,
                1, 0, 0,
                -1, 0, 0,
                0, 1, 0,
                0, -1, 0,
                0, 0, -1;
    vertices *= scale_factor;

    // Define faces connecting the vertices
    MatrixXi faces(8, 3);
    faces << 0, 1, 3,
             0, 3, 2,
             0, 2, 4,
             0, 4, 1,
             5, 3, 1,
             5, 2, 3,
             5, 4, 2,
             5, 1, 4;

    // Convert Eigen matrices to Godot PackedArrays
    std::vector<double> vertices_vector(vertices.data(), vertices.data() + vertices.size());
    std::vector<int> faces_vector(faces.data(), faces.data() + faces.size());

    // Prepare the result as a Variant
    Dictionary result;
    result["vertices"] = PackedArray<double>(vertices_vector);
    result["faces"] = PackedArray<int>(faces_vector);

    return Variant(result);
}

extern "C" Variant transfer_skin_weights(PackedArray<double> p_source_mesh, PackedArray<double> p_target_mesh) {
    using namespace Eigen;
    using namespace igl;

    std::vector<double> source_mesh_data = p_source_mesh.fetch();
    std::vector<double> target_mesh_data = p_target_mesh.fetch();

    Map<MatrixXd> vertices_1(source_mesh_data.data(), source_mesh_data.size() / 3, 3);
    Map<MatrixXd> vertices_2(target_mesh_data.data(), target_mesh_data.size() / 3, 3);

    // Generate simple per-vertex data
    MatrixXd skin_weights(vertices_1.rows(), 2);
    skin_weights.col(0).setConstant(0.3);
    skin_weights.col(1).setConstant(0.7);

    // Closest Point Matching
    double distance_threshold = 0.05 * bounding_box_diagonal(vertices_2);
    double distance_threshold_squared = distance_threshold * distance_threshold;
    int angle_threshold_degrees = 30;

    // Find matches and interpolate skin weights
    // Implement find_matches_closest_surface and inpaint functions

    // Optional smoothing
    // Implement smooth function

    // Convert Eigen matrices to Godot PackedArrays
    std::vector<double> vertices_vector(vertices_2.data(), vertices_2.data() + vertices_2.size());
    std::vector<double> skin_weights_vector(skin_weights.data(), skin_weights.data() + skin_weights.size());

    // Prepare the result as a Variant
    Dictionary result;
    result["vertices"] = PackedArray<double>(vertices_vector);
    result["skin_weights"] = PackedArray<double>(skin_weights_vector);

    return Variant(result);
}
