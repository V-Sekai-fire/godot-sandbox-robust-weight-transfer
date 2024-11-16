#include "test_robust_weight_transfer.h"

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <set>

#include <api.hpp>

#include <igl/point_mesh_squared_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/adjacency_list.h>
#include <igl/min_quad_with_fixed.h>

Eigen::MatrixXd find_closest_point_on_surface(const Eigen::MatrixXd& test_points, const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles) {
    Eigen::VectorXd smallest_squared_distances;
    Eigen::VectorXi primitive_indices;
    Eigen::MatrixXd closest_points;

    igl::point_mesh_squared_distance(test_points, vertices, triangles, smallest_squared_distances, primitive_indices, closest_points);

    Eigen::MatrixXd barycentric_coordinates(test_points.rows(), 3);
    for (int i = 0; i < test_points.rows(); ++i) {
        Eigen::RowVector3d v1 = vertices.row(triangles(primitive_indices(i), 0));
        Eigen::RowVector3d v2 = vertices.row(triangles(primitive_indices(i), 1));
        Eigen::RowVector3d v3 = vertices.row(triangles(primitive_indices(i), 2));
        Eigen::RowVector3d point = closest_points.row(i);
        Eigen::RowVector3d bary;
        igl::barycentric_coordinates(point, v1, v2, v3, bary);
        barycentric_coordinates.row(i) = bary;
    }

    return closest_points;
}

Eigen::MatrixXd interpolate_attribute_from_bary(const Eigen::MatrixXd& vertex_attributes, const Eigen::MatrixXd& barycentric_coordinates, const Eigen::VectorXi& primitive_indices, const Eigen::MatrixXi& mesh_triangles) {
    Eigen::MatrixXd interpolated_attributes(barycentric_coordinates.rows(), vertex_attributes.cols());

    for (int i = 0; i < barycentric_coordinates.rows(); ++i) {
        int tri_idx = primitive_indices(i);
        Eigen::RowVector3i tri = mesh_triangles.row(tri_idx);

        Eigen::RowVectorXd attr1 = vertex_attributes.row(tri(0));
        Eigen::RowVectorXd attr2 = vertex_attributes.row(tri(1));
        Eigen::RowVectorXd attr3 = vertex_attributes.row(tri(2));

        double b1 = barycentric_coordinates(i, 0);
        double b2 = barycentric_coordinates(i, 1);
        double b3 = barycentric_coordinates(i, 2);

        interpolated_attributes.row(i) = b1 * attr1 + b2 * attr2 + b3 * attr3;
    }

    return interpolated_attributes;
}

Eigen::VectorXd normalize_vector(const Eigen::VectorXd& vector) {
    return vector.normalized();
}

std::tuple<Eigen::VectorXi, Eigen::MatrixXd> find_matches_closest_surface(const Eigen::MatrixXd& source_vertices, const Eigen::MatrixXi& source_triangles, const Eigen::MatrixXd& source_normals, const Eigen::MatrixXd& target_vertices, const Eigen::MatrixXi& target_triangles, const Eigen::MatrixXd& target_normals, const Eigen::MatrixXd& source_weights, double distance_threshold_squared, double angle_threshold_degrees) {
    Eigen::VectorXd squared_distance;
    Eigen::VectorXi closest_indices;
    Eigen::MatrixXd closest_points;

    igl::point_mesh_squared_distance(target_vertices, source_vertices, source_triangles, squared_distance, closest_indices, closest_points);

    Eigen::MatrixXd barycentric_coordinates(target_vertices.rows(), 3);
    for (int i = 0; i < target_vertices.rows(); ++i) {
        Eigen::RowVector3d v1 = source_vertices.row(source_triangles(closest_indices(i), 0));
        Eigen::RowVector3d v2 = source_vertices.row(source_triangles(closest_indices(i), 1));
        Eigen::RowVector3d v3 = source_vertices.row(source_triangles(closest_indices(i), 2));
        Eigen::RowVector3d point = closest_points.row(i);
        Eigen::RowVector3d bary;
        igl::barycentric_coordinates(point, v1, v2, v3, bary);
        barycentric_coordinates.row(i) = bary;
    }

    Eigen::MatrixXd target_weights = interpolate_attribute_from_bary(source_weights, barycentric_coordinates, closest_indices, source_triangles);
    Eigen::MatrixXd source_normals_matched_interpolated = interpolate_attribute_from_bary(source_normals, barycentric_coordinates, closest_indices, source_triangles);

    Eigen::VectorXi matched(target_vertices.rows());
    matched.setZero();

    for (int i = 0; i < target_vertices.rows(); ++i) {
        Eigen::Vector3d normalized_source_normal = normalize_vector(source_normals_matched_interpolated.row(i));
        Eigen::Vector3d normalized_target_normal = normalize_vector(target_normals.row(i));

        double radian_angle = std::acos(std::abs(normalized_source_normal.dot(normalized_target_normal)));
        double degree_angle = radian_angle * 180.0 / M_PI;

        if (squared_distance(i) <= distance_threshold_squared && degree_angle <= angle_threshold_degrees) {
            matched(i) = 1;
        }
    }

    return {matched, target_weights};
}

bool is_valid_array(const Eigen::MatrixXd& matrix) {
    return matrix.allFinite();
}

/**
 * Inpaint weights for all the vertices on the target mesh for which we didn't
 * find a good match on the source (i.e. Matched[i] == False).
 *
 * Args:
 *     V2: #V2 by 3 target mesh vertices
 *     F2: #F2 by 3 target mesh triangles indices
 *     W2: #V2 by num_bones, where W2[i,:] are skinning weights copied directly from source using closest point method
 *     Matched: #V2 array of bools, where Matched[i] is True if we found a good match for vertex i on the source mesh
 *
 * Returns:
 *     W_inpainted: #V2 by num_bones, final skinning weights where we inpainted weights for all vertices i where Matched[i] == False
 *     success: true if inpainting succeeded, false otherwise
 */
std::tuple<Eigen::MatrixXd, bool> inpaint(const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& W2, const Eigen::VectorXi& Matched) {
    if (V2.cols() != 3) {
        return {Eigen::MatrixXd(), false};
    }
    if (F2.cols() != 3 || F2.maxCoeff() >= V2.rows()) {
        return {Eigen::MatrixXd(), false};
    }
    if (W2.rows() != V2.rows()) {
        return {Eigen::MatrixXd(), false};
    }
    if (Matched.size() != V2.rows()) {
        return {Eigen::MatrixXd(), false};
    }

    Eigen::SparseMatrix<double> L, M;
    igl::cotmatrix(V2, F2, L);
    igl::massmatrix(V2, F2, igl::MASSMATRIX_TYPE_VORONOI, M);
    L = -L;  // Flip the sign of the Laplacian
    Eigen::SparseMatrix<double> Minv = M.cwiseInverse();
    Eigen::SparseMatrix<double> Q = L.transpose() * Minv * L;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(L.rows(), W2.cols());
    std::vector<int> b;
    Eigen::MatrixXd bc;

    for (int i = 0; i < Matched.size(); ++i) {
        if (Matched(i)) {
            b.push_back(i);
        }
    }

    bc.resize(b.size(), W2.cols());
    for (int i = 0; i < b.size(); ++i) {
        bc.row(i) = W2.row(b[i]);
    }

    Eigen::MatrixXd W_inpainted = Eigen::MatrixXd::Zero(L.rows(), W2.cols());
    Eigen::VectorXi b_vec = Eigen::Map<Eigen::VectorXi>(b.data(), b.size());
    Eigen::SparseMatrix<double> Aeq(0, 0);
    Eigen::MatrixXd Beq(0, W2.cols());
    igl::min_quad_with_fixed_data<double> data;
    igl::min_quad_with_fixed_precompute(Q, b_vec, Aeq, true, data);
    Eigen::MatrixXd Z;
    bool success = igl::min_quad_with_fixed_solve(data, B, bc, Beq, W_inpainted, Z);
    return {W_inpainted, success};
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXi> smooth(const Eigen::MatrixXd& target_vertices, const Eigen::MatrixXi& target_faces, const Eigen::MatrixXd& skinning_weights, const Eigen::VectorXi& matched, double distance_threshold, int num_smooth_iter_steps, double smooth_alpha) {
    Eigen::VectorXi not_matched(matched.size());
    for (int i = 0; i < matched.size(); ++i) {
        not_matched(i) = (matched(i) == 0) ? 1 : 0;
    }
    Eigen::VectorXi vertices_ids_to_smooth = Eigen::VectorXi::Zero(target_vertices.rows());

    std::vector<std::vector<int>> adjacency_list;
    igl::adjacency_list(target_faces, adjacency_list);

    auto get_points_within_distance = [&](int vertex_id) {
        std::vector<int> result;
        if (vertex_id >= adjacency_list.size() || vertex_id < 0) {
            return result;
        }

        std::queue<int> queue;
        std::set<int> visited;
        queue.push(vertex_id);
        visited.insert(vertex_id);

        while (!queue.empty()) {
            int current_vertex = queue.front();
            queue.pop();

            if (current_vertex >= adjacency_list.size() || current_vertex < 0) {
                continue;
            }

            for (int neighbor : adjacency_list[current_vertex]) {
                if (visited.find(neighbor) == visited.end() && (target_vertices.row(vertex_id) - target_vertices.row(neighbor)).norm() < distance_threshold) {
                    visited.insert(neighbor);
                    queue.push(neighbor);
                }
            }
        }

        result.assign(visited.begin(), visited.end());
        return result;
    };

    for (int i = 0; i < target_vertices.rows(); ++i) {
        if (not_matched(i)) {
            std::vector<int> affected_vertices = get_points_within_distance(i);
            for (int vertex : affected_vertices) {
                vertices_ids_to_smooth(vertex) = 1;
            }
        }
    }

    Eigen::MatrixXd smoothed_weights = skinning_weights;
    for (int step_idx = 0; step_idx < num_smooth_iter_steps; ++step_idx) {
        Eigen::MatrixXd new_weights = smoothed_weights;
        for (int i = 0; i < target_vertices.rows(); ++i) {
            if (vertices_ids_to_smooth(i)) {
                if (i >= adjacency_list.size()) {
                    continue;
                }
                const std::vector<int>& neighbors = adjacency_list[i];
                if (neighbors.empty()) {
                    continue;
                }

                Eigen::RowVectorXd weight = smoothed_weights.row(i);
                Eigen::RowVectorXd new_weight = (1 - smooth_alpha) * weight;

                for (int neighbor : neighbors) {
                    new_weight += (smoothed_weights.row(neighbor) / neighbors.size()) * smooth_alpha;
                }

                new_weights.row(i) = new_weight;
            }
        }
        smoothed_weights = new_weights;
    }

    return {smoothed_weights, vertices_ids_to_smooth};
}

bool test_find_closest_point_on_surface() {
    Eigen::MatrixXd vertices(3, 3);
    vertices << -1, -1, -1,
                1, -1, -1,
                1, 1, -1;
    Eigen::MatrixXi triangles(1, 3);
    triangles << 0, 1, 2;
    Eigen::MatrixXd test_points(3, 3);
    test_points << 0, 0, 0,
                   2, 2, 2,
                   0, 0, -2;
    Eigen::MatrixXd expected_points(3, 3);
    expected_points << 0, 0, -1,
                       1, 1, -1,
                       0, 0, -1;

    Eigen::MatrixXd closest_points = find_closest_point_on_surface(test_points, vertices, triangles);
    std::cout << "Closest Points:\n" << closest_points << std::endl;
    std::cout << "Expected Points:\n" << expected_points << std::endl;
    if (!closest_points.isApprox(expected_points, 1e-6)) {
        return false;
    }
    return true;
}

bool test_interpolate_attribute_from_bary() {
    Eigen::MatrixXd vertex_attributes(5, 2);
    vertex_attributes << 1, 2,
                         3, 4,
                         5, 6,
                         7, 8,
                         9, 10;
    Eigen::MatrixXd barycentric_coordinates(2, 3);
    barycentric_coordinates << 0.2, 0.5, 0.3,
                               0.6, 0.3, 0.1;
    Eigen::VectorXi primitive_indices(2);
    primitive_indices << 0, 1;
    Eigen::MatrixXi mesh_triangles(2, 3);
    mesh_triangles << 0, 1, 2,
                      2, 3, 4;
    Eigen::MatrixXd expected_output(2, 2);
    expected_output << 1 * 0.2 + 3 * 0.5 + 5 * 0.3, 2 * 0.2 + 4 * 0.5 + 6 * 0.3,
                       5 * 0.6 + 7 * 0.3 + 9 * 0.1, 6 * 0.6 + 8 * 0.3 + 10 * 0.1;

    Eigen::MatrixXd result = interpolate_attribute_from_bary(vertex_attributes, barycentric_coordinates, primitive_indices, mesh_triangles);
    std::cout << "Interpolated Attributes:\n" << result << std::endl;
    std::cout << "Expected Output:\n" << expected_output << std::endl;
    if (!result.isApprox(expected_output, 1e-6)) {
        return false;
    }
    return true;
}

bool test_normalize_vector() {
    Eigen::VectorXd vector(3);
    vector << 3, 4, 0;
    Eigen::VectorXd expected(3);
    expected << 0.6, 0.8, 0;

    Eigen::VectorXd normalized = normalize_vector(vector);
    std::cout << "Normalized Vector:\n" << normalized << std::endl;
    std::cout << "Expected Vector:\n" << expected << std::endl;
    if (!normalized.isApprox(expected, 1e-6)) {
        return false;
    }
    return true;
}

bool test_find_matches_closest_surface() {
    Eigen::MatrixXd source_vertices(3, 3);
    source_vertices << 0, 0, 0,
                       1, 0, 0,
                       0, 1, 0;
    Eigen::MatrixXi source_triangles(1, 3);
    source_triangles << 0, 1, 2;
    Eigen::MatrixXd source_normals(3, 3);
    source_normals << 0, 0, 1,
                      0, 0, 1,
                      0, 0, 1;
    Eigen::MatrixXd source_weights(3, 2);
    source_weights << 1, 0,
                      0, 1,
                      0.5, 0.5;

    Eigen::MatrixXd target_vertices(2, 3);
    target_vertices << 0.1, 0.1, 0,
                       2, 2, 2;
    Eigen::MatrixXi target_triangles(1, 3);
    target_triangles << 0, 1;
    Eigen::MatrixXd target_normals(2, 3);
    target_normals << 0, 0, 1,
                      1, 0, 0;

    double distance_threshold_squared = 0.5;
    double angle_threshold_degrees = 10;

    Eigen::VectorXi expected_matched(2);
    expected_matched << 1, 0;
    Eigen::MatrixXd expected_weights(2, 2);
    expected_weights << 0.85, 0.15,
                        0.25, 0.75;

    auto [matched, target_weights] = find_matches_closest_surface(source_vertices, source_triangles, source_normals, target_vertices, target_triangles, target_normals, source_weights, distance_threshold_squared, angle_threshold_degrees);
    std::cout << "Matched:\n" << matched << std::endl;
    std::cout << "Expected Matched:\n" << expected_matched << std::endl;
    std::cout << "Target Weights:\n" << target_weights << std::endl;
    std::cout << "Expected Weights:\n" << expected_weights << std::endl;
    if (!matched.isApprox(expected_matched) || !target_weights.isApprox(expected_weights, 1e-6)) {
        return false;
    }
    return true;
}

bool test_is_valid_array() {
    Eigen::MatrixXd valid_matrix(2, 2);
    valid_matrix << 1, 2,
                    3, 4;
    Eigen::MatrixXd invalid_matrix(2, 2);
    invalid_matrix << std::numeric_limits<double>::quiet_NaN(), 2,
                      std::numeric_limits<double>::infinity(), 4;

    std::cout << "Valid Matrix:\n" << valid_matrix << std::endl;
    std::cout << "Invalid Matrix:\n" << invalid_matrix << std::endl;
    if (is_valid_array(valid_matrix) != true || is_valid_array(invalid_matrix) != false) {
        return false;
    }
    return true;
}

bool test_inpaint() {
    Eigen::MatrixXd V2(4, 3);
    V2 << 0, 0, 0,
          1, 0, 0,
          0, 1, 0,
          1, 1, 0;
    Eigen::MatrixXi F2(2, 3);
    F2 << 0, 1, 2,
          1, 2, 3;
    Eigen::MatrixXd W2(4, 2);
    W2 << 1, 0,
          0, 1,
          0.5, 0.5,
          0, 0;
    Eigen::VectorXi Matched(4);
    Matched << 1, 1, 1, 0;
    Eigen::MatrixXd expected_W_inpainted(4, 2);
    expected_W_inpainted << 1.0, 0.0,
                            0.0, 1.0,
                            0.5, 0.5,
                            0.117647, 0.882353;

    auto [W_inpainted, success] = inpaint(V2, F2, W2, Matched);
    std::cout << "Inpainted Weights:\n" << W_inpainted << std::endl;
    std::cout << "Expected Inpainted Weights:\n" << expected_W_inpainted << std::endl;
    if (success != true || !W_inpainted.isApprox(expected_W_inpainted, 1e-6)) {
        return false;
    }
    return true;
}

bool test_smooth() {
    Eigen::MatrixXd target_vertices(5, 3);
    target_vertices << 0, 0, 0,
                       1, 0, 0,
                       0, 1, 0,
                       1, 1, 0,
                       2, 1, 0;
    Eigen::MatrixXi target_faces(2, 3);
    target_faces << 0, 1, 2,
                    1, 2, 3;
    Eigen::MatrixXd skinning_weights(5, 2);
    skinning_weights << 1, 0,
                        0, 1,
                        0.5, 0.5,
                        0.25, 0.75,
                        0.1, 0.9;
    Eigen::VectorXi matched(5);
    matched << 1, 1, 1, 0, 0;
    double distance_threshold = 1.5;

    Eigen::MatrixXd expected_smoothed_weights(5, 2);
    expected_smoothed_weights << 0.85, 0.15,
                                 0.116667, 0.883333,
                                 0.483333, 0.516667,
                                 0.25, 0.75,
                                 0.1, 0.9;
    Eigen::VectorXi expected_vertices_ids_to_smooth(5);
    expected_vertices_ids_to_smooth << 1, 1, 1, 1, 0;

    auto [smoothed_weights, vertices_ids_to_smooth] = smooth(target_vertices, target_faces, skinning_weights, matched, distance_threshold, 1, 0.2);

    std::cout << "Smoothed Weights:\n" << smoothed_weights << std::endl;
    std::cout << "Expected Smoothed Weights:\n" << expected_smoothed_weights << std::endl;
    std::cout << "Vertices IDs to Smooth:\n" << vertices_ids_to_smooth << std::endl;
    std::cout << "Expected Vertices IDs to Smooth:\n" << expected_vertices_ids_to_smooth << std::endl;

    if (!smoothed_weights.isApprox(expected_smoothed_weights, 1e-6) || !vertices_ids_to_smooth.isApprox(expected_vertices_ids_to_smooth)) {
        return false;
    }
    return true;
}

extern "C" Variant run_tests() {
    bool all_tests_passed = true;
    if (!test_find_closest_point_on_surface()) {
        std::cerr << "test_find_closest_point_on_surface failed" << std::endl;
        all_tests_passed = false;
    }
    if (!test_interpolate_attribute_from_bary()) {
        std::cerr << "test_interpolate_attribute_from_bary failed" << std::endl;
        all_tests_passed = false;
    }
    if (!test_normalize_vector()) {
        std::cerr << "test_normalize_vector failed" << std::endl;
        all_tests_passed = false;
    }
    if (!test_find_matches_closest_surface()) {
        std::cerr << "test_find_matches_closest_surface failed" << std::endl;
        all_tests_passed = false;
    }
    if (!test_is_valid_array()) {
        std::cerr << "test_is_valid_array failed" << std::endl;
        all_tests_passed = false;
    }
    if (!test_smooth()) {
        std::cerr << "test_smooth failed" << std::endl;
        all_tests_passed = false;
    }
    if (!test_inpaint()) {
        std::cerr << "test_inpaint failed" << std::endl;
        all_tests_passed = false;
    }
    if (all_tests_passed) {
        std::cout << "All tests passed!" << std::endl;
        return Variant(0);
    } else {
        std::cerr << "Some tests failed." << std::endl;
        return Variant(1);
    }
}
