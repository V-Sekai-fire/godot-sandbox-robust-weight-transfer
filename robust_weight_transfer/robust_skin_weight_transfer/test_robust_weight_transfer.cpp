
#define DOCTEST_CONFIG_IMPLEMENT
#include "thirdparty/doctest.h"

#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "robust_weight_transfer.h"

TEST_CASE("Test find_closest_point_on_surface") {
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
    REQUIRE(closest_points.isApprox(expected_points, 1e-6));
}

TEST_CASE("Test interpolate_attribute_from_bary") {
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
    REQUIRE(result.isApprox(expected_output, 1e-6));
}

TEST_CASE("Test normalize_vector") {
    Eigen::VectorXd vector(3);
    vector << 3, 4, 0;
    Eigen::VectorXd expected(3);
    expected << 0.6, 0.8, 0;

    Eigen::VectorXd normalized = normalize_vector(vector);
    REQUIRE(normalized.isApprox(expected, 1e-6));
}

TEST_CASE("Test find_matches_closest_surface") {
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
    REQUIRE(matched.isApprox(expected_matched));
    REQUIRE(target_weights.isApprox(expected_weights, 1e-6));
}

TEST_CASE("Test is_valid_array") {
    Eigen::MatrixXd valid_matrix(2, 2);
    valid_matrix << 1, 2,
                    3, 4;
    Eigen::MatrixXd invalid_matrix(2, 2);
    invalid_matrix << std::numeric_limits<double>::quiet_NaN(), 2,
                      std::numeric_limits<double>::infinity(), 4;

    REQUIRE(is_valid_array(valid_matrix) == true);
    REQUIRE(is_valid_array(invalid_matrix) == false);
}

TEST_CASE("Test inpaint") {
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
    REQUIRE(success == true);
    REQUIRE(W_inpainted.isApprox(expected_W_inpainted, 1e-6));
}

TEST_CASE("Test smooth") {
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
                                 0.10666667, 0.89333333,
                                 0.48044444, 0.51955556,
                                 0.25871111, 0.74128889,
                                 0.1, 0.9;
    Eigen::VectorXi expected_vertices_ids_to_smooth(5);
    expected_vertices_ids_to_smooth << 1, 1, 1, 1, 1;

    auto [smoothed_weights, vertices_ids_to_smooth] = smooth(target_vertices, target_faces, skinning_weights, matched, distance_threshold, 1, 0.2);
    REQUIRE(smoothed_weights.isApprox(expected_smoothed_weights, 1e-6));
    REQUIRE(vertices_ids_to_smooth.isApprox(expected_vertices_ids_to_smooth));
}