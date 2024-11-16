#ifndef ROBUST_WEIGHT_TRANSFER_HPP
#define ROBUST_WEIGHT_TRANSFER_HPP

#include <Eigen/Dense>

#include <vector>
#include <tuple>

Eigen::MatrixXd find_closest_point_on_surface(const Eigen::MatrixXd& test_points, const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles);

Eigen::MatrixXd interpolate_attribute_from_bary(const Eigen::MatrixXd& vertex_attributes, const Eigen::MatrixXd& barycentric_coordinates, const Eigen::VectorXi& primitive_indices, const Eigen::MatrixXi& mesh_triangles);

Eigen::VectorXd normalize_vector(const Eigen::VectorXd& vector);

std::tuple<Eigen::VectorXi, Eigen::MatrixXd> find_matches_closest_surface(const Eigen::MatrixXd& source_vertices, const Eigen::MatrixXi& source_triangles, const Eigen::MatrixXd& source_normals, const Eigen::MatrixXd& target_vertices, const Eigen::MatrixXi& target_triangles, const Eigen::MatrixXd& target_normals, const Eigen::MatrixXd& source_weights, double distance_threshold_squared, double angle_threshold_degrees);

bool is_valid_array(const Eigen::MatrixXd& matrix);

std::tuple<Eigen::MatrixXd, bool> inpaint(const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& W2, const Eigen::VectorXi& Matched);

std::tuple<Eigen::MatrixXd, Eigen::VectorXi> smooth(const Eigen::MatrixXd& target_vertices, const Eigen::MatrixXi& target_faces, const Eigen::MatrixXd& skinning_weights, const Eigen::VectorXi& matched, double distance_threshold, int num_smooth_iter_steps, double smooth_alpha);

bool test_find_closest_point_on_surface();

bool test_interpolate_attribute_from_bary();

bool test_normalize_vector();

bool test_find_matches_closest_surface();

bool test_is_valid_array();

bool test_inpaint();

bool test_smooth();

extern "C" Variant run_tests();

#endif // ROBUST_WEIGHT_TRANSFER_HPP
