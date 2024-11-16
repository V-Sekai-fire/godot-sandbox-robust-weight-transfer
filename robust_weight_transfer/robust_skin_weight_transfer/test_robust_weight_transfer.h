#ifndef ROBUST_WEIGHT_TRANSFER_HPP
#define ROBUST_WEIGHT_TRANSFER_HPP

#include <api.hpp>
#include <Eigen/Dense>

#include <vector>
#include <tuple>

Eigen::MatrixXd find_closest_point_on_surface(const Eigen::MatrixXd& p_test_points, const Eigen::MatrixXd& p_vertices, const Eigen::MatrixXi& p_triangles);

Eigen::MatrixXd interpolate_attribute_from_bary(const Eigen::MatrixXd& p_vertex_attributes, const Eigen::MatrixXd& p_barycentric_coordinates, const Eigen::VectorXi& p_primitive_indices, const Eigen::MatrixXi& p_mesh_triangles);

Eigen::VectorXd normalize_vector(const Eigen::VectorXd& p_vector);

std::tuple<Eigen::VectorXi, Eigen::MatrixXd> find_matches_closest_surface(const Eigen::MatrixXd& p_source_vertices, const Eigen::MatrixXi& p_source_triangles, const Eigen::MatrixXd& p_source_normals, const Eigen::MatrixXd& target_vertices, const Eigen::MatrixXi& p_target_triangles, const Eigen::MatrixXd& p_target_normals, const Eigen::MatrixXd& p_source_weights, double p_distance_threshold_squared, double p_angle_threshold_degrees);

bool is_valid_array(const Eigen::MatrixXd& p_matrix);

std::tuple<Eigen::MatrixXd, bool> inpaint(const Eigen::MatrixXd& p_V2, const Eigen::MatrixXi& p_F2, const Eigen::MatrixXd& p_W2, const Eigen::VectorXi& p_Matched);

std::tuple<Eigen::MatrixXd, Eigen::VectorXi> smooth(const Eigen::MatrixXd& p_target_vertices, const Eigen::MatrixXi& p_target_faces, const Eigen::MatrixXd& p_skinning_weights, const Eigen::VectorXi& p_matched, double p_distance_threshold, int p_num_smooth_iter_steps, double p_smooth_alpha);

bool test_find_closest_point_on_surface();

bool test_interpolate_attribute_from_bary();

bool test_normalize_vector();

bool test_find_matches_closest_surface();

bool test_is_valid_array();

bool test_inpaint();

bool test_smooth();

extern "C" Variant run_tests();

#endif // ROBUST_WEIGHT_TRANSFER_HPP
