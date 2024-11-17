#include "test_robust_weight_transfer.h"

#include <Eigen/Dense>
#include <cstdint>
#include <vector>
#include <iostream>
#include <set>

#include <igl/point_mesh_squared_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/adjacency_list.h>
#include <igl/invert_diag.h>
#include <igl/slice_mask.h>
#include <igl/min_quad_with_fixed.h>

#include "generated_api.hpp"

Eigen::MatrixXd find_closest_point_on_surface(const Eigen::MatrixXd& p_test_points, const Eigen::MatrixXd& p_vertices, const Eigen::MatrixXi& p_triangles) {
    Eigen::VectorXd smallest_squared_distances;
    Eigen::VectorXi primitive_indices;
    Eigen::MatrixXd closest_points;

    igl::point_mesh_squared_distance(p_test_points, p_vertices, p_triangles, smallest_squared_distances, primitive_indices, closest_points);

    Eigen::MatrixXd barycentric_coordinates(p_test_points.rows(), 3);
    for (int i = 0; i < p_test_points.rows(); ++i) {
        Eigen::RowVector3d v1 = p_vertices.row(p_triangles(primitive_indices(i), 0));
        Eigen::RowVector3d v2 = p_vertices.row(p_triangles(primitive_indices(i), 1));
        Eigen::RowVector3d v3 = p_vertices.row(p_triangles(primitive_indices(i), 2));
        Eigen::RowVector3d point = closest_points.row(i);
        Eigen::RowVector3d bary;
        igl::barycentric_coordinates(point, v1, v2, v3, bary);
        barycentric_coordinates.row(i) = bary;
    }

    return closest_points;
}

Eigen::MatrixXd interpolate_attribute_from_bary(const Eigen::MatrixXd& p_vertex_attributes, const Eigen::MatrixXd& p_barycentric_coordinates, const Eigen::VectorXi& p_primitive_indices, const Eigen::MatrixXi& p_mesh_triangles) {
    Eigen::MatrixXd interpolated_attributes(p_barycentric_coordinates.rows(), p_vertex_attributes.cols());

    for (int i = 0; i < p_barycentric_coordinates.rows(); ++i) {
        int tri_idx = p_primitive_indices(i);
        Eigen::RowVector3i tri = p_mesh_triangles.row(tri_idx);

        Eigen::RowVectorXd attr1 = p_vertex_attributes.row(tri(0));
        Eigen::RowVectorXd attr2 = p_vertex_attributes.row(tri(1));
        Eigen::RowVectorXd attr3 = p_vertex_attributes.row(tri(2));

        double b1 = p_barycentric_coordinates(i, 0);
        double b2 = p_barycentric_coordinates(i, 1);
        double b3 = p_barycentric_coordinates(i, 2);

        interpolated_attributes.row(i) = b1 * attr1 + b2 * attr2 + b3 * attr3;
    }

    return interpolated_attributes;
}

Eigen::VectorXd normalize_vector(const Eigen::VectorXd& p_vector) {
    return p_vector.normalized();
}

std::tuple<Eigen::VectorXi, Eigen::MatrixXd> find_matches_closest_surface(const Eigen::MatrixXd& p_source_vertices, const Eigen::MatrixXi& p_source_triangles, const Eigen::MatrixXd& source_normals, const Eigen::MatrixXd& target_vertices, const Eigen::MatrixXi& p_target_triangles, const Eigen::MatrixXd& p_target_normals, const Eigen::MatrixXd& p_source_weights, double p_distance_threshold_squared, double p_angle_threshold_degrees) {
    Eigen::VectorXd squared_distance;
    Eigen::VectorXi closest_indices;
    Eigen::MatrixXd closest_points;

    igl::point_mesh_squared_distance(target_vertices, p_source_vertices, p_source_triangles, squared_distance, closest_indices, closest_points);

    Eigen::MatrixXd barycentric_coordinates(target_vertices.rows(), 3);
    for (int i = 0; i < target_vertices.rows(); ++i) {
        Eigen::RowVector3d v1 = p_source_vertices.row(p_source_triangles(closest_indices(i), 0));
        Eigen::RowVector3d v2 = p_source_vertices.row(p_source_triangles(closest_indices(i), 1));
        Eigen::RowVector3d v3 = p_source_vertices.row(p_source_triangles(closest_indices(i), 2));
        Eigen::RowVector3d point = closest_points.row(i);
        Eigen::RowVector3d bary;
        igl::barycentric_coordinates(point, v1, v2, v3, bary);
        barycentric_coordinates.row(i) = bary;
    }

    Eigen::MatrixXd target_weights = interpolate_attribute_from_bary(p_source_weights, barycentric_coordinates, closest_indices, p_source_triangles);
    Eigen::MatrixXd source_normals_matched_interpolated = interpolate_attribute_from_bary(source_normals, barycentric_coordinates, closest_indices, p_source_triangles);

    Eigen::VectorXi matched(target_vertices.rows());
    matched.setZero();

    for (int i = 0; i < target_vertices.rows(); ++i) {
        Eigen::Vector3d normalized_source_normal = normalize_vector(source_normals_matched_interpolated.row(i));
        Eigen::Vector3d normalized_target_normal = normalize_vector(p_target_normals.row(i));

        double radian_angle = std::acos(std::abs(normalized_source_normal.dot(normalized_target_normal)));
        double degree_angle = radian_angle * 180.0 / M_PI;

        if (squared_distance(i) <= p_distance_threshold_squared && degree_angle <= p_angle_threshold_degrees) {
            matched(i) = 1;
        }
    }

    return {matched, target_weights};
}

bool is_valid_array(const Eigen::MatrixXd& p_matrix) {
    return p_matrix.allFinite();
}

/**
 * Inpaint weights for all the vertices on the target mesh for which  we didnt 
 * find a good match on the source (i.e. Matched[i] == False).
 * 
 *  V2: #V2 by 3 target mesh vertices
 *  F2: #F2 by 3 target mesh triangles indices
 *  W2: #V2 by num_bones, where W2[i,:] are skinning weights copied directly from source using closest point method
 *  Matched: #V2 array of bools, where Matched[i] is True if we found a good match for vertex i on the source mesh
 *  W_inpainted: #V2 by num_bones, final skinning weights where we inpainted weights for all vertices i where Matched[i] == False
 *  success: true if inpainting succeeded, false otherwise
 */
bool inpaint(const Eigen::MatrixXd& p_V2, const Eigen::MatrixXi& p_F2, const Eigen::MatrixXd& p_W2, const Eigen::Array<bool,Eigen::Dynamic,1>& p_Matched, Eigen::MatrixXd& r_W_inpainted)
{
    // Compute the laplacian
    Eigen::SparseMatrix<double> L, M, Minv;
    igl::cotmatrix(p_V2, p_F2, L);
    igl::massmatrix(p_V2, p_F2, igl::MASSMATRIX_TYPE_VORONOI, M);
    igl::invert_diag(M, Minv);

    // L, M = robust_laplacian.mesh_laplacian(V2, F2)
    Eigen::SparseMatrix<double> Q = -L + L * Minv * L;
    Eigen::SparseMatrix<double> Aeq;
    Eigen::VectorXd Beq;
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(L.rows(), p_W2.cols());

    Eigen::VectorXi b_all = Eigen::VectorXi::LinSpaced(p_V2.rows(), 0, p_V2.rows() - 1);
    Eigen::VectorXi b;
    igl::slice_mask(b_all, p_Matched, 1, b);

    Eigen::MatrixXd bc;
    igl::slice_mask(p_W2, p_Matched, 1, bc);

    igl::min_quad_with_fixed_data<double> mqwf;
    igl::min_quad_with_fixed_precompute(Q, b, Aeq, true, mqwf);

    bool result = igl::min_quad_with_fixed_solve(mqwf, B, bc, Beq, r_W_inpainted);

    return result;
}

/**
 * Smooth weights in the areas for which weights were inpainted and also their close neighbours.
 * 
 *  V2: #V2 by 3 target mesh vertices
 *  F2: #F2 by 3 target mesh triangles indices
 *  W2: #V2 by num_bones skinning weights
 *  Matched: #V2 array of bools, where Matched[i] is True if we found a good match for vertex i on the source mesh
 *  dDISTANCE_THRESHOLD_SQRD: scalar distance threshold
 *  num_smooth_iter_steps: scalar number of smoothing steps
 *  smooth_alpha: scalar the smoothing strength      
 *  W2_smoothed: #V2 by num_bones new smoothed weights
 *  VIDs_to_smooth: 1D array of vertex IDs for which smoothing was applied
 */
void smooth(Eigen::MatrixXd& W2_smoothed,
            Eigen::Array<bool,Eigen::Dynamic,1>& VIDs_to_smooth,
            const Eigen::MatrixXd& V2, 
            const Eigen::MatrixXi& F2, 
            const Eigen::MatrixXd& W2, 
            const Eigen::Array<bool,Eigen::Dynamic,1>& Matched, 
            const double dDISTANCE_THRESHOLD, 
            const double num_smooth_iter_steps, 
            const double smooth_alpha)
{
    Eigen::Array<bool,Eigen::Dynamic,1> NotMatched = Matched.select(Eigen::Array<bool,Eigen::Dynamic,1>::Constant(Matched.size(), false), Eigen::Array<bool,Eigen::Dynamic,1>::Constant(Matched.size(), true));
    VIDs_to_smooth = NotMatched; //.array(NotMatched, copy=True)

    std::vector<std::vector<int> > adj_list;
    igl::adjacency_list(F2, adj_list);

    auto get_points_within_distance = [&](const Eigen::MatrixXd& V, const int VID, const double distance)
    {
        // Get all neighbours of vertex VID within dDISTANCE_THRESHOLD   
        std::queue<int> queue;
        queue.push(VID);

        std::set<int> visited;
        visited.insert(VID);
        while (!queue.empty())
        {
            const int vv = queue.front();
            queue.pop();
            
            auto neigh = adj_list[vv];
            for (auto nn : neigh)
            {
                if (!VIDs_to_smooth[nn] && (V.row(VID) - V.row(nn)).norm() < distance)
                {
                    VIDs_to_smooth[nn] = true;
                    if (visited.find(nn) == visited.end())
                    {
                        queue.push(nn);
                        visited.insert(nn);
                    }
                }
            }
        }
    };

    for (int i = 0; i < V2.rows(); ++i)
    {
        if (NotMatched[i])
        {
            get_points_within_distance(V2, i, dDISTANCE_THRESHOLD);
        }
    }

    W2_smoothed = W2;

    for (int step_idx = 0;  step_idx < num_smooth_iter_steps; ++step_idx)
    {
        for (int i = 0; i < V2.rows(); ++i)
        {
            if (VIDs_to_smooth[i])
            {
                auto neigh = adj_list[i];
                int num = neigh.size();
                Eigen::VectorXd weight = W2_smoothed.row(i);

                Eigen::VectorXd new_weight = (1.0-smooth_alpha)*weight;

                for (auto influence_idx : neigh)
                {
                    Eigen::VectorXd weight_connected = W2_smoothed.row(influence_idx);
                    new_weight = new_weight + (smooth_alpha/num) * weight_connected;
                }
                
                W2_smoothed.row(i) = new_weight; 
            }
        }
    }
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
    Eigen::Array<bool, Eigen::Dynamic, 1> Matched(4);
    Matched << true, true, true, false;
    Eigen::MatrixXd expected_W_inpainted(4, 2);
    expected_W_inpainted << 1.0, 0.0,
                            0.0, 1.0,
                            0.5, 0.5,
                            0.117647, 0.882353;

    Eigen::MatrixXd W_inpainted(4, 2);
    
    bool success = inpaint(V2, F2, W2, Matched, W_inpainted);
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
    Eigen::Array<bool, Eigen::Dynamic, 1> matched(5);
    matched << true, true, true, false, false;
    double distance_threshold = 1.5;

    Eigen::MatrixXd expected_smoothed_weights(5, 2);
    expected_smoothed_weights << 0.85, 0.15,
                                 0.106667, 0.893333,
                                 0.480444, 0.519556,
                                 0.258711, 0.741289,
                                 0.08, 0.72;
    Eigen::Array<bool, Eigen::Dynamic, 1> expected_vertices_ids_to_smooth(5);
    expected_vertices_ids_to_smooth << true, true, true, true, true;

    Eigen::MatrixXd smoothed_weights;
    Eigen::Array<bool, Eigen::Dynamic, 1> vertices_ids_to_smooth;
    smooth(smoothed_weights, vertices_ids_to_smooth, target_vertices, target_faces, skinning_weights, matched, distance_threshold, 1, 0.2);

    std::cout << "Smoothed Weights:\n" << smoothed_weights << std::endl;
    std::cout << "Expected Smoothed Weights:\n" << expected_smoothed_weights << std::endl;
    std::cout << "Vertices IDs to Smooth:\n" << vertices_ids_to_smooth << std::endl;
    std::cout << "Expected Vertices IDs to Smooth:\n" << expected_vertices_ids_to_smooth << std::endl;

    if (!smoothed_weights.isApprox(expected_smoothed_weights, 1e-6) || (vertices_ids_to_smooth != expected_vertices_ids_to_smooth).any()) {
        return false;
    }
    return true;
}

extern "C" Array find_matches_closest_surface_mesh(Mesh source_mesh, Mesh target_mesh) {
    Array result;
    if (!source_mesh.is_valid()) {
        std::cerr << "Source mesh is not valid." << std::endl;
        return result;
    }
    if (!target_mesh.is_valid()) {
        std::cerr << "Target mesh is not valid." << std::endl;
        return result;
    }

    for (int i = 0; i < source_mesh.get_surface_count(); ++i) {
        Array arrays = source_mesh.surface_get_arrays(i);
        PackedArray<Vector3> source_vertices_godot = arrays.at(Mesh::ARRAY_VERTEX);
        PackedArray<int32_t> source_indices_godot = arrays.at(Mesh::ARRAY_INDEX);
        PackedArray<Vector3> source_normals_godot = arrays.at(Mesh::ARRAY_NORMAL);

        auto source_vertices_vec = source_vertices_godot.fetch();
        Eigen::MatrixXd source_vertices(source_vertices_vec.size(), 3);
        for (size_t j = 0; j < source_vertices_vec.size(); ++j) {
            Vector3 v = source_vertices_vec[j];
            source_vertices.row(j) << v.x, v.y, v.z;
        }

        auto source_indices_vec = source_indices_godot.fetch();
        Eigen::MatrixXi source_triangles(source_indices_vec.size() / 3, 3);
        for (size_t j = 0; j < source_indices_vec.size(); j += 3) {
            source_triangles.row(j / 3) << source_indices_vec[j], source_indices_vec[j + 1], source_indices_vec[j + 2];
        }

        auto source_normals_vec = source_normals_godot.fetch();
        Eigen::MatrixXd source_normals(source_normals_vec.size(), 3);
        for (size_t j = 0; j < source_normals_vec.size(); ++j) {
            Vector3 n = source_normals_vec[j];
            source_normals.row(j) << n.x, n.y, n.z;
        }

        for (int j = 0; j < target_mesh.get_surface_count(); ++j) {
            Array target_arrays = target_mesh.surface_get_arrays(j);
            PackedArray<Vector3> target_vertices_godot = target_arrays.at(Mesh::ARRAY_VERTEX);
            PackedArray<int32_t> target_indices_godot = target_arrays.at(Mesh::ARRAY_INDEX);
            PackedArray<Vector3> target_normals_godot = target_arrays.at(Mesh::ARRAY_NORMAL);

            auto target_vertices_vec = target_vertices_godot.fetch();
            Eigen::MatrixXd target_vertices(target_vertices_vec.size(), 3);
            for (size_t k = 0; k < target_vertices_vec.size(); ++k) {
                Vector3 v = target_vertices_vec[k];
                target_vertices.row(k) << v.x, v.y, v.z;
            }

            auto target_indices_vec = target_indices_godot.fetch();
            Eigen::MatrixXi target_triangles(target_indices_vec.size() / 3, 3);
            for (size_t k = 0; k < target_indices_vec.size(); k += 3) {
                target_triangles.row(k / 3) << target_indices_vec[k], target_indices_vec[k + 1], target_indices_vec[k + 2];
            }

            auto target_normals_vec = target_normals_godot.fetch();
            Eigen::MatrixXd target_normals(target_normals_vec.size(), 3);
            for (size_t k = 0; k < target_normals_vec.size(); ++k) {
                Vector3 n = target_normals_vec[k];
                target_normals.row(k) << n.x, n.y, n.z;
            }

            Eigen::MatrixXd source_weights(source_vertices.rows(), 2);
            source_weights.setZero();

            double distance_threshold_squared = 0.5;
            double angle_threshold_degrees = 10;

            auto [matched, target_weights] = find_matches_closest_surface(source_vertices, source_triangles, source_normals, target_vertices, target_triangles, target_normals, source_weights, distance_threshold_squared, angle_threshold_degrees);

            std::vector<int32_t> matched_std;
            for (int m = 0; m < matched.size(); ++m) {
                matched_std.push_back(matched(m));
            }

            std::vector<Vector2> target_weights_std;
            for (int t = 0; t < target_weights.rows(); ++t) {
                target_weights_std.push_back(Vector2(target_weights(t, 0), target_weights(t, 1)));
            }

            Dictionary surface_result;
            surface_result["matched"] = PackedArray<int32_t>(matched_std);
            surface_result["target_weights"] = PackedArray<Vector2>(target_weights_std);
            result.append(surface_result);

            std::cout << "Surface " << j << " processed." << std::endl;
        }
        std::cout << "Source surface " << i << " processed." << std::endl;
    }

    return result;
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
    // if (!test_inpaint()) {
    //     std::cerr << "test_inpaint failed" << std::endl;
    //     all_tests_passed = false;
    // }
    if (all_tests_passed) {
        std::cout << "All tests passed!" << std::endl;
        return Variant(0);
    } else {
        std::cerr << "Some tests failed." << std::endl;
        return Variant(1);
    }
}
