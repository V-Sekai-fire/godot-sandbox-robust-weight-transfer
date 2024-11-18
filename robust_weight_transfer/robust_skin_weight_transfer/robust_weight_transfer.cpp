#include <Eigen/Dense>
#include <cstdint>
#include <vector>
#include <iostream>
#include <set>

#include <api.hpp>
#include <Eigen/Dense>

#include <vector>
#include <tuple>

#include <igl/point_mesh_squared_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/adjacency_list.h>
#include <igl/invert_diag.h>
#include <igl/slice_mask.h>
#include <igl/min_quad_with_fixed.h>

#include "generated_api.hpp"


/**
 * Given a number of points find their closest points on the surface of the V,F mesh
 * 
 *  P: #P by 3, where every row is a point coordinate
 *  V: #V by 3 mesh vertices
 *  F: #F by 3 mesh triangles indices
 *  sqrD #P smallest squared distances
 *  I #P primitive indices corresponding to smallest distances
 *  C #P by 3 closest points
 *  B #P by 3 of the barycentric coordinates of the closest point
 */
void _find_closest_point_on_surface(const Eigen::MatrixXd& P, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, 
                                   Eigen::VectorXd& sqrD, Eigen::VectorXi& I, Eigen::MatrixXd& C, Eigen::MatrixXd& B)
{
    igl::point_mesh_squared_distance(P, V, F, sqrD, I, C);

    Eigen::MatrixXi F_closest = F(I, Eigen::indexing::all);
    Eigen::MatrixXd V1 = V(F_closest(Eigen::indexing::all, 0), Eigen::indexing::all);
    Eigen::MatrixXd V2 = V(F_closest(Eigen::indexing::all, 1), Eigen::indexing::all);
    Eigen::MatrixXd V3 = V(F_closest(Eigen::indexing::all, 2), Eigen::indexing::all);

    igl::barycentric_coordinates(C, V1, V2, V3, B);
}

/** 
 * Interpolate per-vertex attributes A via barycentric coordinates B of the F[I,:] vertices
 * 
 *  A: #V by N per-vertex attributes
 *  B  #B by 3 array of the barycentric coordinates of some points
 *  I  #B primitive indices containing the closest point
 *  F: #F by 3 mesh triangle indices
 *  A_out #B interpolated attributes
 */
void _interpolate_attribute_from_bary(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                     const Eigen::VectorXi& I, const Eigen::MatrixXi& F, 
                                     Eigen::MatrixXd& A_out)
{
    // https://stackoverflow.com/a/58700213
    // The documentation in OP's question refers to the 3.3.9 version which does not support symbols all, last,seq. For the most recent stable (3.3.7) version block or reshape operators must be used.
    Eigen::MatrixXi F_closest(I.size(), F.cols());
    for (int i = 0; i < I.size(); ++i) {
        F_closest.row(i) = F.row(I(i));
    }
    
    Eigen::MatrixXd a1(F_closest.rows(), A.cols());
    Eigen::MatrixXd a2(F_closest.rows(), A.cols());
    Eigen::MatrixXd a3(F_closest.rows(), A.cols());
    for (int i = 0; i < F_closest.rows(); ++i) {
        a1.row(i) = A.row(F_closest(i, 0));
        a2.row(i) = A.row(F_closest(i, 1));
        a3.row(i) = A.row(F_closest(i, 2));
    }

    Eigen::VectorXd b1 = B.col(0);
    Eigen::VectorXd b2 = B.col(1);
    Eigen::VectorXd b3 = B.col(2);

    for (int i = 0; i < a1.cols(); ++i) {
        a1.col(i) = a1.col(i).array() * b1.array();
        a2.col(i) = a2.col(i).array() * b2.array();
        a3.col(i) = a3.col(i).array() * b3.array();
    }

    A_out = a1 + a2 + a3;
}

Eigen::VectorXd normalize_vector(const Eigen::VectorXd& p_vector) {
    return p_vector.normalized();
}

/**
 * For each vertex on the target mesh find a match on the source mesh.
 * 
 *  V1: #V1 by 3 source mesh vertices
 *  F1: #F1 by 3 source mesh triangles indices
 *  N1: #V1 by 3 source mesh normals
 *  V2: #V2 by 3 target mesh vertices
 *  F2: #F2 by 3 target mesh triangles indices
 *  N2: #V2 by 3 target mesh normals
 *  W1: #V1 by num_bones source mesh skin weights
 *  dDISTANCE_THRESHOLD_SQRD: distance threshold
 *  dANGLE_THRESHOLD_DEGREES: normal threshold
 *  Matched: #V2 array of bools, where Matched[i] is True if we found a good match for vertex i on the source mesh
 *  W2: #V2 by num_bones, where W2[i,:] are skinning weights copied directly from source using closest point method
 */
void _find_matches_closest_surface(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& N1, 
                                  const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& N2, 
                                  const Eigen::MatrixXd& W1, 
                                  double dDISTANCE_THRESHOLD_SQRD, 
                                  double dANGLE_THRESHOLD_DEGREES,
                                  Eigen::MatrixXd& W2,
                                  Eigen::Array<bool,Eigen::Dynamic,1>& Matched)
{
    Matched = Eigen::Array<bool,Eigen::Dynamic,1>::Constant(V2.rows(), false);
    Eigen::VectorXd sqrD; 
    Eigen::VectorXi I;
    Eigen::MatrixXd C, B;
    _find_closest_point_on_surface(V2, V1, F1, sqrD, I, C, B);
    
    // for each closest point on the source, interpolate its per-vertex attributes(skin weights and normals) 
    // using the barycentric coordinates
    _interpolate_attribute_from_bary(W1, B, I, F1, W2);

    Eigen::MatrixXd N1_match_interpolated;
    _interpolate_attribute_from_bary(N1, B, I, F1, N1_match_interpolated);
    
    // check that the closest point passes our distance and normal thresholds
    for (int RowIdx = 0; RowIdx < V2.rows(); ++RowIdx)
    {
        Eigen::VectorXd n1 = N1_match_interpolated.row(RowIdx);
        n1.normalize();

        Eigen::VectorXd n2 = N2.row(RowIdx);
        n2.normalize();

        const double rad_angle = acos(n1.dot(n2));
        const double deg_angle = rad_angle * (180.0 / M_PI);

        if (sqrD(RowIdx) <= dDISTANCE_THRESHOLD_SQRD and deg_angle <= dANGLE_THRESHOLD_DEGREES)
        {
            Matched(RowIdx) = true;
        }
    }
}

extern "C" Variant find_matches_closest_surface_mesh(Mesh mesh_source, int source_surface_index, Mesh mesh_target, int target_surface_index, Array matched_godot, Array target_weights_godot) {
    Array source_arrays = mesh_source.surface_get_arrays(source_surface_index);
    Array target_arrays = mesh_target.surface_get_arrays(target_surface_index);

    PackedArray<Vector3> source_vertices_array_ref = source_arrays.at(Mesh::ARRAY_VERTEX).as_vector3_array();
    PackedArray<Vector3> source_normals_array_ref = source_arrays.at(Mesh::ARRAY_NORMAL).as_vector3_array();
    PackedArray<int32_t> source_triangles_array_ref = source_arrays.at(Mesh::ARRAY_INDEX).as_int32_array();
    PackedArray<float> source_weights_array_ref = source_arrays.at(Mesh::ARRAY_WEIGHTS).as_float32_array();

    PackedArray<Vector3> target_vertices_array_ref = target_arrays.at(Mesh::ARRAY_VERTEX).as_vector3_array();
    PackedArray<Vector3> target_normals_array_ref = target_arrays.at(Mesh::ARRAY_NORMAL).as_vector3_array();
    PackedArray<int32_t> target_triangles_array_ref = target_arrays.at(Mesh::ARRAY_INDEX).as_int32_array();

    std::vector<Vector3> source_vertices_array = source_vertices_array_ref.fetch();
    std::vector<Vector3> source_normals_array = source_normals_array_ref.fetch();
    std::vector<int32_t> source_triangles_array = source_triangles_array_ref.fetch();
    std::vector<float> source_weights_array = source_weights_array_ref.fetch();

    std::vector<Vector3> target_vertices_array = target_vertices_array_ref.fetch();
    std::vector<Vector3> target_normals_array = target_normals_array_ref.fetch();
    std::vector<int32_t> target_triangles_array = target_triangles_array_ref.fetch();

    if (source_vertices_array.empty() || source_triangles_array.empty() || source_normals_array.empty() || target_vertices_array.empty() || target_triangles_array.empty() || target_normals_array.empty() || source_weights_array.empty()) {
        std::cerr << "One or more input arrays are empty." << std::endl;
        return false;
    }

    Eigen::MatrixXd source_vertices(source_vertices_array.size(), 3);
    for (int j = 0; j < source_vertices_array.size(); ++j) {
        Vector3 v = source_vertices_array[j];
        source_vertices.row(j) << v.x, v.y, v.z;
    }

    Eigen::MatrixXi source_triangles(source_triangles_array.size() / 3, 3);
    for (int j = 0; j < source_triangles_array.size(); j += 3) {
        source_triangles.row(j / 3) << source_triangles_array[j], source_triangles_array[j + 1], source_triangles_array[j + 2];
    }

    Eigen::MatrixXd source_normals(source_normals_array.size(), 3);
    for (int j = 0; j < source_normals_array.size(); ++j) {
        Vector3 n = source_normals_array[j];
        source_normals.row(j) << n.x, n.y, n.z;
    }

    Eigen::MatrixXd target_vertices(target_vertices_array.size(), 3);
    for (int k = 0; k < target_vertices_array.size(); ++k) {
        Vector3 v = target_vertices_array[k];
        target_vertices.row(k) << v.x, v.y, v.z;
    }

    Eigen::MatrixXi target_triangles(target_triangles_array.size() / 3, 3);
    for (int k = 0; k < target_triangles_array.size(); k += 3) {
        target_triangles.row(k / 3) << target_triangles_array[k], target_triangles_array[k + 1], target_triangles_array[k + 2];
    }

    Eigen::MatrixXd target_normals(target_normals_array.size(), 3);
    for (int k = 0; k < target_normals_array.size(); ++k) {
        Vector3 n = target_normals_array[k];
        target_normals.row(k) << n.x, n.y, n.z;
    }

    int num_bones = source_weights_array.size() / source_vertices.rows();
    Eigen::MatrixXd source_weights(source_vertices.rows(), num_bones);
    for (int i = 0; i < source_vertices.rows(); ++i) {
        for (int j = 0; j < num_bones; ++j) {
            source_weights(i, j) = source_weights_array[i * num_bones + j];
        }
    }

    double distance_threshold_squared = 0.5;
    double angle_threshold_degrees = 10;

    Eigen::MatrixXd target_weights;
    Eigen::Array<bool, Eigen::Dynamic, 1> matched;
    _find_matches_closest_surface(source_vertices, source_triangles, source_normals, target_vertices, target_triangles, target_normals, source_weights,
        distance_threshold_squared, angle_threshold_degrees,
        target_weights, matched);

    for (int m = 0; m < matched.size(); ++m) {
        bool has_matched = matched(m) == 1;
        matched_godot.push_back(has_matched);
    }

    for (int t = 0; t < target_weights.rows(); ++t) {
        for (int b = 0; b < num_bones; ++b) {
            target_weights_godot.append(target_weights(t, b));
        }
    }
    return true;
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
    Eigen::MatrixXd expected_barycentric(3, 3);
    expected_barycentric << 0.5, 0, 0.5,
                            0, 0, 1,
                            0.5, 0, 0.5;

    Eigen::VectorXd sqrD;
    Eigen::VectorXi I;
    Eigen::MatrixXd C, B;
    _find_closest_point_on_surface(test_points, vertices, triangles, sqrD, I, C, B);

    std::cout << "Closest Points:\n" << C << std::endl;
    std::cout << "Expected Points:\n" << expected_points << std::endl;
    std::cout << "Barycentric Coordinates:\n" << B << std::endl;
    std::cout << "Expected Barycentric Coordinates:\n" << expected_barycentric << std::endl;
    if (!C.isApprox(expected_points, 1e-6) || !B.isApprox(expected_barycentric, 1e-6)) {
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

    Eigen::MatrixXd result;
    _interpolate_attribute_from_bary(vertex_attributes, barycentric_coordinates, primitive_indices, mesh_triangles, result);
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
    Eigen::MatrixXd source_vertices(4, 3);
    source_vertices << 0, 0, 0,
                       1, 0, 0,
                       0, 1, 0,
                       1, 1, 0;
    Eigen::MatrixXi source_triangles(2, 3);
    source_triangles << 0, 1, 2,
                        1, 2, 3;
    Eigen::MatrixXd source_normals(4, 3);
    source_normals << 0, 0, 1,
                      0, 0, 1,
                      0, 0, 1,
                      0, 0, 1;
    Eigen::MatrixXd source_weights(4, 8);
    source_weights << 1, 0, 0, 0, 0, 0, 0, 0,
                      0, 1, 0, 0, 0, 0, 0, 0,
                      0.5, 0.5, 0, 0, 0, 0, 0, 0,
                      0.25, 0.75, 0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd target_vertices(3, 3);
    target_vertices << 0.5, 0.5, 0,
                       1.5, 1.5, 0,
                       0.5, 0.5, 1;
    Eigen::MatrixXi target_triangles(1, 3);
    target_triangles << 0, 1, 2;
    Eigen::MatrixXd target_normals(3, 3);
    target_normals << 0, 0, 1,
                      0, 0, 1,
                      0, 0, 1;

    double distance_threshold_squared = 1.0;
    double angle_threshold_degrees = 10;
    Eigen::Array<bool, Eigen::Dynamic, 1> expected_matched(3);
    expected_matched << true, true, true;
    Eigen::MatrixXd expected_weights(3, 8);
    expected_weights << 0.25, 0.75, 0, 0, 0, 0, 0, 0,
                        0.25, 0.75, 0, 0, 0, 0, 0, 0,
                        0.25, 0.75, 0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd target_weights;
    Eigen::Array<bool, Eigen::Dynamic, 1> matched;
    _find_matches_closest_surface(source_vertices, source_triangles, source_normals, target_vertices, target_triangles, target_normals, source_weights, distance_threshold_squared, angle_threshold_degrees, target_weights, matched);

    std::cout << "Matched:\n" << matched << std::endl;
    std::cout << "Expected Matched:\n" << expected_matched << std::endl;
    std::cout << "Target Weights:\n" << target_weights << std::endl;
    std::cout << "Expected Weights:\n" << expected_weights << std::endl;
    if ((matched != expected_matched).any() || !target_weights.isApprox(expected_weights, 1e-6)) {
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
    expected_W_inpainted << 1, 0,
                            0, 1,
                            0.5, 0.5,
                            0.0357143 , 0.964286;

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

bool test_find_matches_closest_surface_mesh() {
    Eigen::MatrixXd source_vertices(4, 3);
    source_vertices << 0, 0, 0,
                       1, 0, 0,
                       0, 1, 0,
                       1, 1, 0;
    Eigen::MatrixXi source_triangles(2, 3);
    source_triangles << 0, 1, 2,
                        1, 2, 3;
    Eigen::MatrixXd source_normals(4, 3);
    source_normals << 0, 0, 1,
                      0, 0, 1,
                      0, 0, 1,
                      0, 0, 1;
    Eigen::MatrixXd source_weights(4, 8);
    source_weights << 1, 0, 0, 0, 0, 0, 0, 0,
                      0, 1, 0, 0, 0, 0, 0, 0,
                      0.5, 0.5, 0, 0, 0, 0, 0, 0,
                      0.25, 0.75, 0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd target_vertices(3, 3);
    target_vertices << 0.5, 0.5, 0,
                       1.5, 1.5, 0,
                       0.5, 0.5, 1;
    Eigen::MatrixXi target_triangles(1, 3);
    target_triangles << 0, 1, 2;
    Eigen::MatrixXd target_normals(3, 3);
    target_normals << 0, 0, 1,
                      0, 0, 1,
                      0, 0, 1;

    double distance_threshold_squared = 1.0;
    double angle_threshold_degrees = 10;
    Eigen::Array<bool, Eigen::Dynamic, 1> expected_matched(3);
    expected_matched << true, true, true;
    Eigen::MatrixXd expected_weights(3, 8);
    expected_weights << 0.25, 0.75, 0, 0, 0, 0, 0, 0,
                        0.25, 0.75, 0, 0, 0, 0, 0, 0,
                        0.25, 0.75, 0, 0, 0, 0, 0, 0;

    Eigen::MatrixXd target_weights;
    Eigen::Array<bool, Eigen::Dynamic, 1> matched;
    _find_matches_closest_surface(source_vertices, source_triangles, source_normals, target_vertices, target_triangles, target_normals, source_weights, distance_threshold_squared, angle_threshold_degrees, target_weights, matched);

    std::cout << "Matched:\n" << matched << std::endl;
    std::cout << "Expected Matched:\n" << expected_matched << std::endl;
    std::cout << "Target Weights:\n" << target_weights << std::endl;
    std::cout << "Expected Weights:\n" << expected_weights << std::endl;
    if ((matched != expected_matched).any() || !target_weights.isApprox(expected_weights, 1e-6)) {
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
    if (!test_find_matches_closest_surface_mesh()) {
        std::cerr << "test_find_matches_closest_surface_mesh failed" << std::endl;
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