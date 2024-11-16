#include <api.hpp>
#include <iostream>
#include <iomanip>

#include <Eigen/Dense>
#include <igl/point_mesh_squared_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/bounding_box_diagonal.h>
#include <fstream>
#include <igl/AABB.h>

#include "robust_weight_transfer.h"

extern "C" void funlockfile(FILE *) {}
extern "C" void flockfile(FILE *) {}

