#include "roscon_collision/sdf_model.hpp"
#include "roscon_collision/sphere_model.hpp"
#include <algorithm>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace roscon_collision {
SdfObject makeSdfObject(const fcl::CollisionGeometryd &geometry, const fcl::Transform3d &world_pose,
                        std::vector<SdfTriangle> &triangles) {
  SdfObject object;
  object.world_to_local = makeSphereTransform(world_pose.inverse());
  if (const auto *box = dynamic_cast<const fcl::Boxd *>(&geometry)) {
    object.shape = SdfShape::Box;
    object.half_x = box->side.x() / 2.0;
    object.half_y = box->side.y() / 2.0;
    object.half_z = box->side.z() / 2.0;
  } else if (const auto *sphere = dynamic_cast<const fcl::Sphered *>(&geometry)) {
    object.shape = SdfShape::Sphere;
    object.radius = sphere->radius;
  } else if (const auto *cylinder = dynamic_cast<const fcl::Cylinderd *>(&geometry)) {
    object.shape = SdfShape::Cylinder;
    object.radius = cylinder->radius;
    object.half_z = cylinder->lz / 2.0;
  } else if (const auto *mesh = dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd> *>(&geometry)) {
    object.shape = SdfShape::Mesh;
    object.triangle_start = triangles.size();
    object.triangle_count = mesh->num_tris;
    if (mesh->num_tris <= 0 || triangles.size() + mesh->num_tris > 100000) {
      throw std::runtime_error(
          "SDF meshes must contain triangles; scene limit is 100000 triangles");
    }
    // Assimp can duplicate vertices along normal seams. Weld exact positions for topology checks.
    std::map<std::tuple<double, double, double>, int> vertex_ids;
    std::map<std::pair<int, int>, int> edge_counts;
    std::map<std::pair<int, int>, int> edge_directions;
    std::vector<SdfTriangle> mesh_triangles;
    for (int index = 0; index < mesh->num_tris; ++index) {
      Eigen::Vector3d vertices[3];
      int ids[3];
      for (int corner = 0; corner < 3; ++corner) {
        vertices[corner] = mesh->vertices[mesh->tri_indices[index][corner]];
        const auto &vertex = vertices[corner];
        if (!vertex.allFinite()) {
          throw std::runtime_error("SDF mesh contains non-finite vertices");
        }
        const auto key = std::make_tuple(vertex.x(), vertex.y(), vertex.z());
        auto entry = vertex_ids.find(key);
        if (entry == vertex_ids.end()) {
          entry = vertex_ids.emplace(key, vertex_ids.size()).first;
        }
        ids[corner] = entry->second;
      }
      if ((vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).squaredNorm() < 1e-24) {
        throw std::runtime_error("SDF mesh contains degenerate triangles");
      }
      for (int corner = 0; corner < 3; ++corner) {
        const int first = ids[corner];
        const int second = ids[(corner + 1) % 3];
        const auto edge = std::minmax(first, second);
        ++edge_counts[edge];
        edge_directions[edge] += first < second ? 1 : -1;
      }
      mesh_triangles.push_back({{vertices[0].x(), vertices[0].y(), vertices[0].z()},
                                {vertices[1].x(), vertices[1].y(), vertices[1].z()},
                                {vertices[2].x(), vertices[2].y(), vertices[2].z()}});
    }
    for (const auto &edge : edge_counts) {
      if (edge.second != 2 || edge_directions.at(edge.first) != 0) {
        throw std::runtime_error(
            "SDF mesh must be watertight and consistently oriented; repair the mesh or use FCL");
      }
    }
    triangles.insert(triangles.end(), mesh_triangles.begin(), mesh_triangles.end());
  } else {
    throw std::runtime_error("Unsupported SDF environment geometry");
  }
  return object;
}
} // namespace roscon_collision
