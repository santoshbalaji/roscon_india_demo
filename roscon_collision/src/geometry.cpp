#include "roscon_collision/geometry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <vector>

namespace roscon_collision {
std::string resolveAssetPath(const std::string &resource) {
  const std::string package_prefix = "package://";
  const std::string file_prefix = "file://";
  if (resource.compare(0, package_prefix.size(), package_prefix) == 0) {
    const std::string package_path = resource.substr(package_prefix.size());
    const std::size_t separator = package_path.find('/');
    if (separator == std::string::npos) {
      throw std::runtime_error("Package URI needs a file path: " + resource);
    }
    return ament_index_cpp::get_package_share_directory(package_path.substr(0, separator)) +
           package_path.substr(separator);
  }
  if (resource.compare(0, file_prefix.size(), file_prefix) == 0) {
    return resource.substr(file_prefix.size());
  }
  if (!std::filesystem::path(resource).is_absolute()) {
    throw std::runtime_error("Use package://, file://, or an absolute asset path: " + resource);
  }
  return resource;
}

fcl::Transform3d poseToTransform(const urdf::Pose &pose) {
  fcl::Transform3d transform = fcl::Transform3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                                    pose.rotation.z);
  transform.linear() = rotation.normalized().toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose transformToPose(const fcl::Transform3d &transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond rotation(transform.rotation());
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

static void requirePositive(double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error("Geometry dimensions and mesh scales must be finite and positive");
  }
}

Shape loadShape(const urdf::Geometry &geometry) {
  Shape shape;
  shape.marker.action = visualization_msgs::msg::Marker::ADD;
  shape.marker.pose.orientation.w = 1.0;
  if (geometry.type == urdf::Geometry::BOX) {
    const urdf::Box &box = dynamic_cast<const urdf::Box &>(geometry);
    requirePositive(box.dim.x);
    requirePositive(box.dim.y);
    requirePositive(box.dim.z);
    shape.geometry = std::make_shared<fcl::Boxd>(box.dim.x, box.dim.y, box.dim.z);
    shape.marker.type = visualization_msgs::msg::Marker::CUBE;
    shape.marker.scale.x = box.dim.x;
    shape.marker.scale.y = box.dim.y;
    shape.marker.scale.z = box.dim.z;
  } else if (geometry.type == urdf::Geometry::SPHERE) {
    const urdf::Sphere &sphere = dynamic_cast<const urdf::Sphere &>(geometry);
    requirePositive(sphere.radius);
    shape.geometry = std::make_shared<fcl::Sphered>(sphere.radius);
    shape.marker.type = visualization_msgs::msg::Marker::SPHERE;
    shape.marker.scale.x = 2.0 * sphere.radius;
    shape.marker.scale.y = 2.0 * sphere.radius;
    shape.marker.scale.z = 2.0 * sphere.radius;
  } else if (geometry.type == urdf::Geometry::CYLINDER) {
    const urdf::Cylinder &cylinder = dynamic_cast<const urdf::Cylinder &>(geometry);
    requirePositive(cylinder.radius);
    requirePositive(cylinder.length);
    shape.geometry = std::make_shared<fcl::Cylinderd>(cylinder.radius, cylinder.length);
    shape.marker.type = visualization_msgs::msg::Marker::CYLINDER;
    shape.marker.scale.x = 2.0 * cylinder.radius;
    shape.marker.scale.y = 2.0 * cylinder.radius;
    shape.marker.scale.z = cylinder.length;
  } else if (geometry.type == urdf::Geometry::MESH) {
    const urdf::Mesh &mesh = dynamic_cast<const urdf::Mesh &>(geometry);
    requirePositive(mesh.scale.x);
    requirePositive(mesh.scale.y);
    requirePositive(mesh.scale.z);
    const std::string path = resolveAssetPath(mesh.filename);
    Assimp::Importer importer;
    const aiScene *scene =
        importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                                    aiProcess_PreTransformVertices);
    if (scene == nullptr || !scene->HasMeshes()) {
      throw std::runtime_error("Cannot load mesh " + path + ": " + importer.GetErrorString());
    }
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
    for (unsigned int mesh_index = 0; mesh_index < scene->mNumMeshes; ++mesh_index) {
      const aiMesh &part = *scene->mMeshes[mesh_index];
      const int vertex_offset = static_cast<int>(vertices.size());
      for (unsigned int vertex_index = 0; vertex_index < part.mNumVertices; ++vertex_index) {
        const aiVector3D &vertex = part.mVertices[vertex_index];
        vertices.emplace_back(vertex.x * mesh.scale.x, vertex.y * mesh.scale.y,
                              vertex.z * mesh.scale.z);
      }
      for (unsigned int face_index = 0; face_index < part.mNumFaces; ++face_index) {
        const aiFace &face = part.mFaces[face_index];
        if (face.mNumIndices == 3) {
          triangles.emplace_back(vertex_offset + face.mIndices[0], vertex_offset + face.mIndices[1],
                                 vertex_offset + face.mIndices[2]);
        }
      }
    }
    if (triangles.empty()) {
      throw std::runtime_error("Mesh contains no triangles: " + path);
    }
    auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    if (model->beginModel() != fcl::BVH_OK ||
        model->addSubModel(vertices, triangles) != fcl::BVH_OK ||
        model->endModel() != fcl::BVH_OK) {
      throw std::runtime_error("FCL could not build mesh: " + path);
    }
    shape.geometry = model;
    shape.marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    shape.marker.mesh_resource = "file://" + path;
    shape.marker.mesh_use_embedded_materials = false;
    shape.marker.scale.x = mesh.scale.x;
    shape.marker.scale.y = mesh.scale.y;
    shape.marker.scale.z = mesh.scale.z;
  } else {
    throw std::runtime_error("Unsupported geometry type");
  }
  return shape;
}
} // namespace roscon_collision
