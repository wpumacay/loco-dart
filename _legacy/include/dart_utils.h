#pragma once

#include <dart_common.h>

// Assimp helper functionality
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

using namespace dart;

namespace tysoc {
namespace utils {

    Eigen::Vector3d toEigenVec3( const tysoc::TVec3& vec );
    Eigen::Isometry3d toEigenTransform( const tysoc::TMat4& transform );
    tysoc::TVec3 fromEigenVec3( const Eigen::Vector3d& vec );
    tysoc::TMat4 fromEigenTransform( const Eigen::Isometry3d& transform );

    dynamics::ShapePtr createCollisionShape( const TCollisionData& colliderData );

}}