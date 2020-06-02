#pragma once

#include <loco_common.h>
#include <loco_data.h>
// Main Dart-API
#include <dart/dart.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

namespace loco {
namespace dartsim {

    // Conversions from and to eigen-math types and tinymath-math types
    Eigen::Vector3d vec3_to_eigen( const TVec3& vec );
    Eigen::Vector4d vec4_to_eigen( const TVec4& vec );
    Eigen::Matrix3d mat3_to_eigen( const TMat3& mat );
    Eigen::Matrix4d mat4_to_eigen( const TMat4& mat );
    Eigen::Isometry3d mat4_to_eigen_tf( const TMat4& mat );
    TVec3 vec3_from_eigen( const Eigen::Vector3d& vec );
    TVec4 vec4_from_eigen( const Eigen::Vector4d& vec );
    TMat3 mat3_from_eigen( const Eigen::Matrix3d& mat );
    TMat4 mat4_from_eigen( const Eigen::Matrix4d& mat );
    TMat4 mat4_from_eigen_tf( const Eigen::Isometry3d& tf );


    // Creates a dart collision-shape from given user-data
    dart::dynamics::ShapePtr CreateCollisionShape( const TShapeData& data );

    // Creates an assimp-scene object from given user data
    const aiScene* CreateAssimpSceneFromVertexData( const std::vector<float>& vertices, const std::vector<int>& faces );

    // Custom ODE-like collision-filtering functionality
    class TDartBitmaskCollisionFilter : public dart::collision::BodyNodeCollisionFilter
    {
        public :

            bool ignoresCollision( const dart::collision::CollisionObject* object_1,
                                   const dart::collision::CollisionObject* object_2 ) const override;

            void setCollisionGroup( const dart::dynamics::ShapeNode* shape, int collision_group );

            void setCollisionMask( const dart::dynamics::ShapeNode* shape, int collision_mask );

        private :

            std::unordered_map<const dart::dynamics::ShapeNode*,int> m_CollisionGroupsMap;
            std::unordered_map<const dart::dynamics::ShapeNode*,int> m_CollisionMasksMap;
    };
}}