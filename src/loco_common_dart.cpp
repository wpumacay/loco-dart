
#include <loco_common_dart.h>

namespace loco {
namespace dartsim {

    Eigen::Vector3d vec3_to_eigen( const TVec3& vec )
    {
        return Eigen::Vector3d( vec.x(), vec.y(), vec.z() );
    }

    Eigen::Vector4d vec4_to_eigen( const TVec4& vec )
    {
        return Eigen::Vector4d( vec.x(), vec.y(), vec.z(), vec.w() );
    }

    Eigen::Matrix3d mat3_to_eigen( const TMat3& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        Eigen::Matrix3d eig_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                eig_mat( i, j ) = mat( i, j );
        return eig_mat;
    }

    Eigen::Matrix4d mat4_to_eigen( const TMat4& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        Eigen::Matrix4d eig_mat;
        for ( size_t i = 0; i < 4; i++ )
            for ( size_t j = 0; j < 4; j++ )
                eig_mat( i, j ) = mat( i, j );
        return eig_mat;
    }

    Eigen::Isometry3d mat4_to_eigen_tf( const TMat4& mat )
    {
        Eigen::Isometry3d tf( Eigen::Isometry3d::Identity() );
        Eigen::Vector3d translation = vec3_to_eigen( mat.col( 3 ) );
        Eigen::Matrix3d rotation;
        for ( ssize_t i = 0; i < 3; i++ )
            for ( ssize_t j = 0; j < 3; j++ )
                rotation( i, j ) = mat( i, j );

        tf.rotate( rotation );
        tf.translation() = translation;
        return tf;
    }

    TVec3 vec3_from_eigen( const Eigen::Vector3d& vec )
    {
        return TVec3( vec.x(), vec.y(), vec.z() );
    }

    TVec4 vec4_from_eigen( const Eigen::Vector4d& vec )
    {
        return TVec4( vec.x(), vec.y(), vec.z(), vec.w() );
    }

    TMat3 mat3_from_eigen( const Eigen::Matrix3d& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        TMat3 tm_mat;
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                tm_mat( i, j ) = mat( i, j );
        return tm_mat;
    }

    TMat4 mat4_from_eigen( const Eigen::Matrix4d& mat )
    {
        // @note: can't directly memcpy as internal-types are different (double vs float)
        TMat4 tm_mat;
        for ( size_t i = 0; i < 4; i++ )
            for ( size_t j = 0; j < 4; j++ )
                tm_mat( i, j ) = mat( i, j );
        return tm_mat;
    }

    TMat4 mat4_from_eigen_tf( const Eigen::Isometry3d& tf )
    {
        TMat4 tm_mat;
        tm_mat.set( vec3_from_eigen( tf.translation() ), 3 );
        auto& rotation = tf.rotation();
        for ( ssize_t i = 0; i < 3; i++ )
            for ( ssize_t j = 0; j < 3; j++ )
                tm_mat( i, j ) = rotation( i, j );
        return tm_mat;
    }

    dart::dynamics::ShapePtr CreateCollisionShape( const TShapeData& data )
    {
        switch ( data.type )
        {
            case eShapeType::PLANE :
                return std::make_shared<dart::dynamics::BoxShape>( vec3_to_eigen( { data.size.x(), data.size.y(), 0.01 } ) );
            case eShapeType::BOX :
                return std::make_shared<dart::dynamics::BoxShape>( vec3_to_eigen( data.size ) );
            case eShapeType::SPHERE :
                return std::make_shared<dart::dynamics::SphereShape>( data.size.x() );
            case eShapeType::CYLINDER :
                return std::make_shared<dart::dynamics::CylinderShape>( data.size.x(), data.size.y() );
            case eShapeType::CAPSULE :
                return std::make_shared<dart::dynamics::CapsuleShape>( data.size.x(), data.size.y() );
            case eShapeType::ELLIPSOID :
                return std::make_shared<dart::dynamics::EllipsoidShape>( 2.0 * vec3_to_eigen( data.size ) );
            case eShapeType::MESH :
            {
                const auto& mesh_data = data.mesh_data;
                if ( mesh_data.filename != "" )
                {
                    if ( const auto assimp_scene = dart::dynamics::MeshShape::loadMesh( mesh_data.filename ) )
                        return std::make_shared<dart::dynamics::MeshShape>( vec3_to_eigen( data.size ), assimp_scene );
                }
                else if ( mesh_data.vertices.size() > 0 )
                {
                    if ( const auto assimp_scene = CreateAssimpSceneFromVertexData( mesh_data ) )
                        return std::make_shared<dart::dynamics::MeshShape>( vec3_to_eigen( data.size ), assimp_scene );
                }

                LOCO_CORE_ERROR( "CreateCollisionShape >>> Couldn't create dart mesh-shape" );
                return nullptr;
            }
            case eShapeType::HFIELD :
            {
                const auto& hfield_data = data.hfield_data;
                const ssize_t num_width_samples = hfield_data.nWidthSamples;
                const ssize_t num_depth_samples = hfield_data.nDepthSamples;
                const auto& heights = hfield_data.heights;
                const float scale_x = data.size.x() / ( num_width_samples - 1 );
                const float scale_y = data.size.y() / ( num_depth_samples - 1 );
                const float scale_z = data.size.z();

                auto hfield_shape = std::make_shared<dart::dynamics::HeightmapShapef>();
                hfield_shape->setHeightField( num_width_samples, num_depth_samples, heights );
                hfield_shape->setScale( Eigen::Vector3f( scale_x, scale_y, scale_z ) );
                return hfield_shape;
            }
        }

        LOCO_CORE_ERROR( "CreateCollisionShape >>> Couldn't create dart coll-shape" );
        return nullptr;
    }

    const aiScene* CreateAssimpSceneFromVertexData( const TMeshData& mesh_data )
    {
        // @todo: implement me
        return nullptr;
    }



}}