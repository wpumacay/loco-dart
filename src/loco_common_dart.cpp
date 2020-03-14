
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
        if ( mesh_data.vertices.size() % 3 != 0 )
            LOCO_CORE_ERROR( "CreateAssimpSceneFromVertexData >>> there must be 3 elements per vertex" );
        if ( mesh_data.faces.size() % 3 != 0 )
            LOCO_CORE_ERROR( "CreateAssimpSceneFromVertexData >>> there must be 3 elements per face" );

        const ssize_t num_vertices = mesh_data.vertices.size() / 3;
        const ssize_t num_faces = mesh_data.faces.size() / 3;

        auto assimp_scene = new aiScene();
        assimp_scene->mMaterials = new aiMaterial*[1];
        assimp_scene->mMaterials[0] = new aiMaterial();
        assimp_scene->mNumMaterials = 1;
        assimp_scene->mMeshes = new aiMesh*[1];
        assimp_scene->mMeshes[0] = new aiMesh();
        assimp_scene->mMeshes[0]->mMaterialIndex = 0;
        assimp_scene->mNumMeshes = 1;
        assimp_scene->mRootNode = new aiNode();
        assimp_scene->mRootNode->mMeshes = new unsigned int[1];
        assimp_scene->mRootNode->mMeshes[0] = 0;
        assimp_scene->mRootNode->mNumMeshes = 1;

        auto assimp_mesh = assimp_scene->mMeshes[0];
        assimp_mesh->mVertices = new aiVector3D[num_vertices];
        assimp_mesh->mNumVertices = num_vertices;
        assimp_mesh->mFaces = new aiFace[num_faces];
        assimp_mesh->mNumFaces = num_faces;
        for ( ssize_t v = 0; v < num_vertices; v++ )
        {
            assimp_mesh->mVertices[v] = aiVector3D( mesh_data.vertices[3 * v + 0],
                                                    mesh_data.vertices[3 * v + 1],
                                                    mesh_data.vertices[3 * v + 2] );
        }
        for ( ssize_t f = 0; f < num_faces; f++ )
        {
            aiFace& assimp_face = assimp_mesh->mFaces[f];
            assimp_face.mIndices = new unsigned int[3];
            assimp_face.mNumIndices = 3;
            assimp_face.mIndices[0] = mesh_data.faces[3 * f + 0];
            assimp_face.mIndices[1] = mesh_data.faces[3 * f + 1];
            assimp_face.mIndices[2] = mesh_data.faces[3 * f + 2];
        }
        //// aiExportScene( assimp_scene, "obj", "./mesh_sample.obj", 0x0 );
        return assimp_scene;
    }

}}