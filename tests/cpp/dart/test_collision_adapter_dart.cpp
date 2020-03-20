
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_dart.h>
#include <primitives/loco_single_body_collider_adapter_dart.h>

bool allclose_vec3( const Eigen::Vector3d& eig_vec_1, const Eigen::Vector3d& eig_vec_2, double tolerance = 1e-5 )
{
    return ( std::abs( eig_vec_1.x() - eig_vec_2.x() ) <= tolerance ) &&
           ( std::abs( eig_vec_1.y() - eig_vec_2.y() ) <= tolerance ) &&
           ( std::abs( eig_vec_1.z() - eig_vec_2.z() ) <= tolerance );
}

bool allclose_vec3( const Eigen::Vector3d& eig_vec, const loco::TVec3& vec, double tolerance = 1e-5 )
{
    return ( std::abs( eig_vec.x() - vec.x() ) <= tolerance ) &&
           ( std::abs( eig_vec.y() - vec.y() ) <= tolerance ) &&
           ( std::abs( eig_vec.z() - vec.z() ) <= tolerance );
}

std::vector<float> create_hfield( size_t nWidthSamples, size_t nDepthSamples )
{
    std::vector<float> hfield( nWidthSamples * nDepthSamples );
    for ( size_t i = 0; i < nDepthSamples; i++ )
    {
        for ( size_t j = 0; j < nWidthSamples; j++ )
        {
            const float x = (float)j / ( nWidthSamples - 1 ) - 0.5f;
            const float y = (float)i / ( nDepthSamples - 1 ) - 0.5f;
            hfield[i * nWidthSamples + j] = 2.5f * ( x * x + y * y );
        }
    }
    return hfield;
}

std::pair<std::vector<float>, std::vector<int>> create_mesh_tetrahedron()
{
    std::vector<float> vertices = { 0.0f, 0.0f, 0.0f,
                                    1.0f, 0.0f, 0.0f,
                                    0.0f, 1.0f, 0.0f,
                                    0.0f, 0.0f, 1.0f };
    std::vector<int> faces = { 0, 2, 1,
                               0, 1, 3,
                               1, 2, 3,
                               0, 3, 2 };
    return { vertices, faces };
}

TEST( TestLocoDartCollisionAdapter, TestLocoDartCollisionAdapterBuild )
{
    loco::TLogger::Init();

    std::vector<loco::eShapeType> vec_col_types = { loco::eShapeType::BOX,
                                                    loco::eShapeType::SPHERE,
                                                    loco::eShapeType::PLANE,
                                                    loco::eShapeType::CYLINDER,
                                                    loco::eShapeType::CAPSULE,
                                                    loco::eShapeType::ELLIPSOID };
    std::vector<loco::TVec3> vec_col_sizes = { { 0.1, 0.2, 0.3 },
                                               { 0.1, 0.1, 0.1 },
                                               { 10.0, 10.0, 1.0 },
                                               { 0.2, 0.8, 0.2 },
                                               { 0.2, 0.8, 0.2 },
                                               { 0.2, 0.3, 0.4 } };
    std::vector<loco::TCollisionData> vec_col_data;
    for ( size_t i = 0; i < vec_col_types.size(); i++ )
    {
        auto col_data = loco::TCollisionData();
        col_data.type = vec_col_types[i];
        col_data.size = vec_col_sizes[i];
        vec_col_data.push_back( col_data );
    }
    std::vector<std::unique_ptr<loco::TSingleBodyCollider>> vec_colliders;
    std::vector<std::unique_ptr<loco::dartsim::TDartSingleBodyColliderAdapter>> vec_colliders_adapters;
    for ( size_t i = 0; i < vec_col_data.size(); i++ )
    {
        const auto collider_name = loco::ToString( vec_col_data[i].type ) + "_collider";
        auto col_obj = std::make_unique<loco::TSingleBodyCollider>( collider_name, vec_col_data[i] );
        auto col_adapter = std::make_unique<loco::dartsim::TDartSingleBodyColliderAdapter>( col_obj.get() );
        col_adapter->Build();
        ASSERT_TRUE( col_adapter->collision_shape() != nullptr );
        vec_colliders.push_back( std::move( col_obj ) );
        vec_colliders_adapters.push_back( std::move( col_adapter ) );
    }

    std::vector<std::string> vec_expected_types = { "BoxShape",
                                                    "SphereShape",
                                                    "BoxShape",
                                                    "CylinderShape",
                                                    "CapsuleShape",
                                                    "EllipsoidShape" };
    for ( size_t i = 0; i < vec_colliders_adapters.size(); i++ )
    {
        auto& dart_collision_shape = vec_colliders_adapters[i]->collision_shape();
        EXPECT_EQ( dart_collision_shape->getType(), vec_expected_types[i] );
        if ( vec_col_types[i] == loco::eShapeType::BOX )
        {
            auto box_shape = dynamic_cast<dart::dynamics::BoxShape*>( dart_collision_shape.get() );
            ASSERT_TRUE( box_shape != nullptr );
            const Eigen::Vector3d size = box_shape->getSize();
            const loco::TVec3 expected_size = { 0.1f, 0.2f, 0.3f };
            EXPECT_TRUE( allclose_vec3( size, expected_size ) );
        }
        else if ( vec_col_types[i] == loco::eShapeType::SPHERE )
        {
            auto sphere_shape = dynamic_cast<dart::dynamics::SphereShape*>( dart_collision_shape.get() );
            ASSERT_TRUE( sphere_shape != nullptr );
            const double radius = sphere_shape->getRadius();
            const loco::TScalar expected_radius = 0.1;
            EXPECT_TRUE( std::abs( radius - expected_radius ) < 1e-5 );
        }
        else if ( vec_col_types[i] == loco::eShapeType::CYLINDER )
        {
            auto cylinder_shape = dynamic_cast<dart::dynamics::CylinderShape*>( dart_collision_shape.get() );
            ASSERT_TRUE( cylinder_shape != nullptr );
            const double radius = cylinder_shape->getRadius();
            const double height = cylinder_shape->getHeight();
            const loco::TScalar expected_radius = 0.2;
            const loco::TScalar expected_height = 0.8;
            EXPECT_TRUE( std::abs( radius - expected_radius ) < 1e-5 );
            EXPECT_TRUE( std::abs( height - expected_height ) < 1e-5 );
        }
        else if ( vec_col_types[i] == loco::eShapeType::CAPSULE )
        {
            auto capsule_shape = dynamic_cast<dart::dynamics::CapsuleShape*>( dart_collision_shape.get() );
            ASSERT_TRUE( capsule_shape != nullptr );
            const double radius = capsule_shape->getRadius();
            const double height = capsule_shape->getHeight();
            const loco::TScalar expected_radius = 0.2;
            const loco::TScalar expected_height = 0.8;
            EXPECT_TRUE( std::abs( radius - expected_radius ) < 1e-5 );
            EXPECT_TRUE( std::abs( height - expected_height ) < 1e-5 );
        }
    }
}

TEST( TestLocoDartCollisionAdapter, TestLocoDartCollisionAdapterMeshBuild )
{
    loco::TLogger::Init();

    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::MESH;
    col_data.size = { 0.2f, 0.2f, 0.2f };
    col_data.mesh_data.filename = loco::PATH_RESOURCES + "meshes/monkey.stl";

    const auto collider_name = loco::ToString( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TSingleBodyCollider>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::dartsim::TDartSingleBodyColliderAdapter>( col_obj.get() );
    col_adapter->Build();
    auto& dart_collision_shape = col_adapter->collision_shape();
    ASSERT_TRUE( dart_collision_shape != nullptr );
    EXPECT_EQ( dart_collision_shape->getType(), "MeshShape" );
    auto dart_mesh_shape = dynamic_cast<dart::dynamics::MeshShape*>( dart_collision_shape.get() );
    ASSERT_TRUE( dart_mesh_shape != nullptr );
    EXPECT_TRUE( allclose_vec3( dart_mesh_shape->getScale(), Eigen::Vector3d( 0.2, 0.2, 0.2 ) ) );
}

TEST( TestLocoDartCollisionAdapter, TestLocoDartCollisionAdapterMeshUserBuild )
{
    loco::TLogger::Init();

    auto vertices_faces = create_mesh_tetrahedron();

    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::MESH;
    col_data.size = { 0.2f, 0.2f, 0.2f };
    col_data.mesh_data.vertices = vertices_faces.first;
    col_data.mesh_data.faces = vertices_faces.second;

    const auto collider_name = loco::ToString( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TSingleBodyCollider>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::dartsim::TDartSingleBodyColliderAdapter>( col_obj.get() );
    col_adapter->Build();
    auto& dart_collision_shape = col_adapter->collision_shape();
    ASSERT_TRUE( dart_collision_shape != nullptr );
    EXPECT_EQ( dart_collision_shape->getType(), dart::dynamics::MeshShape::getStaticType() );
    auto dart_mesh_shape = dynamic_cast<dart::dynamics::MeshShape*>( dart_collision_shape.get() );
    ASSERT_TRUE( dart_mesh_shape != nullptr );
}

TEST( TestLocoDartCollisionAdapter, TestLocoDartCollisionAdapterHfieldBuild )
{
    loco::TLogger::Init();

    const size_t num_width_samples = 40;
    const size_t num_depth_samples = 40;
    auto col_data = loco::TCollisionData();
    col_data.type = loco::eShapeType::HFIELD;
    col_data.size = { 10.0f, 10.0f, 2.0f }; // width, depth, scale-height
    col_data.hfield_data.nWidthSamples = num_width_samples;
    col_data.hfield_data.nDepthSamples = num_depth_samples;
    col_data.hfield_data.heights = create_hfield( num_width_samples, num_depth_samples );

    const loco::TVec4 expected_size = { 0.5f * 10.0f,
                                        0.5f * 10.0f,
                                        *std::max_element( col_data.hfield_data.heights.begin(),
                                                           col_data.hfield_data.heights.end() ) * 2.0f,
                                        loco::dartsim::LOCO_DART_HFIELD_BASE };

    const auto collider_name = loco::ToString( col_data.type ) + "_collider";
    auto col_obj = std::make_unique<loco::TSingleBodyCollider>( collider_name, col_data );
    auto col_adapter = std::make_unique<loco::dartsim::TDartSingleBodyColliderAdapter>( col_obj.get() );
    col_adapter->Build();
    auto& dart_collision_shape = col_adapter->collision_shape();
    ASSERT_TRUE( dart_collision_shape != nullptr );
    EXPECT_EQ( dart_collision_shape->getType(), dart::dynamics::HeightmapShapef::getStaticType() );
    auto dart_hfield_shape = dynamic_cast<dart::dynamics::HeightmapShapef*>( dart_collision_shape.get() );
    ASSERT_TRUE( dart_hfield_shape != nullptr );
    const auto dart_scale = dart_hfield_shape->getScale();
    const loco::TScalar expected_delta_width = col_data.size.x() / ( num_width_samples - 1 );
    const loco::TScalar expected_delta_depth = col_data.size.y() / ( num_depth_samples - 1 );
    const loco::TScalar expected_scale_height = col_data.size.z();
    EXPECT_TRUE( std::abs( dart_scale.x() - expected_delta_width ) < 1e-5 );
    EXPECT_TRUE( std::abs( dart_scale.y() - expected_delta_depth ) < 1e-5 );
    EXPECT_TRUE( std::abs( dart_scale.z() - expected_scale_height ) < 1e-5 );
    EXPECT_EQ( dart_hfield_shape->getWidth(), num_width_samples );
    EXPECT_EQ( dart_hfield_shape->getDepth(), num_depth_samples );
}