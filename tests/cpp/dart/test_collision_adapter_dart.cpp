
#include <loco.h>
#include <gtest/gtest.h>

#include <loco_simulation_dart.h>
#include <adapters/loco_collision_adapter_dart.h>

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
    std::vector<std::unique_ptr<loco::TCollision>> vec_colliders;
    std::vector<std::unique_ptr<loco::dartsim::TDartCollisionAdapter>> vec_colliders_adapters;
    for ( size_t i = 0; i < vec_col_data.size(); i++ )
    {
        const auto collider_name = loco::ToString( vec_col_data[i].type ) + "_collider";
        auto col_obj = std::make_unique<loco::TCollision>( collider_name, vec_col_data[i] );
        auto col_adapter = std::make_unique<loco::dartsim::TDartCollisionAdapter>( col_obj.get() );
        col_adapter->Build();
        ASSERT_TRUE( col_adapter->collision_shape() != nullptr );
        vec_colliders.push_back( std::move( col_obj ) );
        vec_colliders_adapters.push_back( std::move( col_adapter ) );
    }

    std::vector<std::string> vec_expected_types = { "BoxShape",
                                                    "SphereShape",
                                                    "PlaneShape",
                                                    "CylinderShape",
                                                    "CapsuleShape",
                                                    "EllipsoidShape" };
    for ( size_t i = 0; i < vec_colliders_adapters.size(); i++ )
    {
        auto dart_collision_shape = vec_colliders_adapters[i]->collision_shape();
        EXPECT_EQ( dart_collision_shape->getType(), vec_expected_types[i] );
        if ( auto box_shape = dynamic_cast<dart::dynamics::BoxShape*>( dart_collision_shape.get() ) )
        {
            const Eigen::Vector3d size = box_shape->getSize();
            const loco::TVec3 expected_size = { 0.1f, 0.2f, 0.3f };
            EXPECT_TRUE( allclose_vec3( size, expected_size ) );
        }
        else if ( auto sphere_shape = dynamic_cast<dart::dynamics::SphereShape*>( dart_collision_shape.get() ) )
        {
            const double radius = sphere_shape->getRadius();
            const loco::TScalar expected_radius = 0.1;
            EXPECT_TRUE( std::abs( radius - expected_radius ) < 1e-5 );
        }
        else if ( auto cylinder_shape = dynamic_cast<dart::dynamics::CylinderShape*>( dart_collision_shape.get() ) )
        {
            const double radius = cylinder_shape->getRadius();
            const double height = cylinder_shape->getHeight();
            const loco::TScalar expected_radius = 0.2;
            const loco::TScalar expected_height = 0.8;
            EXPECT_TRUE( std::abs( radius - expected_radius ) < 1e-5 );
            EXPECT_TRUE( std::abs( height - expected_height ) < 1e-5 );
        }
        else if ( auto capsule_shape = dynamic_cast<dart::dynamics::CapsuleShape*>( dart_collision_shape.get() ) )
        {
            const double radius = capsule_shape->getRadius();
            const double height = capsule_shape->getHeight();
            const loco::TScalar expected_radius = 0.2;
            const loco::TScalar expected_height = 0.8;
            EXPECT_TRUE( std::abs( radius - expected_radius ) < 1e-5 );
            EXPECT_TRUE( std::abs( height - expected_height ) < 1e-5 );
        }
    }
}