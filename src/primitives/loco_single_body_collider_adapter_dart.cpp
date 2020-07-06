
#include <primitives/loco_single_body_collider_adapter_dart.h>

namespace loco {
namespace primitives {

    TDartSingleBodyColliderAdapter::TDartSingleBodyColliderAdapter( TSingleBodyCollider* collision_ref )
        : TISingleBodyColliderAdapter( collision_ref )
    {
        m_DartShape = nullptr;
        m_DartShapeNodeRef = nullptr;
        m_DartWorldRef = nullptr;
    }

    TDartSingleBodyColliderAdapter::~TDartSingleBodyColliderAdapter()
    {
        if ( m_ColliderRef )
            m_ColliderRef->DetachSim();
        m_ColliderRef = nullptr;

        m_DartShape = nullptr;
        m_DartShapeNodeRef = nullptr;
        m_DartWorldRef = nullptr;
    }

    void TDartSingleBodyColliderAdapter::Build()
    {
        m_DartShape = dartsim::CreateCollisionShape( m_ColliderRef->data() );
        m_DartShapeNodeRef = nullptr;
        m_DartWorldRef = nullptr;
    }

    void TDartSingleBodyColliderAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_DartWorldRef, "TDartSingleBodyColliderAdapter::Initialize >>> must have a \
                          valid reference to the dart-world object to initialize collider {0}", m_ColliderRef->name() );

        auto& collision_filter = m_DartWorldRef->getConstraintSolver()->getCollisionOption().collisionFilter;
        auto bitmask_collision_filter = dynamic_cast<dartsim::TDartBitmaskCollisionFilter*>( collision_filter.get() );
        if ( bitmask_collision_filter )
        {
            bitmask_collision_filter->setCollisionGroup( m_DartShapeNodeRef, m_ColliderRef->collisionGroup() );
            bitmask_collision_filter->setCollisionMask( m_DartShapeNodeRef, m_ColliderRef->collisionMask() );
        }
        ChangeFriction( m_ColliderRef->data().friction.x() );
    }

    void TDartSingleBodyColliderAdapter::OnDetach()
    {
        m_Detached = true;
        m_ColliderRef = nullptr;
    }

    void TDartSingleBodyColliderAdapter::ChangeSize( const TVec3& new_size )
    {
        if ( !m_DartShape )
            return;

        switch ( m_ColliderRef->shape() )
        {
            case eShapeType::BOX :
            {
                if ( auto box_shape = dynamic_cast<dart::dynamics::BoxShape*>( m_DartShape.get() ) )
                    box_shape->setSize( dartsim::vec3_to_eigen( new_size ) );
                break;
            }
            case eShapeType::SPHERE :
            {
                if ( auto sphere_shape = dynamic_cast<dart::dynamics::SphereShape*>( m_DartShape.get() ) )
                    sphere_shape->setRadius( new_size.x() );
                break;
            }
            case eShapeType::CYLINDER :
            {
                if ( auto cylinder_shape = dynamic_cast<dart::dynamics::CylinderShape*>( m_DartShape.get() ) )
                {
                    cylinder_shape->setRadius( new_size.x() );
                    cylinder_shape->setHeight( new_size.y() );
                }
                break;
            }
            case eShapeType::CAPSULE :
            {
                if ( auto capsule_shape = dynamic_cast<dart::dynamics::CapsuleShape*>( m_DartShape.get() ) )
                {
                    capsule_shape->setRadius( new_size.x() );
                    capsule_shape->setHeight( new_size.y() );
                }
                break;
            }
            case eShapeType::ELLIPSOID :
            {
                if ( auto ellipsoid_shape = dynamic_cast<dart::dynamics::EllipsoidShape*>( m_DartShape.get() ) )
                    ellipsoid_shape->setDiameters( 2.0 * dartsim::vec3_to_eigen( new_size ) );
                break;
            }
            case eShapeType::CONVEX_MESH :
            {
                if ( auto mesh_shape = dynamic_cast<dart::dynamics::ConvexHullShape*>( m_DartShape.get() ) )
                    mesh_shape->setScale( dartsim::vec3_to_eigen( new_size ) );
                break;
            }
            case eShapeType::TRIANGULAR_MESH :
            {
                if ( auto mesh_shape = dynamic_cast<dart::dynamics::TriangleMeshShape*>( m_DartShape.get() ) )
                    mesh_shape->setScale( dartsim::vec3_to_eigen( new_size ) );
                break;
            }
            case eShapeType::HEIGHTFIELD :
            {
                if ( auto hfield_shape = dynamic_cast<dart::dynamics::HeightmapShapef*>( m_DartShape.get() ) )
                {
                    const auto& hfield_data = m_ColliderRef->data().hfield_data;
                    const ssize_t num_width_samples = hfield_data.nWidthSamples;
                    const ssize_t num_depth_samples = hfield_data.nDepthSamples;
                    const float scale_x = new_size.x() / ( num_width_samples - 1 );
                    const float scale_y = new_size.y() / ( num_depth_samples - 1 );
                    const float scale_z = new_size.z();
                    hfield_shape->setScale( Eigen::Vector3f( scale_x, scale_y, scale_z ) );
                }
                break;
            }
        }
    }

    void TDartSingleBodyColliderAdapter::ChangeVertexData( const std::vector<float>& vertices, const std::vector<int>& faces )
    {
        if ( !m_DartShape )
            return;

        const aiScene* new_mesh_data = dartsim::CreateAssimpSceneFromVertexData( vertices, faces );
        if ( auto convex_hull_shape = dynamic_cast<dart::dynamics::ConvexHullShape*>( m_DartShape.get() ) )
            convex_hull_shape->setMesh( new_mesh_data );
    }

    void TDartSingleBodyColliderAdapter::ChangeElevationData( const std::vector<float>& heights )
    {
        if ( !m_DartShape )
            return;

        const ssize_t num_width_samples = m_ColliderRef->data().hfield_data.nWidthSamples;
        const ssize_t num_depth_samples = m_ColliderRef->data().hfield_data.nDepthSamples;
        const ssize_t num_total_samples = num_width_samples * num_depth_samples;
        if ( num_total_samples != heights.size() )
        {
            LOCO_CORE_WARN( "TDartSingleBodyColliderAdapter::ChangeElevationData >>> heights-data mismatch for \
                              collider {0}", m_ColliderRef->name() );
            LOCO_CORE_WARN( "\tndepth-samples       : {0}", num_width_samples );
            LOCO_CORE_WARN( "\tnwidth-samples       : {0}", num_depth_samples );
            LOCO_CORE_WARN( "\texpected buffer-size : {0}", num_total_samples );
            LOCO_CORE_WARN( "\tgiven buffer-size    : {0}", heights.size() );
            return;
        }

        if ( auto hfield_shape = dynamic_cast<dart::dynamics::HeightmapShapef*>( m_DartShape.get() ) )
            hfield_shape->setHeightField( num_width_samples, num_depth_samples, heights );
    }

    void TDartSingleBodyColliderAdapter::ChangeCollisionGroup( int collisionGroup )
    {
        if ( !m_DartWorldRef )
        {
            LOCO_CORE_ERROR( "TDartSingleBodyColliderAdapter::ChangeCollisionGroup >>> collider {0} doesn't have \
                              a handle to the dart-world to filter collisions accordingly", m_ColliderRef->name() );
            return;
        }

        // @todo: Must extend Dart functionality to allow collision-groups and collision-masks, as
        //        in bullet, mujoco, and raisim. So far, it seems only single exclusions are possible.
        LOCO_CORE_WARN( "TDartSingleBodyColliderAdapter::ChangeCollisionGroup >>> feature not supported yet" );
    }

    void TDartSingleBodyColliderAdapter::ChangeCollisionMask( int collisionMask )
    {
        if ( !m_DartWorldRef )
        {
            LOCO_CORE_ERROR( "TDartSingleBodyColliderAdapter::ChangeCollisionMask >>> collider {0} doesn't have \
                              a handle to the dart-world to filter collisions accordingly", m_ColliderRef->name() );
            return;
        }

        // @todo: Must extend Dart functionality to allow collision-groups and collision-masks, as
        //        in bullet, mujoco, and raisim. So far, it seems only single exclusions are possible.
        LOCO_CORE_WARN( "TDartSingleBodyColliderAdapter::ChangeCollisionMask >>> feature not supported yet" );
    }

    void TDartSingleBodyColliderAdapter::ChangeFriction( const TScalar& friction )
    {
        if ( !m_DartShapeNodeRef )
        {
            LOCO_CORE_ERROR( "TDartSingleBodyColliderAdapter::ChangeFriction >>> collider {0} doesn't have \
                             a handle to its corresponding dart-shape-node", m_ColliderRef->name() );
            return;
        }

        m_DartShapeNodeRef->getDynamicsAspect()->setFrictionCoeff( friction );
    }
}}