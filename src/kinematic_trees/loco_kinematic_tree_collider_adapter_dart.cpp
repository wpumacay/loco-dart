
#include <kinematic_trees/loco_kinematic_tree_collider_adapter_dart.h>

namespace loco {
namespace kintree {

    TDartKinematicTreeColliderAdapter::TDartKinematicTreeColliderAdapter( TKinematicTreeCollider* collider_ref )
        : TIKinematicTreeColliderAdapter( collider_ref ) {}

    TDartKinematicTreeColliderAdapter::~TDartKinematicTreeColliderAdapter()
    {
        if ( m_ColliderRef )
            m_ColliderRef->DetachSim();
        m_ColliderRef = nullptr;

        m_DartShape = nullptr;
        m_DartShapeNodeRef = nullptr;
        m_DartWorldRef = nullptr;
    }

    void TDartKinematicTreeColliderAdapter::Build()
    {
        m_DartShape = dartsim::CreateCollisionShape( m_ColliderRef->data() );
        m_DartShapeNodeRef = nullptr;
        m_DartWorldRef = nullptr;
    }

    void TDartKinematicTreeColliderAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_DartWorldRef, "TDartKinematicTreeColliderAdapter::Initialize >>> must have a \
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

    void TDartKinematicTreeColliderAdapter::ChangeSize( const TVec3& new_size )
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
            default :
            {
                LOCO_CORE_ERROR( "TDartKinematicTreeColliderAdapter::ChangeSize >>> shape {0} not supported "
                                 "by kinematic-tree-collider object {1}", ToString( m_ColliderRef->shape() ), m_ColliderRef->name() );
            }
        }
    }

    void TDartKinematicTreeColliderAdapter::ChangeCollisionGroup( int collisionGroup )
    {
        if ( !m_DartWorldRef )
        {
            LOCO_CORE_ERROR( "TDartKinematicTreeColliderAdapter::ChangeCollisionGroup >>> collider {0} doesn't have \
                              a handle to the dart-world to filter collisions accordingly", m_ColliderRef->name() );
            return;
        }

        auto& collision_filter = m_DartWorldRef->getConstraintSolver()->getCollisionOption().collisionFilter;
        if ( auto bitmask_collision_filter = dynamic_cast<dartsim::TDartBitmaskCollisionFilter*>( collision_filter.get() ) )
            bitmask_collision_filter->setCollisionGroup( m_DartShapeNodeRef, m_ColliderRef->collisionGroup() );
    }

    void TDartKinematicTreeColliderAdapter::ChangeCollisionMask( int collisionMask )
    {
        if ( !m_DartWorldRef )
        {
            LOCO_CORE_ERROR( "TDartKinematicTreeColliderAdapter::ChangeCollisionMask >>> collider {0} doesn't have \
                              a handle to the dart-world to filter collisions accordingly", m_ColliderRef->name() );
            return;
        }

        auto& collision_filter = m_DartWorldRef->getConstraintSolver()->getCollisionOption().collisionFilter;
        if ( auto bitmask_collision_filter = dynamic_cast<dartsim::TDartBitmaskCollisionFilter*>( collision_filter.get() ) )
            bitmask_collision_filter->setCollisionMask( m_DartShapeNodeRef, m_ColliderRef->collisionMask() );
    }

    void TDartKinematicTreeColliderAdapter::ChangeFriction( const TScalar& friction )
    {
        if ( !m_DartShapeNodeRef )
        {
            LOCO_CORE_ERROR( "TDartKinematicTreeColliderAdapter::ChangeFriction >>> collider {0} doesn't have \
                             a handle to its corresponding dart-shape-node", m_ColliderRef->name() );
            return;
        }

        m_DartShapeNodeRef->getDynamicsAspect()->setFrictionCoeff( friction );
    }
}}