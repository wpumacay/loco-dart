
#include <kinematic_trees/loco_kinematic_tree_collider_adapter_dart.h>

namespace loco {
namespace kintree {

    TDartKinematicTreeColliderAdapter::TDartKinematicTreeColliderAdapter( TKinematicTreeCollider* collider_ref )
        : TIKinematicTreeColliderAdapter( collider_ref )
    {
        LOCO_CORE_ASSERT( collider_ref, "TDartKinematicTreeColliderAdapter >>> adaptee (kintree-collider) "
                          "should be a valid reference (nullptr given)" );
    }

    TDartKinematicTreeColliderAdapter::~TDartKinematicTreeColliderAdapter()
    {
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
        LOCO_CORE_ASSERT( m_DartWorldRef, "TDartKinematicTreeColliderAdapter::Initialize >>> must have a "
                          "valid reference to the dart-world object to initialize collider {0}", m_ColliderRef->name() );

        auto& collision_filter = m_DartWorldRef->getConstraintSolver()->getCollisionOption().collisionFilter;
        if ( auto bitmask_collision_filter = dynamic_cast<dartsim::TDartBitmaskCollisionFilter*>( collision_filter.get() ) )
        {
            bitmask_collision_filter->setCollisionGroup( m_DartShapeNodeRef, m_ColliderRef->collisionGroup() );
            bitmask_collision_filter->setCollisionMask( m_DartShapeNodeRef, m_ColliderRef->collisionMask() );
        }
        ChangeFriction( m_ColliderRef->data().friction.x() );
    }

    void TDartKinematicTreeColliderAdapter::SetLocalTransform( const TMat4& local_tf )
    {
        
    }

    void TDartKinematicTreeColliderAdapter::ChangeSize( const TVec3& new_size )
    {
        if ( !m_DartShape )
        {
            LOCO_CORE_ERROR( "TDartKinematicTreeColliderAdapter::ChangeSize >>> dart-shape should exists "
                             "in order to update the size of the kintree-collider {0} (but got nullptr instead)", m_ColliderRef->name() );
            return;
        }

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
            default:
            {
                LOCO_CORE_WARNING( "TDartKinematicTreeColliderAdapter::ChangeSize >>> Couldn't shape size of "
                                   "kintree-collider {0}, as its shape is of type {1}", m_ColliderRef->name(), ToString( m_ColliderRef->shape() ) );
            }
        }
    }

}}