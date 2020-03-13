
#include <adapters/loco_single_body_adapter_dart.h>

namespace loco {
namespace dartsim {

    TDartSingleBodyAdapter::TDartSingleBodyAdapter( TIBody* body_ref )
        : TIBodyAdapter( body_ref )
    {
        LOCO_CORE_ASSERT( body_ref, "TDartSingleBodyAdapter >>> expected non-null body-obj reference" );

        m_DartSkeleton = nullptr;
        m_DartBodyNodeRef = nullptr;
        m_DartJointRef = nullptr;
        m_DartWorldRef = nullptr;
    }

    TDartSingleBodyAdapter::~TDartSingleBodyAdapter()
    {
        m_DartSkeleton = nullptr;
        m_DartBodyNodeRef = nullptr;
        m_DartJointRef = nullptr;
        m_DartWorldRef = nullptr;
    }

    void TDartSingleBodyAdapter::Build()
    {
        m_DartSkeleton = dart::dynamics::Skeleton::create( m_bodyRef->name() );
        const auto dyntype = m_bodyRef->dyntype();
        if ( dyntype == eDynamicsType::STATIC )
        {
            dart::dynamics::BodyNode::Properties body_properties;
            body_properties.mName = m_bodyRef->name();
            dart::dynamics::WeldJoint::Properties joint_properties;
            joint_properties.mName = m_bodyRef->name() + "_weldjoint";

            auto joint_bodynode_pair = m_DartSkeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
                                                            nullptr, joint_properties, body_properties );
            m_DartJointRef = joint_bodynode_pair.first;
            m_DartBodyNodeRef = joint_bodynode_pair.second;
        }
        else
        {
            dart::dynamics::BodyNode::Properties body_properties;
            body_properties.mName = m_bodyRef->name();
            dart::dynamics::FreeJoint::Properties joint_properties;
            joint_properties.mName = m_bodyRef->name() + "_freejoint";

            auto joint_bodynode_pair = m_DartSkeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
                                                            nullptr, joint_properties, body_properties );
            m_DartJointRef = joint_bodynode_pair.first;
            m_DartBodyNodeRef = joint_bodynode_pair.second;
        }

        if ( auto collider = m_bodyRef->collision() )
        {
            if ( auto collider_adapter = static_cast<TDartCollisionAdapter*>( collider->adapter() ) )
            {
                collider_adapter->Build();
                auto& dart_collision_shape = collider_adapter->collision_shape();
                auto shape_node = m_DartBodyNodeRef->createShapeNodeWith<
                                                        dart::dynamics::CollisionAspect,
                                                        dart::dynamics::DynamicsAspect>( dart_collision_shape );
                collider_adapter->SetDartShapeNode( shape_node );
                if ( dyntype == eDynamicsType::DYNAMIC )
                {
                    dart::dynamics::Inertia body_inertia;
                    const auto& inertia_data = m_bodyRef->data().inertia;

                    if ( inertia_data.mass > 0.0f )
                        body_inertia.setMass( inertia_data.mass );
                    else
                        body_inertia.setMass( dart_collision_shape->getVolume() * collider->data().density );

                    if ( ( inertia_data.ixx > 0.0f ) && ( inertia_data.iyy > 0.0f ) && ( inertia_data.izz > 0.0f ) && 
                         ( inertia_data.ixy >= 0.0f ) && ( inertia_data.ixz >= 0.0f ) && ( inertia_data.iyz >= 0.0f ) )
                        body_inertia.setMoment( inertia_data.ixx, inertia_data.iyy, inertia_data.izz,
                                                inertia_data.ixy, inertia_data.ixz, inertia_data.iyz );
                    else
                        body_inertia.setMoment( dart_collision_shape->computeInertia( body_inertia.getMass() ) );

                    m_DartBodyNodeRef->setInertia( body_inertia );
                }
            }
        }

        SetTransform( m_bodyRef->tf0() );
    }

    void TDartSingleBodyAdapter::Initialize()
    {
        LOCO_CORE_ASSERT( m_DartWorldRef, "TDartSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid dart-world reference", m_bodyRef->name() );
        LOCO_CORE_ASSERT( m_DartSkeleton, "TDartSingleBodyAdapter::Initialize >>> body {0} must have \
                          a valid dart-rigid-body. Perhaps missing call to ->Build()?", m_bodyRef->name() );

        m_DartWorldRef->addSkeleton( m_DartSkeleton );
    }

    void TDartSingleBodyAdapter::Reset()
    {
        SetTransform( m_bodyRef->tf0() );
        _SetLinearVelocity( { 0.0, 0.0, 0.0 } );
        _SetAngularVelocity( { 0.0, 0.0, 0.0 } );
    }

    void TDartSingleBodyAdapter::PreStep()
    {
        // Nothing to prepare before to a simulation step
    }

    void TDartSingleBodyAdapter::PostStep()
    {
        // Nothing to process after to a simulation step
    }

    // @todo: remove set-position and set-rotation from adapter. Use set-transform instead
    void TDartSingleBodyAdapter::SetPosition( const TVec3& position )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyAdapter::SetPosition >>> body {0} must have \
                          a valid dart-joint to set its position. Perhaps missing call to ->Build()", m_bodyRef->name() );

        auto dart_tf = mat4_to_eigen_tf( m_bodyRef->tf() );
        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
            static_cast<dart::dynamics::FreeJoint*>( m_DartJointRef )->setTransform( dart_tf );
        else
            m_DartJointRef->setTransformFromParentBodyNode( dart_tf );
    }

    // @todo: remove set-position and set-rotation from adapter. Use set-transform instead
    void TDartSingleBodyAdapter::SetRotation( const TMat3& rotation )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyAdapter::SetRotation >>> body {0} must have \
                          a valid dart-joint to set its rotation. Perhaps missing call to ->Build()", m_bodyRef->name() );

        auto dart_tf = mat4_to_eigen_tf( m_bodyRef->tf() );
        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
            static_cast<dart::dynamics::FreeJoint*>( m_DartJointRef )->setTransform( dart_tf );
        else
            m_DartJointRef->setTransformFromParentBodyNode( dart_tf );
    }

    void TDartSingleBodyAdapter::SetTransform( const TMat4& transform )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyAdapter::SetTransform >>> body {0} must have \
                          a valid dart-joint to set its transform. Perhaps missing call to ->Build()", m_bodyRef->name() );

        auto dart_tf = mat4_to_eigen_tf( transform );
        if ( m_bodyRef->dyntype() == eDynamicsType::DYNAMIC )
            static_cast<dart::dynamics::FreeJoint*>( m_DartJointRef )->setTransform( dart_tf );
        else
            m_DartJointRef->setTransformFromParentBodyNode( dart_tf );
    }

    // @todo: remove set-position and set-rotation from adapter. Use set-transform instead
    void TDartSingleBodyAdapter::GetPosition( TVec3& dstPosition )
    {
        LOCO_CORE_ASSERT( m_DartBodyNodeRef, "TDartSingleBodyAdapter::GetPosition >>> body {0} must have \
                          a valid dart-bodynode to get its position. Perhaps missing call to ->Build()", m_bodyRef->name() );

        dstPosition = vec3_from_eigen( m_DartBodyNodeRef->getTransform().translation() );
    }

    // @todo: remove set-position and set-rotation from adapter. Use set-transform instead
    void TDartSingleBodyAdapter::GetRotation( TMat3& dstRotation )
    {
        LOCO_CORE_ASSERT( m_DartBodyNodeRef, "TDartSingleBodyAdapter::GetRotation >>> body {0} must have \
                          a valid dart-bodynode to get its rotation. Perhaps missing call to ->Build()", m_bodyRef->name() );

        dstRotation = mat3_from_eigen( m_DartBodyNodeRef->getTransform().rotation() );
    }

    void TDartSingleBodyAdapter::GetTransform( TMat4& dstTransform )
    {
        LOCO_CORE_ASSERT( m_DartBodyNodeRef, "TDartSingleBodyAdapter::GetTransform >>> body {0} must have \
                          a valid dart-bodynode to get its transform. Perhaps missing call to ->Build()", m_bodyRef->name() );

        dstTransform = mat4_from_eigen_tf( m_DartBodyNodeRef->getTransform() );
    }

    // @todo: remove, as adaptee already sets m_tf0, and backends don't actually require to update internal initial configurations
    void TDartSingleBodyAdapter::SetInitialPosition( const TVec3& position )
    {}

    // @todo: remove, as adaptee already sets m_tf0, and backends don't actually require to update internal initial configurations
    void TDartSingleBodyAdapter::SetInitialRotation( const TMat3& rotation )
    {}

    // @todo: remove, as adaptee already sets m_tf0, and backends don't actually require to update internal initial configurations
    void TDartSingleBodyAdapter::SetInitialTransform( const TMat4& transform )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::SetLocalPosition( const TVec3& position )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::SetLocalRotation( const TMat3& rotation )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::SetLocalTransform( const TMat4& transform )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::GetLocalPosition( TVec3& position )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::GetLocalRotation( TMat3& rotation )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::GetLocalTransform( TMat4& transform )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::SetInitialLocalPosition( const TVec3& position )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::SetInitialLocalRotation( const TMat3& rotation )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::SetInitialLocalTransform( const TMat4& transform )
    {}

    // @todo: remove set|get locals, and make a single-body object instead (separate from compound-body|kintree-body)
    void TDartSingleBodyAdapter::_SetLinearVelocity( const TVec3& linear_vel )
    {
        if ( m_bodyRef->dyntype() != eDynamicsType::DYNAMIC )
            return;

        LOCO_CORE_ASSERT( m_DartJointRef->getNumDofs() == 6, "TDartSingleBodyAdapter:::_SetLinearVelocity >>> \
                          body {0}'s' free-joint must have 6 dofs", m_bodyRef->name() );
        m_DartJointRef->setVelocity( 3, linear_vel.x() );
        m_DartJointRef->setVelocity( 4, linear_vel.y() );
        m_DartJointRef->setVelocity( 5, linear_vel.z() );
    }

    void TDartSingleBodyAdapter::_SetAngularVelocity( const TVec3& angular_vel )
    {
        if ( !m_DartJointRef || m_bodyRef->dyntype() != eDynamicsType::DYNAMIC )
            return;

        LOCO_CORE_ASSERT( m_DartJointRef->getNumDofs() == 6, "TDartSingleBodyAdapter:::_SetAngularVelocity >>> \
                          body {0}'s' free-joint must have 6 dofs", m_bodyRef->name() );
        m_DartJointRef->setVelocity( 0, angular_vel.x() );
        m_DartJointRef->setVelocity( 1, angular_vel.y() );
        m_DartJointRef->setVelocity( 2, angular_vel.z() );
    }



}}