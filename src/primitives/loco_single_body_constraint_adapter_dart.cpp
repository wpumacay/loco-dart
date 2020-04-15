
#include <primitives/loco_single_body_constraint_adapter_dart.h>

namespace loco {
namespace dartsim {

    //********************************************************************************************//
    //                              Dart-Adapter Interface Impl.                                //
    //********************************************************************************************//

    TIDartSingleBodyConstraintAdapter::TIDartSingleBodyConstraintAdapter()
    {
        m_DartSkeletonRef = nullptr;
        m_DartJointRef = nullptr;
        m_DartBodyNodeRef = nullptr;
    }

    TIDartSingleBodyConstraintAdapter::~TIDartSingleBodyConstraintAdapter()
    {
        m_DartSkeletonRef = nullptr;
        m_DartJointRef = nullptr;
        m_DartBodyNodeRef = nullptr;
    }

    //********************************************************************************************//
    //                              Revolute-constraint Adapter Impl                              //
    //********************************************************************************************//

    void TDartSingleBodyRevoluteConstraintAdapter::Build()
    {
        auto revolute_constraint = dynamic_cast<TSingleBodyRevoluteConstraint*>( m_ConstraintRef );
        LOCO_CORE_ASSERT( revolute_constraint, "TDartSingleBodyRevoluteConstraintAdapter::Build >>> \
                          constraint reference must be of type \"Revolute\", for constraint named {0}", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_DartSkeletonRef, "TDartSingleBodyRevoluteConstraintAdapter::Build >>> \
                          dart-skeleton reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );

        const std::string body_name = m_ConstraintRef->parent()->name();
        const std::string joint_name = m_ConstraintRef->name();

        const Eigen::Vector3d jnt_pivot = vec3_to_eigen( TVec3( revolute_constraint->local_tf().col( 3 ) ) );
        const Eigen::Vector3d jnt_axis = vec3_to_eigen( TMat3( revolute_constraint->local_tf() ) * revolute_constraint->axis() );

        dart::dynamics::RevoluteJoint::Properties jnt_properties;
        jnt_properties.mName = joint_name;
        jnt_properties.mAxis = jnt_axis;
        jnt_properties.mT_ChildBodyToJoint.translation() = jnt_pivot;
        jnt_properties.mRestPositions[0] = 0.0;
        jnt_properties.mSpringStiffnesses[0] = 0.0;
        jnt_properties.mDampingCoefficients[0] = 0.0;

        auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
                                                        nullptr, jnt_properties, dart::dynamics::BodyNode::AspectProperties( body_name ) );

        m_DartJointRef = joint_bodynode_pair.first;
        m_DartBodyNodeRef = joint_bodynode_pair.second;
        m_DartJointRef->setPositionLimitEnforced( false );
    }

    void TDartSingleBodyRevoluteConstraintAdapter::Initialize()
    {
        SetHingeAngle( 0.0f );
        _SetHingeSpeed( 0.0f );
    }

    void TDartSingleBodyRevoluteConstraintAdapter::Reset()
    {
        SetHingeAngle( 0.0f );
        _SetHingeSpeed( 0.0f );
    }

    void TDartSingleBodyRevoluteConstraintAdapter::SetHingeAngle( TScalar hinge_angle )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyRevoluteConstraintAdapter::SetHingeAngle >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setPosition( 0, hinge_angle );
    }

    void TDartSingleBodyRevoluteConstraintAdapter::SetLimits( const TVec2& limits )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyRevoluteConstraintAdapter::SetLimits >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );

        const bool is_limited = ( limits.x() < limits.y() );
        m_DartJointRef->setPositionLimitEnforced( is_limited );
        if ( is_limited )
        {
            m_DartJointRef->setPositionLowerLimit( 0, limits.x() );
            m_DartJointRef->setPositionUpperLimit( 0, limits.y() );
        }
    }

    void TDartSingleBodyRevoluteConstraintAdapter::GetHingeAngle( TScalar& dst_hinge_angle )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyRevoluteConstraintAdapter::GetHingeAngle >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        dst_hinge_angle = m_DartJointRef->getPosition( 0 );
    }

    void TDartSingleBodyRevoluteConstraintAdapter::_SetHingeSpeed( TScalar hinge_speed )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyRevoluteConstraintAdapter::_SetHingeSpeed >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setVelocity( 0, hinge_speed );
    }

    //********************************************************************************************//
    //                              Prismatic-constraint Adapter Impl                             //
    //********************************************************************************************//

    void TDartSingleBodyPrismaticConstraintAdapter::Build()
    {
        auto prismatic_constraint = dynamic_cast<TSingleBodyPrismaticConstraint*>( m_ConstraintRef );
        LOCO_CORE_ASSERT( prismatic_constraint, "TDartSingleBodyPrismaticConstraintAdapter::Build >>> \
                          constraint reference must be of type \"Prismatic\", for constraint named {0}", m_ConstraintRef->name() );
        LOCO_CORE_ASSERT( m_DartSkeletonRef, "TDartSingleBodyPrismaticConstraintAdapter::Build >>> \
                          dart-skeleton reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );

        const std::string body_name = m_ConstraintRef->parent()->name();
        const std::string joint_name = m_ConstraintRef->name();

        const Eigen::Vector3d jnt_pivot = vec3_to_eigen( TVec3( prismatic_constraint->local_tf().col( 3 ) ) );
        const Eigen::Vector3d jnt_axis = vec3_to_eigen( TMat3( prismatic_constraint->local_tf() ) * prismatic_constraint->axis() );

        dart::dynamics::PrismaticJoint::Properties jnt_properties;
        jnt_properties.mName = joint_name;
        jnt_properties.mAxis = jnt_axis;
        jnt_properties.mT_ChildBodyToJoint.translation() = jnt_pivot;
        jnt_properties.mRestPositions[0] = 0.0;
        jnt_properties.mSpringStiffnesses[0] = 0.0;
        jnt_properties.mDampingCoefficients[0] = 0.0;

        auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>(
                                                        nullptr, jnt_properties, dart::dynamics::BodyNode::AspectProperties( body_name ) );

        m_DartJointRef = joint_bodynode_pair.first;
        m_DartBodyNodeRef = joint_bodynode_pair.second;
        m_DartJointRef->setPositionLimitEnforced( false );
    }

    void TDartSingleBodyPrismaticConstraintAdapter::Initialize()
    {
        SetSlidePosition( 0.0f );
        _SetSlideSpeed( 0.0f );
    }

    void TDartSingleBodyPrismaticConstraintAdapter::Reset()
    {
        SetSlidePosition( 0.0f );
        _SetSlideSpeed( 0.0f );
    }

    void TDartSingleBodyPrismaticConstraintAdapter::SetSlidePosition( TScalar slide_position )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyPrismaticConstraintAdapter::SetSlidePosition >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setPosition( 0, slide_position );
    }

    void TDartSingleBodyPrismaticConstraintAdapter::SetLimits( const TVec2& limits )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyPrismaticConstraintAdapter::SetLimits >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );

        const bool is_limited = ( limits.x() < limits.y() );
        m_DartJointRef->setPositionLimitEnforced( is_limited );
        if ( is_limited )
        {
            m_DartJointRef->setPositionLowerLimit( 0, limits.x() );
            m_DartJointRef->setPositionUpperLimit( 0, limits.y() );
        }
    }

    void TDartSingleBodyPrismaticConstraintAdapter::GetSlidePosition( TScalar& dst_slide_position )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyPrismaticConstraintAdapter::GetSlidePosition >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        dst_slide_position = m_DartJointRef->getPosition( 0 );
    }

    void TDartSingleBodyPrismaticConstraintAdapter::_SetSlideSpeed( TScalar slide_speed )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyPrismaticConstraintAdapter::_SetSlideSpeed >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setVelocity( 0, slide_speed );
    }

    //********************************************************************************************//
    //                              Spherical-constraint Adapter Impl                             //
    //********************************************************************************************//

    void TDartSingleBodySphericalConstraintAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_DartSkeletonRef, "TDartSingleBodySphericalConstraintAdapter::Build >>> \
                          dart-skeleton reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );

        const std::string body_name = m_ConstraintRef->parent()->name();
        const std::string joint_name = m_ConstraintRef->name();

        const Eigen::Vector3d jnt_pivot = vec3_to_eigen( TVec3( m_ConstraintRef->local_tf().col( 3 ) ) );

        dart::dynamics::BallJoint::Properties jnt_properties;
        jnt_properties.mName = joint_name;
        jnt_properties.mT_ChildBodyToJoint.translation() = jnt_pivot;

        auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::BallJoint>(
                                                        nullptr, jnt_properties, dart::dynamics::BodyNode::AspectProperties( body_name ) );

        m_DartJointRef = joint_bodynode_pair.first;
        m_DartBodyNodeRef = joint_bodynode_pair.second;
    }

    void TDartSingleBodySphericalConstraintAdapter::Initialize()
    {
        _SetBallPosition( { 0.0f, 0.0f, 0.0f } );
        _SetBallSpeed( { 0.0f, 0.0f, 0.0f } );
    }

    void TDartSingleBodySphericalConstraintAdapter::Reset()
    {
        _SetBallPosition( { 0.0f, 0.0f, 0.0f } );
        _SetBallSpeed( { 0.0f, 0.0f, 0.0f } );
    }

    void TDartSingleBodySphericalConstraintAdapter::_SetBallPosition( const TVec3& rxyz_position )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodySphericalConstraintAdapter::_SetBallPosition >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setPosition( 0, rxyz_position.x() );
        m_DartJointRef->setPosition( 1, rxyz_position.y() );
        m_DartJointRef->setPosition( 2, rxyz_position.z() );
    }

    void TDartSingleBodySphericalConstraintAdapter::_SetBallSpeed( const TVec3& rxyz_speed )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodySphericalConstraintAdapter::_SetBallSpeed >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setVelocity( 0, rxyz_speed.x() );
        m_DartJointRef->setVelocity( 1, rxyz_speed.y() );
        m_DartJointRef->setVelocity( 2, rxyz_speed.z() );
    }

    //********************************************************************************************//
    //                            Translational-constraint Adapter Impl                           //
    //********************************************************************************************//

    void TDartSingleBodyTranslational3dConstraintAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_DartSkeletonRef, "TDartSingleBodyTranslational3dConstraintAdapter::Build >>> \
                          dart-skeleton reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );

        const std::string body_name = m_ConstraintRef->parent()->name();
        const std::string joint_name = m_ConstraintRef->name();

        dart::dynamics::TranslationalJoint::Properties jnt_properties;
        jnt_properties.mName = joint_name;

        auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint>(
                                                        nullptr, jnt_properties, dart::dynamics::BodyNode::AspectProperties( body_name ) );

        m_DartJointRef = joint_bodynode_pair.first;
        m_DartBodyNodeRef = joint_bodynode_pair.second;
    }

    void TDartSingleBodyTranslational3dConstraintAdapter::Initialize()
    {
        _SetGeneralizedPosition( { 0.0f, 0.0f, 0.0f } );
        _SetGeneralizedSpeed( { 0.0f, 0.0f, 0.0f } );
    }

    void TDartSingleBodyTranslational3dConstraintAdapter::Reset()
    {
        _SetGeneralizedPosition( { 0.0f, 0.0f, 0.0f } );
        _SetGeneralizedSpeed( { 0.0f, 0.0f, 0.0f } );
    }

    void TDartSingleBodyTranslational3dConstraintAdapter::_SetGeneralizedPosition( const TVec3& txyz_position )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyTranslational3dConstraintAdapter::_SetBallPosition >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setPosition( 0, txyz_position.x() );
        m_DartJointRef->setPosition( 1, txyz_position.y() );
        m_DartJointRef->setPosition( 2, txyz_position.z() );
    }

    void TDartSingleBodyTranslational3dConstraintAdapter::_SetGeneralizedSpeed( const TVec3& txyz_speed )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyTranslational3dConstraintAdapter::_SetBallSpeed >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setVelocity( 0, txyz_speed.x() );
        m_DartJointRef->setVelocity( 1, txyz_speed.y() );
        m_DartJointRef->setVelocity( 2, txyz_speed.z() );
    }

    //********************************************************************************************//
    //                            Translational-constraint Adapter Impl                           //
    //********************************************************************************************//

    void TDartSingleBodyPlanarConstraintAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_DartSkeletonRef, "TDartSingleBodyPlanarConstraintAdapter::Build >>> \
                          dart-skeleton reference must be provided before calling ->Build(), for constraint named {0}", m_ConstraintRef->name() );

        const std::string body_name = m_ConstraintRef->parent()->name();
        const std::string joint_name = m_ConstraintRef->name();

        dart::dynamics::PlanarJoint::Properties jnt_properties;
        jnt_properties.mPlaneType = dart::dynamics::detail::PlaneType::ZX;
        jnt_properties.mName = joint_name;

        auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::PlanarJoint>(
                                                        nullptr, jnt_properties, dart::dynamics::BodyNode::AspectProperties( body_name ) );

        m_DartJointRef = joint_bodynode_pair.first;
        m_DartBodyNodeRef = joint_bodynode_pair.second;
    }

    void TDartSingleBodyPlanarConstraintAdapter::Initialize()
    {
        _SetGeneralizedPosition( { 0.0f, 0.0f, 0.0f } );
        _SetGeneralizedSpeed( { 0.0f, 0.0f, 0.0f } );
    }

    void TDartSingleBodyPlanarConstraintAdapter::Reset()
    {
        _SetGeneralizedPosition( { 0.0f, 0.0f, 0.0f } );
        _SetGeneralizedSpeed( { 0.0f, 0.0f, 0.0f } );
    }

    void TDartSingleBodyPlanarConstraintAdapter::_SetGeneralizedPosition( const TVec3& txz_ry_position )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyPlanarConstraintAdapter::_SetBallPosition >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setPosition( 0, txz_ry_position.x() ); // trans-x is first dof
        m_DartJointRef->setPosition( 1, txz_ry_position.z() ); // trans-z is second dof
        m_DartJointRef->setPosition( 2, txz_ry_position.y() ); // rot-y is third dof
    }

    void TDartSingleBodyPlanarConstraintAdapter::_SetGeneralizedSpeed( const TVec3& txz_ry_speed )
    {
        LOCO_CORE_ASSERT( m_DartJointRef, "TDartSingleBodyPlanarConstraintAdapter::_SetBallSpeed >>> \
                          dart-joint reference must be valid (got nullptr), for constraint named \"{0}\"", m_ConstraintRef->name() );
        m_DartJointRef->setVelocity( 0, txz_ry_speed.x() ); // trans-x is first dof
        m_DartJointRef->setVelocity( 1, txz_ry_speed.z() ); // trans-z is second dof
        m_DartJointRef->setVelocity( 2, txz_ry_speed.y() ); // rot-y is third dof
    }
}}