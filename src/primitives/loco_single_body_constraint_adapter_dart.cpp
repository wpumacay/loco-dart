
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
        jnt_properties.mT_ParentBodyToJoint.translation() = jnt_pivot;
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
    }

    void TDartSingleBodyRevoluteConstraintAdapter::Reset()
    {
        SetHingeAngle( 0.0f );
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
}}