
#include <kinematic_trees/loco_kinematic_tree_joint_adapter_dart.h>

namespace loco {
namespace kintree {

    TDartKinematicTreeJointAdapter::TDartKinematicTreeJointAdapter( TKinematicTreeJoint* joint_ref )
        : TIKinematicTreeJointAdapter( joint_ref ) {}

    TDartKinematicTreeJointAdapter::~TDartKinematicTreeJointAdapter()
    {
        m_DartSkeletonRef = nullptr;
        m_DartJointRef = nullptr;
        m_DartBodyNodeRef = nullptr;
    }

    void TDartKinematicTreeJointAdapter::Build()
    {
        LOCO_CORE_ASSERT( m_DartSkeletonRef, "" );

        const auto joint_type = m_JointRef->type();
        const auto joint_name = m_JointRef->name();
        const auto joint_data = m_JointRef->data();
        const auto joint_local_tf = joint_data.local_tf;
        const auto joint_local_axis = joint_data.local_axis;

        auto body_parent = m_JointRef->parent();
        auto body_parent_parent = body_parent->parent(); // nullptr <=> root body
        const bool is_root = ( body_parent_parent == nullptr );
        const auto tf_body_parent_parent_to_joint = ( !is_root ) ? body_parent->tf() :
                        body_parent->local_tf().inverse() * joint_local_tf;

        const Eigen::Vector3d jnt_pos_local = dartsim::vec3_to_eigen( TVec3( joint_local_tf.col( 3 ) ) );
        const Eigen::Matrix3d jnt_rot_local = dartsim::mat3_to_eigen( TMat3( joint_local_tf ) );
        const Eigen::Vector3d jnt_axis_body_frame = dartsim::vec3_to_eigen( TMat3( joint_local_tf ) * joint_local_axis );
        const Eigen::Vector3d body_pos_parent_to_joint = dartsim::vec3_to_eigen( TVec3( tf_body_parent_parent_to_joint.col( 3 ) ) );
        const Eigen::Matrix3d body_rot_parent_to_joint = dartsim::mat3_to_eigen( TMat3( tf_body_parent_parent_to_joint ) );

        auto dart_body_parent_parent_bodynode = ( is_root ) ? nullptr :
                    static_cast<TKinematicTreeBodyAdapter*>( body_parent_parent->adapter() )->body_node();

        /**/ if ( joint_type == eJointType::REVOLUTE )
        {
            dart::dynamics::RevoluteJoint::Properties jnt_properties;
            jnt_properties.mName = joint_name;
            jnt_properties.mAxis = jnt_axis_body_frame;
            jnt_properties.mT_ParentBodyToJoint.linear() = body_rot_parent_to_joint;
            jnt_properties.mT_ParentBodyToJoint.translation() = body_pos_parent_to_joint;
            jnt_properties.mT_ChildBodyToJoint.linear() = jnt_rot_local;
            jnt_properties.mT_ChildBodyToJoint.translation() = jnt_pos_local;
            jnt_properties.mRestPositions[0] = 0.0;
            jnt_properties.mSpringStiffnesses[0] = joint_data.stiffness;
            jnt_properties.mDampingCoefficients[0] = joint_data.damping;

            dart::dynamics::BodyNode::Properties body_properties;
            body_properties.mName = body_parent->name();

            auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
                                            dart_body_parent_parent_bodynode, jnt_properties, body_properties );
            m_DartJointRef = joint_bodynode_pair.first;
            m_DartBodyNodeRef = joint_bodynode_pair.second;
        }
        else if ( joint_type == eJointType::PRISMATIC )
        {
            dart::dynamics::PrismaticJoint::Properties jnt_properties;
            jnt_properties.mName = joint_name;
            jnt_properties.mAxis = jnt_axis_world;
            jnt_properties.mT_ParentBodyToJoint.linear() = body_rot_parent_to_joint;
            jnt_properties.mT_ParentBodyToJoint.translation() = body_pos_parent_to_joint;
            jnt_properties.mT_ChildBodyToJoint.linear() = jnt_rot_local;
            jnt_properties.mT_ChildBodyToJoint.translation() = jnt_pos_local;
            jnt_properties.mRestPositions[0] = 0.0;
            jnt_properties.mSpringStiffnesses[0] = joint_data.stiffness;
            jnt_properties.mDampingCoefficients[0] = joint_data.damping;

            dart::dynamics::BodyNode::Properties body_properties;
            body_properties.mName = body_parent->name();

            auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>(
                                            dart_body_parent_parent_bodynode, jnt_properties, body_properties );
            m_DartJointRef = joint_bodynode_pair.first;
            m_DartBodyNodeRef = joint_bodynode_pair.second;
        }
        else if ( joint_type == eJointType::SPHERICAL )
        {
            dart::dynamics::SphericalJoint::Properties jnt_properties;
            jnt_properties.mName = joint_name;
            jnt_properties.mT_ParentBodyToJoint.linear() = body_rot_parent_to_joint;
            jnt_properties.mT_ParentBodyToJoint.translation() = body_pos_parent_to_joint;
            jnt_properties.mT_ChildBodyToJoint.linear() = jnt_rot_local;
            jnt_properties.mT_ChildBodyToJoint.translation() = jnt_pos_local;

            dart::dynamics::BodyNode::Properties body_properties;
            body_properties.mName = body_parent->name();

            auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::BallJoint>(
                                            dart_body_parent_parent_bodynode, jnt_properties, body_properties );
            m_DartJointRef = joint_bodynode_pair.first;
            m_DartBodyNodeRef = joint_bodynode_pair.second;
        }
        else if ( joint_type == eJointType::PLANAR )
        {
            dart::dynamics::PlanarJoint::Properties jnt_properties;
            jnt_properties.mName = joint_name;
            jnt_properties.mPlaneType = dart::dynamics::detail::PlaneType::ARBITRARY;
            jnt_properties.mTransAxis1 = dartsim::vec3_to_eigen( joint_data.plane_axis_1 );
            jnt_properties.mTransAxis2 = dartsim::vec3_to_eigen( joint_data.plane_axis_2 );
            jnt_properties.mT_ParentBodyToJoint.translation() = body_pos_parent_to_joint;
            jnt_properties.mT_ChildBodyToJoint.translation() = jnt_pos_local;

            dart::dynamics::BodyNode::Properties body_properties;
            body_properties.mName = body_parent->name();

            auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::PlanarJoint>(
                                            dart_body_parent_parent_bodynode, jnt_properties, body_properties );
            m_DartJointRef = joint_bodynode_pair.first;
            m_DartBodyNodeRef = joint_bodynode_pair.second;
        }
        else if ( joint_type == eJointType::FIXED )
        {
            dart::dynamics::WeldJoint::Properties jnt_properties;
            jnt_properties.mName = joint_name;
            jnt_properties.mT_ParentBodyToJoint.linear() = body_rot_parent_to_joint;
            jnt_properties.mT_ParentBodyToJoint.translation() = body_pos_parent_to_joint;
            jnt_properties.mT_ChildBodyToJoint.linear() = jnt_rot_local;
            jnt_properties.mT_ChildBodyToJoint.translation() = jnt_pos_local;

            dart::dynamics::BodyNode::Properties body_properties;
            body_properties.mName = body_parent->name();

            auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
                                            dart_body_parent_parent_bodynode, jnt_properties, body_properties );
            m_DartJointRef = joint_bodynode_pair.first;
            m_DartBodyNodeRef = joint_bodynode_pair.second;
        }
        else if ( joint_type == eJointType::FREE )
        {
            dart::dynamics::FreeJoint::Properties jnt_properties;
            jnt_properties.mName = joint_name;

            dart::dynamics::BodyNode::Properties body_properties;
            body_properties.mName = body_parent->name();

            auto joint_bodynode_pair = m_DartSkeletonRef->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
                                            dart_body_parent_parent_bodynode, jnt_properties, body_properties );
            m_DartJointRef = joint_bodynode_pair.first;
            m_DartBodyNodeRef = joint_bodynode_pair.second;
        }
        else
        {
            LOCO_CORE_ERROR( "TDartKinematicTreeJointAdapter::Build >>> invalid joint-type given to kintree-joint {0}", m_JointRef->name() );
        }
    }

    void TDartKinematicTreeJointAdapter::Initialize()
    {

    }

    void TDartKinematicTreeJointAdapter::Reset()
    {

    }

    void TDartKinematicTreeJointAdapter::SetQpos( const std::vector<TScalar>& qpos )
    {

    }

    void TDartKinematicTreeJointAdapter::SetQvel( const std::vector<TScalar>& qvel )
    {

    }

    void TDartKinematicTreeJointAdapter::SetLocalTransform( const TMat4& local_tf )
    {

    }

    void TDartKinematicTreeJointAdapter::ChangeStiffness( const TScalar& stiffness )
    {

    }

    void TDartKinematicTreeJointAdapter::ChangeArmature( const TScalar& armature )
    {

    }

    void TDartKinematicTreeJointAdapter::ChangeDamping( const TScalar& damping )
    {

    }

    void TDartKinematicTreeJointAdapter::ChangeAxis( const TVec3& axis )
    {

    }

    void TDartKinematicTreeJointAdapter::ChangeLimits( const TVec2& limits )
    {

    }

    void TDartKinematicTreeJointAdapter::GetQpos( std::vector<TScalar>& dst_qpos )
    {

    }

    void TDartKinematicTreeJointAdapter::GetQvel( std::vector<TScalar>& dst_qvel )
    {

    }
}}