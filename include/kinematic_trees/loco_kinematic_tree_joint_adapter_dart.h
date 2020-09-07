#pragma once

#include <loco_common_dart.h>
#include <kinematic_trees/loco_kinematic_tree_joint_adapter.h>

namespace loco {
namespace kintree {
    class TKinematicTreeJoint;
}}

namespace loco {
namespace kintree {

    class TDartKinematicTreeJointAdapter : public TIKinematicTreeJointAdapter
    {
    public :

        TDartKinematicTreeJointAdapter( TKinematicTreeJoint* joint_ref );

        TDartKinematicTreeJointAdapter( const TDartKinematicTreeJointAdapter& other ) = delete;

        TDartKinematicTreeJointAdapter& operator=( const TDartKinematicTreeJointAdapter& other ) = delete;

        ~TDartKinematicTreeJointAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetQpos( const std::vector<TScalar>& qpos ) override;

        void SetQvel( const std::vector<TScalar>& qvel ) override;

        void SetLocalTransform( const TMat4& local_tf ) override;

        void ChangeStiffness( const TScalar& stiffness ) override;

        void ChangeArmature( const TScalar& armature ) override;

        void ChangeDamping( const TScalar& damping ) override;

        void ChangeAxis( const TVec3& axis ) override;

        void ChangeLimits( const TVec2& limits ) override;

        void GetQpos( std::vector<TScalar>& dst_qpos ) override;

        void GetQvel( std::vector<TScalar>& dst_qvel ) override;

        void SetDartSkeleton( dart::dynamics::Skeleton* skeleton_ref ) { m_DartSkeletonRef = skeleton_ref; }

        void SetDartParentsParentBody( dart::dynamics::BodyNode* body_node_ref ) {  }

        dart::dynamics::Skeleton* skeleton() { return m_DartSkeletonRef; }

        const dart::dynamics::Skeleton* skeleton() const { return m_DartSkeletonRef; }

        dart::dynamics::Joint* joint() { return m_DartJointRef; }

        const dart::dynamics::Joint* joint() const { return m_DartJointRef; }

        dart::dynamics::BodyNode* body_node() { return m_DartBodyNodeRef; }

        const dart::dynamics::BodyNode* body_node() const { return m_DartBodyNodeRef; }

    private :

        /// Reference to skeleton-ptr (to create joint|body-node aspect)
        dart::dynamics::Skeleton* m_DartSkeletonRef = nullptr;
        /// Reference to joint-ptr created for this constraint
        dart::dynamics::Joint* m_DartJointRef = nullptr;
        /// Reference to bodynode-ptr created during joint creation (related to its parent body)
        dart::dynamics::BodyNode* m_DartBodyNodeRef = nullptr;
        /// Reference to bodynode-ptr created during joint creation (related to its parent's parent body)
        dart::dynamics::BodyNode* m_DartParentsParentBodyNodeRef = nullptr;
    };
}}