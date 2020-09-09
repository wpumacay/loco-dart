#pragma once

#include <loco_common_dart.h>
#include <kinematic_trees/loco_kinematic_tree_adapter.h>

namespace loco {
namespace kintree {
    class TKinematicTree;
}}

namespace loco {
namespace kintree {

    class TDartKinematicTreeAdapter : public TIKinematicTreeAdapter
    {
    public :

        TDartKinematicTreeAdapter( TKinematicTree* kintree_ref )
            : TIKinematicTreeAdapter( kintree_ref ) {}

        ~TDartKinematicTreeAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetTransform( const TMat4& tf ) override;

        void SetLinearVelocity( const TVec3& linear_vel ) override;

        void SetAngularVelocity( const TVec3& angular_vel ) override;

        void GetTransform( TMat4& dst_tf ) override;

        void GetLinearVelocity( TVec3& dst_linearl_vel ) override;

        void GetAngularVelocity( TVec3& dst_angular_vel ) override;

        dart::dynamics::SkeletonPtr& skeleton() { return m_DartSkeleton; }

        const dart::dynamics::SkeletonPtr& skeleton() const { return m_DartSkeleton; }

    private :

        // Internal dart resource that holds articulated system
        dart::dynamics::SkeletonPtr m_DartSkeleton;
    };
}}