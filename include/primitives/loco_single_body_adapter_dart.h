#pragma once

#include <loco_common_dart.h>
#include <primitives/loco_single_body_collider_adapter_dart.h>
#include <primitives/loco_single_body_adapter.h>

namespace loco {
    class TSingleBody;
}

namespace loco {
namespace dartsim {

    class TDartSingleBodyAdapter : public TISingleBodyAdapter
    {
    public :

        TDartSingleBodyAdapter( TSingleBody* body_ref );

        TDartSingleBodyAdapter( const TDartSingleBodyAdapter& other ) = delete;

        TDartSingleBodyAdapter& operator=( const TDartSingleBodyAdapter& other ) = delete;

        ~TDartSingleBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void OnDetach() override;

        void SetTransform( const TMat4& transform ) override;

        void SetLinearVelocity( const TVec3& linear_vel ) override;

        void SetAngularVelocity( const TVec3& angular_vel ) override;

        void SetForceCOM( const TVec3& force_com ) override;

        void SetTorqueCOM( const TVec3& torque_com ) override;

        void GetTransform( TMat4& dst_transform ) override;

        void GetLinearVelocity( TVec3& dst_linear_vel ) override;

        void GetAngularVelocity( TVec3& dst_angular_vel ) override;

        void SetDartWorld( dart::simulation::World* world_ref );

        dart::dynamics::SkeletonPtr& skeleton() { return m_DartSkeleton; }

        const dart::dynamics::SkeletonPtr& skeleton() const { return m_DartSkeleton; }

        dart::dynamics::BodyNode* body_node() { return m_DartBodyNodeRef; }

        const dart::dynamics::BodyNode* body_node() const { return m_DartBodyNodeRef; }

        dart::dynamics::Joint* joint() { return m_DartJointRef; }

        const dart::dynamics::Joint* joint() const { return m_DartJointRef; }

    private :

        // Internal dart resource that holds articulated system (even primitives are skeletons, as dart uses minimal coordinates)
        dart::dynamics::SkeletonPtr m_DartSkeleton;
        // Reference to the dart resource that holds the information of the body in simulation (mass, inertia, com, ...)
        dart::dynamics::BodyNode* m_DartBodyNodeRef;
        // Reference to the dart resource that holds the information of the joint associated with the body above in the skeleton
        dart::dynamics::Joint* m_DartJointRef;
        // Reference to the dart-world related to the current simulation
        dart::simulation::World* m_DartWorldRef;
    };

}}