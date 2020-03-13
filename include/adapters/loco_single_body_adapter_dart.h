#pragma once

#include <loco_common_dart.h>
#include <adapters/loco_body_adapter.h>
#include <adapters/loco_collision_adapter_dart.h>

namespace loco {
    class TIBody;
}

namespace loco {
namespace dartsim {

    class TDartSingleBodyAdapter : public TIBodyAdapter
    {
    public :

        TDartSingleBodyAdapter( TIBody* body_ref );

        TDartSingleBodyAdapter( const TDartSingleBodyAdapter& other ) = delete;

        TDartSingleBodyAdapter& operator=( const TDartSingleBodyAdapter& other ) = delete;

        ~TDartSingleBodyAdapter();

        void Build() override;

        void Initialize() override;

        void PreStep() override;

        void PostStep() override;

        void Reset() override;

        void SetPosition( const TVec3& position ) override;

        void SetRotation( const TMat3& rotation ) override;

        void SetTransform( const TMat4& transform ) override;

        void GetPosition( TVec3& dstPosition ) override;

        void GetRotation( TMat3& dstRotation ) override;

        void GetTransform( TMat4& dstTransform ) override;

        void SetInitialPosition( const TVec3& position ) override;

        void SetInitialRotation( const TMat3& rotation ) override;

        void SetInitialTransform( const TMat4& transform ) override;

        void SetLocalPosition( const TVec3& position ) override;

        void SetLocalRotation( const TMat3& rotation ) override;

        void SetLocalTransform( const TMat4& transform ) override;

        void GetLocalPosition( TVec3& position ) override;

        void GetLocalRotation( TMat3& rotation ) override;

        void GetLocalTransform( TMat4& transform ) override;

        void SetInitialLocalPosition( const TVec3& position ) override;

        void SetInitialLocalRotation( const TMat3& rotation ) override;

        void SetInitialLocalTransform( const TMat4& transform ) override;

        void SetDartWorld( dart::simulation::World* world_ref ) { m_DartWorldRef = world_ref; }

        dart::dynamics::SkeletonPtr& skeleton() { return m_DartSkeleton; }

        const dart::dynamics::SkeletonPtr& skeleton() const { return m_DartSkeleton; }

        dart::dynamics::BodyNode* body_node() { return m_DartBodyNodeRef; }

        const dart::dynamics::BodyNode* body_node() const { return m_DartBodyNodeRef; }

        dart::dynamics::Joint* joint() { return m_DartJointRef; }

        const dart::dynamics::Joint* joint() const { return m_DartJointRef; }

    private :

        void _SetLinearVelocity( const TVec3& linear_vel );

        void _SetAngularVelocity( const TVec3& angular_vel );

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