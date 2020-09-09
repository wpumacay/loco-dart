#pragma once

#include <loco_common_dart.h>
#include <kinematic_trees/loco_kinematic_tree_body_adapter.h>

namespace loco {
namespace kintree {
    class TKinematicTreeBody;
}}

namespace loco {
namespace kintree {

    class TDartKinematicTreeBodyAdapter : public TIKinematicTreeBodyAdapter
    {
    public :

        TDartKinematicTreeBodyAdapter( TKinematicTreeBody* body_ref )
            : TIKinematicTreeBodyAdapter( body_ref ) {}

        ~TIKinematicTreeBodyAdapter();

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetForceCOM( const TVec3& force ) override;

        void SetTorqueCOM( const TVec3& torque ) override;

        void GetTransform( TMat4& dst_transform ) override;

        dart::dynamics::BodyNode* body_node() { return m_DartBodyNodeRef; }

        const dart::dynamics::BodyNode* body_node() const { return m_DartBodyNodeRef; }

    private :

        // Reference to the dart resource that holds the information of the body in simulation (mass, inertia, com, ...)
        dart::dynamics::BodyNode* m_DartBodyNodeRef = nullptr;
    };
}}