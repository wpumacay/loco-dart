#pragma once

#include <loco_common_dart.h>
#include <primitives/loco_single_body_collider_adapter.h>

namespace loco {
namespace primitives {

    const float LOCO_DART_HFIELD_BASE = 1.0f;

    class TDartSingleBodyColliderAdapter : public TISingleBodyColliderAdapter
    {
    public :

        TDartSingleBodyColliderAdapter( TSingleBodyCollider* collision_ref );

        TDartSingleBodyColliderAdapter( const TDartSingleBodyColliderAdapter& other ) = delete;

        TDartSingleBodyColliderAdapter& operator=( const TDartSingleBodyColliderAdapter& other ) = delete;

        ~TDartSingleBodyColliderAdapter();

        void Build() override;

        void Initialize() override;

        void OnDetach() override;

        void ChangeSize( const TVec3& new_size ) override;

        void ChangeVertexData( const std::vector<float>& vertices, const std::vector<int>& faces );

        void ChangeElevationData( const std::vector<float>& heights ) override;

        void ChangeCollisionGroup( int collisionGroup ) override;

        void ChangeCollisionMask( int collisionMask ) override;

        void ChangeFriction( const TScalar& friction ) override;

        void SetDartShapeNode( dart::dynamics::ShapeNode* shape_node_ref ) { m_DartShapeNodeRef = shape_node_ref; }

        void SetDartWorld( dart::simulation::World* world_ref ) { m_DartWorldRef = world_ref; }

        dart::dynamics::ShapePtr& collision_shape() { return m_DartShape; }

        const dart::dynamics::ShapePtr& collision_shape() const { return m_DartShape; }

    private :

        // Owned internal dart resource for collider data (dims, type, ...)
        dart::dynamics::ShapePtr m_DartShape;
        // Reference to the internal dart resource for collider information in simulation (friction, ...)
        dart::dynamics::ShapeNode* m_DartShapeNodeRef;
        // Reference to the internal dart world
        dart::simulation::World* m_DartWorldRef;
    };
}}