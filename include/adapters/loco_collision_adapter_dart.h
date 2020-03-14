#pragma once

#include <loco_common_dart.h>
#include <adapters/loco_collision_adapter.h>

namespace loco {
namespace dartsim {

    const float LOCO_DART_HFIELD_BASE = 1.0f;

    class TDartCollisionAdapter : public TICollisionAdapter
    {
    public :

        TDartCollisionAdapter( TCollision* collisionRef );

        TDartCollisionAdapter( const TDartCollisionAdapter& other ) = delete;

        TDartCollisionAdapter& operator=( const TDartCollisionAdapter& other ) = delete;

        ~TDartCollisionAdapter();

        void Build() override;

        void Initialize() override;

        void PreStep() override;

        void PostStep() override;

        void Reset() override;

        void SetLocalPosition( const TVec3& position ) override;

        void SetLocalRotation( const TMat3& rotation ) override;

        void SetLocalTransform( const TMat4& transform ) override;

        void ChangeSize( const TVec3& new_size ) override;

        void ChangeElevationData( const std::vector<float>& heights ) override;

        void ChangeCollisionGroup( int collisionGroup ) override;

        void ChangeCollisionMask( int collisionMask ) override;

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