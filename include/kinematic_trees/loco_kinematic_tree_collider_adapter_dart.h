#pragma once

#include <loco_common_dart.h>
#include <kinematic_trees/loco_kinematic_tree_collider_adapter.h>

namespace loco {
namespace kintree {
    class TKinematicTreeCollider;
}}

namespace loco {
namespace kintree {

    class TDartKinematicTreeColliderAdapter : public TIKinematicTreeColliderAdapter
    {
    public :

        TDartKinematicTreeColliderAdapter( TKinematicTreeCollider* collider_ref );

        TDartKinematicTreeColliderAdapter( const TDartKinematicTreeColliderAdapter& other ) = delete;

        TDartKinematicTreeColliderAdapter& operator=( const TDartKinematicTreeColliderAdapter& other ) = delete;

        ~TDartKinematicTreeColliderAdapter();

        void Build() override;

        void Initialize() override;

        void SetLocalTransform( const TMat4& local_tf ) override;

        void ChangeSize( const TVec3& new_size ) override;

        void ChangeCollisionGroup( int collision_group ) override;

        void ChangeCollisionMask( int collision_mask ) override;

        void ChangeFriction( const TScalar& friction ) override;

        void SetDartShapeNode( dart::dynamics::ShapeNode* shape_node_ref ) { m_DartShapeNodeRef = shape_node_ref; }

        void SetDartWorld( dart::simulation::World* world_ref ) { m_DartWorldRef = world_ref; }

        dart::dynamics::ShapePtr& collision_shape() { return m_DartShape; }

        const dart::dynamics::ShapePtr& collision_shape() const { return m_DartShape; }

    private :

        // Owned internal dart resource for collider data (dims, type, ...)
        dart::dynamics::ShapePtr m_DartShape = nullptr;
        // Reference to the internal dart resource for collider information in simulation (friction, ...)
        dart::dynamics::ShapeNode* m_DartShapeNodeRef = nullptr;
        // Reference to the internal dart world
        dart::simulation::World* m_DartWorldRef = nullptr;
    };
}}