#pragma once

#include <dart_common.h>
#include <dart_utils.h>

#include <adapters/collision_adapter.h>

using namespace dart;

namespace tysoc {

    class TCollision;

    class TDartCollisionAdapter : public TICollisionAdapter
    {

    public :

        TDartCollisionAdapter( TCollision* collision );

        ~TDartCollisionAdapter();

        void build() override;

        void reset() override;

        void update() override;

        void setLocalPosition( const TVec3& position ) override;

        void setLocalRotation( const TMat3& rotation ) override;

        void setLocalTransform( const TMat4& transform ) override;

        void changeSize( const TVec3& newSize ) override;

        void changeElevationData( const std::vector< float >& heightData ) override;

        void setShapeNode( dynamics::ShapeNodePtr shapeNode ) { m_dartShapeNodePtr = shapeNode; }

        dynamics::ShapePtr collisionShape() const { return m_dartShapePtr; }

        dynamics::ShapeNodePtr collisionShapeNode() const { return m_dartShapeNodePtr; }

    private :

        /* internal dart resource for collider data (dims, type, ...) */
        dynamics::ShapePtr m_dartShapePtr;

        /* internal dart resource for collider information in simuulation (friction, ...) */
        dynamics::ShapeNodePtr m_dartShapeNodePtr;

        /* type of shape that this drawable represents */
        eShapeType  m_type;
    };

}