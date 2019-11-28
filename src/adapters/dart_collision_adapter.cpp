
#include <adapters/dart_collision_adapter.h>

using namespace dart;

namespace tysoc {

    TDartCollisionAdapter::TDartCollisionAdapter( TCollision* collision )
        : TICollisionAdapter( collision )
    {
        m_dartShapePtr = nullptr;
        m_dartShapeNodePtr = nullptr;
    }

    TDartCollisionAdapter::~TDartCollisionAdapter()
    {
        m_dartShapePtr = nullptr;
        m_dartShapeNodePtr = nullptr;
    }

    void TDartCollisionAdapter::build()
    {
        // create collider according to user-data
        m_dartShapePtr = utils::createCollisionShape( m_collisionPtr->data() );
        // keep shape type
        m_type = m_collisionPtr->data().type;
    }

    void TDartCollisionAdapter::reset()
    {
        // nothing required for now, as the functionality exposed in the other methods seems enough
    }

    void TDartCollisionAdapter::update()
    {
        // do nothing for now, because so far we only need to use the overriden methods
    }

    void TDartCollisionAdapter::setLocalPosition( const TVec3& position )
    {
        if ( !m_dartShapePtr || !m_dartShapeNodePtr )
        {
            TYSOC_CORE_ERROR( "Dart collision-adapter >> null dart resources while trying to set local position" );
            return;
        }

        TYSOC_CORE_WARN( "Dart collision-adapter >> composite shapes not supported yet, so local-position is not required" );
    }

    void TDartCollisionAdapter::setLocalRotation( const TMat3& rotation )
    {
        if ( !m_dartShapePtr || !m_dartShapeNodePtr )
        {
            TYSOC_CORE_ERROR( "Dart collision-adapter >> null dart resources while trying to set local rotation" );
            return;
        }

        TYSOC_CORE_WARN( "Dart collision-adapter >> composite shapes not supported yet, so loca-rotation is not required" );
    }

    void TDartCollisionAdapter::setLocalTransform( const TMat4& transform )
    {
        if ( !m_dartShapePtr || !m_dartShapeNodePtr )
        {
            TYSOC_CORE_ERROR( "Dart collision-adapter >> null dart resources while trying to set local transform" );
            return;
        }

        TYSOC_CORE_WARN( "Dart collision-adapter >> composite shapes not supported yet, so loca-transform is not required" );
    }

    void TDartCollisionAdapter::changeSize( const TVec3& newSize )
    {
        if ( m_type == eShapeType::PLANE )
        {
            TYSOC_CORE_WARN( "Dart collision-adapter >> Plane shapes are infinite in this backend, so no changes can be made" );
        }
        else if ( m_type == eShapeType::BOX )
        {
            dynamic_cast< dynamics::BoxShape* >( m_dartShapePtr.get() )->setSize( utils::toEigenVec3( newSize ) );
        }
        else if ( m_type == eShapeType::SPHERE )
        {
            dynamic_cast< dynamics::SphereShape* >( m_dartShapePtr.get() )->setRadius( newSize.x );
        }
        else if ( m_type == eShapeType::CYLINDER )
        {
            dynamic_cast< dynamics::CylinderShape* >( m_dartShapePtr.get() )->setRadius( newSize.x );
            dynamic_cast< dynamics::CylinderShape* >( m_dartShapePtr.get() )->setHeight( newSize.y );
        }
        else if ( m_type == eShapeType::CAPSULE )
        {
            dynamic_cast< dynamics::CapsuleShape* >( m_dartShapePtr.get() )->setRadius( newSize.x );
            dynamic_cast< dynamics::CapsuleShape* >( m_dartShapePtr.get() )->setHeight( newSize.y );
        }
        else if ( m_type == eShapeType::ELLIPSOID )
        {
            dynamic_cast< dynamics::EllipsoidShape* >( m_dartShapePtr.get() )->setDiameters( utils::toEigenVec3( newSize ) );
        }
        else if ( m_type == eShapeType::MESH )
        {
            TYSOC_CORE_WARN( "Dart collision-adapter >> Mesh shapes don't support to change the size at runtime in this backend" );
        }
        else if ( m_type == eShapeType::HFIELD )
        {
            TYSOC_CORE_WARN( "Dart collision-adapter >> Heightfield shapes don't support to change the size at runtime. Use changeElevationData instead" );
        }
        else
        {
            TYSOC_CORE_ERROR( "Dart collision-adapter >> Tried changing size of an unsupported shape" );
        }
    }

    void TDartCollisionAdapter::changeElevationData( const std::vector< float >& heightData )
    {
        // @todo: wip, adding support for hfields
    }

}