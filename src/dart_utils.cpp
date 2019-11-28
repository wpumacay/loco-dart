
#include <dart_utils.h>

using namespace dart;

namespace tysoc {
namespace utils {

    Eigen::Vector3d toEigenVec3( const TVec3& vec )
    {
        return Eigen::Vector3d( vec.x, vec.y, vec.z );
    }

    Eigen::Isometry3d toEigenTransform( const TMat4& transform )
    {
        Eigen::Isometry3d _res;
        _res.setIdentity();
        
        auto _rotation = _res.rotation();
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _rotation( i, j ) = transform.get( i, j );

        _res.rotate( _rotation );
        _res.translation() = toEigenVec3( transform.getPosition() );

        return _res;
    }

    TVec3 fromEigenVec3( const Eigen::Vector3d& vec )
    {
        return TVec3( vec.x(), vec.y(), vec.z() );
    }

    TMat4 fromEigenTransform( const Eigen::Isometry3d& transform )
    {
        TMat4 _res;

        _res.setPosition( fromEigenVec3( transform.translation() ) );
        
        auto _rotation = transform.rotation();
        for ( size_t i = 0; i < 3; i++ )
            for ( size_t j = 0; j < 3; j++ )
                _res.set( i, j, _rotation( i, j ) );

        return _res;
    }

    dynamics::ShapePtr createCollisionShape( const TShapeData& data )
    {
        dynamics::Shape* _colshape = nullptr;

        if ( data.type == eShapeType::PLANE )
        {
            // create a simple plane with normal corresponding to the z-axis
            _colshape = new dynamics::PlaneShape( Eigen::Vector3d::UnitZ(), 0.0 );
        }
        else if ( data.type == eShapeType::BOX )
        {
            // seems that the box-shape accepts full width-height-depth instead of halves
            _colshape = new dynamics::BoxShape( toEigenVec3( data.size ) );
        }
        else if ( data.type == eShapeType::SPHERE )
        {
            // pass just the radius (first entry in the size vector)
            _colshape = new dynamics::SphereShape( data.size.x );
        }
        else if ( data.type == eShapeType::CYLINDER )
        {
            // pass just the radius and height, as it seems that cylinders have Z-aligned axes
            _colshape = new dynamics::CylinderShape( data.size.x, data.size.y );
        }
        else if ( data.type == eShapeType::CAPSULE )
        {
            // pass just the radius and height, as it seems that capsules have Z-aligned axes
            _colshape = new dynamics::CapsuleShape( data.size.x, data.size.y );
        }
        else if ( data.type == eShapeType::MESH )
        {
            TYSOC_CORE_ERROR( "Sorry, mesh shapes are not supported yet :(" );
        }
        else if ( data.type == eShapeType::ELLIPSOID )
        {
            TYSOC_CORE_ERROR( "Sorry, ellipsoid shapes are not supported yet :(" );
        }
        else if ( data.type == eShapeType::HFIELD )
        {
            TYSOC_CORE_ERROR( "Sorry, heightfield shapes are not suported yet :(" );
        }
        else if ( data.type == eShapeType::NONE )
        {
            // if none is given, then the user wants a dummy shape for some funky functionality
            _colshape = nullptr;
        }

        return dynamics::ShapePtr( _colshape );
    }

}}