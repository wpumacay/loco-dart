
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

    dynamics::ShapePtr createCollisionShape( const TCollisionData& colliderData )
    {
        dynamics::Shape* _colshape = nullptr;

        if ( colliderData.type == eShapeType::PLANE )
        {
            // create a simple plane with normal corresponding to the z-axis
            _colshape = new dynamics::PlaneShape( Eigen::Vector3d::UnitZ(), 0.0 );
        }
        else if ( colliderData.type == eShapeType::BOX )
        {
            // seems that the box-shape accepts full width-height-depth instead of halves
            _colshape = new dynamics::BoxShape( toEigenVec3( colliderData.size ) );
        }
        else if ( colliderData.type == eShapeType::SPHERE )
        {
            // pass just the radius (first entry in the size vector)
            _colshape = new dynamics::SphereShape( colliderData.size.x );
        }
        else if ( colliderData.type == eShapeType::CYLINDER )
        {
            // pass just the radius and height, as it seems that cylinders have Z-aligned axes
            _colshape = new dynamics::CylinderShape( colliderData.size.x, colliderData.size.y );
        }
        else if ( colliderData.type == eShapeType::CAPSULE )
        {
            // pass just the radius and height, as it seems that capsules have Z-aligned axes
            _colshape = new dynamics::CapsuleShape( colliderData.size.x, colliderData.size.y );
        }
        else if ( colliderData.type == eShapeType::MESH )
        {
            const aiScene* _meshAssimp = dynamics::MeshShape::loadMesh( colliderData.filename );
            if ( _meshAssimp )
                _colshape = new dynamics::MeshShape( toEigenVec3( colliderData.size ), _meshAssimp );
        }
        else if ( colliderData.type == eShapeType::ELLIPSOID )
        {
            TYSOC_CORE_ERROR( "Sorry, ellipsoid shapes are not supported yet :(" );
        }
        else if ( colliderData.type == eShapeType::HFIELD )
        {
            using Vector3 = Eigen::Matrix<float, 3, 1>;
            //// // create the buffer for the heightfield (needs un-normalized heights) (@todo: change heightData to unnormalized)  
            //// std::vector<float> _heightsDataUnnormalized;
            //// for ( size_t i = 0; i < colliderData.hdata.heightcolliderData.size(); i++ )
            ////     _heightsDataUnnormalized.push_back( colliderData.hdata.heightData[i] * colliderData.size.z );

            //// auto& _hfdata = colliderData.hdata;
            //// std::vector< float > _heights( _hfdata.nWidthSamples * _hfdata.nDepthSamples, 0.0f );
            //// for ( int i = 0; i < _hfdata.nWidthSamples; i++ )
            //// {
            ////     for ( int j = 0; j < _hfdata.nDepthSamples; j++ )
            ////     {
            ////         int _pindexUserBuffer = i + j * _hfdata.nWidthSamples;
            ////         int _pindexDartBuffer = j + i * _hfdata.nDepthSamples;
            ////         _heights[_pindexDartBuffer] = _hfdata.heightData[_pindexUserBuffer] * colliderData.size.z;
            ////     }
            //// }

            // create collision shape for the heightfield (scale sets the dimensions in width-depth)
            _colshape = new dynamics::HeightmapShapef();
            dynamic_cast< dynamics::HeightmapShapef* >( _colshape )->setHeightField( colliderData.hdata.nWidthSamples,
                                                                                     colliderData.hdata.nDepthSamples,
                                                                                     colliderData.hdata.heightData );
            auto _scale = Vector3( colliderData.size.x / ( colliderData.hdata.nWidthSamples - 1 ),
                                   colliderData.size.y / ( colliderData.hdata.nDepthSamples - 1 ), 
                                   colliderData.size.z );
            dynamic_cast< dynamics::HeightmapShapef* >( _colshape )->setScale( _scale );
        }
        else if ( colliderData.type == eShapeType::NONE )
        {
            // if none is given, then the user wants a dummy shape for some funky functionality
            TYSOC_CORE_WARN( "Null collision shape created, are you making dummy objects?" );
            _colshape = nullptr;
        }

        return dynamics::ShapePtr( _colshape );
    }

}}