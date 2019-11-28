
#include <adapters/dart_body_adapter.h>

using namespace dart;

namespace tysoc {

    TDartBodyAdapter::TDartBodyAdapter( TBody* bodyPtr )
        : TIBodyAdapter( bodyPtr )
    {

    }

    TDartBodyAdapter::~TDartBodyAdapter()
    {

    }

    void TDartBodyAdapter::build()
    {

    }

    void TDartBodyAdapter::reset()
    {

    }

    void TDartBodyAdapter::update()
    {

    }

    void TDartBodyAdapter::setPosition( const TVec3& position )
    {

    }

    void TDartBodyAdapter::setRotation( const TMat3& rotation )
    {

    }

    void TDartBodyAdapter::setTransform( const TMat4& transform )
    {

    }

    void TDartBodyAdapter::getPosition( TVec3& dstPosition )
    {

    }

    void TDartBodyAdapter::getRotation( TMat3& dstRotation )
    {

    }

    void TDartBodyAdapter::getTransform( TMat4& dstTransform )
    {

    }

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr )
    {
        return new TDartBodyAdapter( bodyPtr );
    }

}