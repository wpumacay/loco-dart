#pragma once

#include <dart_common.h>
#include <dart_utils.h>

#include <adapters/body_adapter.h>
#include <adapters/dart_collision_adapter.h>

using namespace dart;

namespace tysoc {

    class TDartBodyAdapter : public TIBodyAdapter
    {

    public :

        TDartBodyAdapter( TBody* bodyPtr );

        ~TDartBodyAdapter();

        void build() override;

        void reset() override;

        void update() override;

        void setPosition( const TVec3& position ) override;

        void setRotation( const TMat3& rotation ) override;

        void setTransform( const TMat4& transform ) override;

        void getPosition( TVec3& dstPosition ) override;

        void getRotation( TMat3& dstRotation ) override;

        void getTransform( TMat4& dstTransform ) override;

    private :

    };

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr );

}