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

        dynamics::SkeletonPtr skeleton() const { return m_dartSkeletonPtr; }

        dynamics::BodyNodePtr bodyNode() const { return m_dartBodyNodePtr; }

        dynamics::JointPtr joint() const { return m_dartJointPtr; }

    private :

        void _createJointBodyNodePair_fixed();
        void _createJointBodyNodePair_free();

    private :

        /* internal dart resource that holds articulated system (even primitives are skeletons, as dart uses minimal coordinates) */
        dynamics::SkeletonPtr m_dartSkeletonPtr;

        /* internal dart resource that holds the information of the body in simulation (mass, inertia, com, ...) */
        dynamics::BodyNodePtr m_dartBodyNodePtr;

        /* internal dart resource that holds the information of the joint associated with the body above in the skeleton */
        dynamics::JointPtr m_dartJointPtr;

    };

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr );

}