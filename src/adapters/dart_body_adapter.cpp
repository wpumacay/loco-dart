
#include <adapters/dart_body_adapter.h>

using namespace dart;

namespace tysoc {

    TDartBodyAdapter::TDartBodyAdapter( TBody* bodyPtr )
        : TIBodyAdapter( bodyPtr )
    {
        m_dartBodyNodePtr   = nullptr;
        m_dartJointPtr      = nullptr;
        m_dartSkeletonPtr   = nullptr;
    }

    TDartBodyAdapter::~TDartBodyAdapter()
    {
        m_dartBodyNodePtr   = nullptr;
        m_dartJointPtr      = nullptr;
        m_dartSkeletonPtr   = nullptr;
    }

    void TDartBodyAdapter::build()
    {
        if ( !m_bodyPtr )
        {
            TYSOC_CORE_ERROR( "Dart body-adapter >>> tried to construct body-adapter with null body reference" );
            return;
        }
        
        /* skeleton for this body (dart uses minimal coordinates and, as far as I've checked, it
           represents all simulation bodies and articulated systems with the same skeleton class */
        m_dartSkeletonPtr = dynamics::Skeleton::create( m_bodyPtr->name() );

        /* create joint-bodynode pair for this body (@todo: add support for multi-link systems) */
        if ( m_bodyPtr->dyntype() == eDynamicsType::STATIC )
            _createJointBodyNodePair_fixed();
        else
            _createJointBodyNodePair_free();

        /* add collider information, and grab associated shapenode */
        auto _collider = m_bodyPtr->collision();
        if ( !_collider )
        {
            TYSOC_CORE_ERROR( "Dart body-adapter >>> body {0} has no colliders", m_bodyPtr->name() );
            return;
        }

        if ( _collider->adapter() )
        {
            auto _colliderAdapter = dynamic_cast< TDartCollisionAdapter* >( _collider->adapter() );
            if ( _colliderAdapter->collisionShape() )
            {
                auto _shapeNodePtr = m_dartBodyNodePtr->createShapeNodeWith< dynamics::CollisionAspect,
                                                                             dynamics::DynamicsAspect >( _colliderAdapter->collisionShape() );
                _colliderAdapter->setShapeNode( _shapeNodePtr );

                /* compute inertial properties of the body */
                if ( m_bodyPtr->dyntype() == eDynamicsType::DYNAMIC )
                {
                    dynamics::Inertia _bnInertia;
                    if ( m_bodyPtr->data().inertialData.mass != 0.0f )
                    {
                        _bnInertia.setMass( m_bodyPtr->data().inertialData.mass );
                        _bnInertia.setMoment( m_bodyPtr->data().inertialData.ixx,
                                              m_bodyPtr->data().inertialData.iyy,
                                              m_bodyPtr->data().inertialData.izz,
                                              m_bodyPtr->data().inertialData.ixy,
                                              m_bodyPtr->data().inertialData.ixz,
                                              m_bodyPtr->data().inertialData.iyz );

                        // @todo: add support to recompute principal axes if local-transform given, and ...
                        //        to set the COM of the inertial frame w.r.t. body frame
                    }
                    else
                    {
                        auto _mass = _colliderAdapter->collisionShape()->getVolume() * _collider->data().density;
                        auto _inertia = _colliderAdapter->collisionShape()->computeInertia( _mass );
                        _bnInertia.setMass( _mass );
                        _bnInertia.setMoment( _inertia );
                    }

                    m_dartBodyNodePtr->setInertia( _bnInertia );
                }
            }
        }
        else
        {
            TYSOC_CORE_ERROR( "Dart body-adapter >>> body {0}'s main collider doesn't have a valid adapter", m_bodyPtr->name() );
        }

        /* set initial transform */
        if ( m_dartJointPtr )
        {
            auto _tf0 = utils::toEigenTransform( m_bodyPtr->tf0() );
            if ( m_bodyPtr->dyntype() == eDynamicsType::STATIC )
                m_dartJointPtr->setTransformFromParentBodyNode( _tf0 );
            else
                dynamic_cast< dynamics::FreeJoint* >( m_dartJointPtr.get() )->setTransform( _tf0 );
        }
    }

    void TDartBodyAdapter::reset()
    {
        if ( !m_dartJointPtr )
            return;

        auto _tf0 = utils::toEigenTransform( m_bodyPtr->tf0() );
        if ( m_bodyPtr->dyntype() == eDynamicsType::STATIC )
            m_dartJointPtr->setTransformFromParentBodyNode( _tf0 );
        else
            dynamic_cast< dynamics::FreeJoint* >( m_dartJointPtr.get() )->setTransform( _tf0 );

        for ( size_t i = 0; i < m_dartJointPtr->getNumDofs(); i++ )
            m_dartJointPtr->setVelocity( i, 0.0 );
    }

    void TDartBodyAdapter::update()
    {
        // do nothing for now, because so far we only need to use the overriden methods
    }

    void TDartBodyAdapter::setPosition( const TVec3& position )
    {
        if ( !m_bodyPtr || !m_dartBodyNodePtr )
            return;

        
    }

    void TDartBodyAdapter::setRotation( const TMat3& rotation )
    {

    }

    void TDartBodyAdapter::setTransform( const TMat4& transform )
    {

    }

    void TDartBodyAdapter::getPosition( TVec3& dstPosition )
    {
        if ( !m_dartBodyNodePtr )
            return;

        dstPosition = utils::fromEigenTransform( m_dartBodyNodePtr->getTransform() ).getPosition();
    }

    void TDartBodyAdapter::getRotation( TMat3& dstRotation )
    {
        if ( !m_dartBodyNodePtr )
            return;

        dstRotation = utils::fromEigenTransform( m_dartBodyNodePtr->getTransform() ).getRotation();
    }

    void TDartBodyAdapter::getTransform( TMat4& dstTransform )
    {
        if ( !m_dartBodyNodePtr )
            return;

        dstTransform = utils::fromEigenTransform( m_dartBodyNodePtr->getTransform() );
    }

    void TDartBodyAdapter::_createJointBodyNodePair_fixed()
    {
        auto _pairJointBodyNode = m_dartSkeletonPtr->createJointAndBodyNodePair< dynamics::WeldJoint >();
        m_dartJointPtr = _pairJointBodyNode.first;
        m_dartBodyNodePtr = _pairJointBodyNode.second;
    }

    void TDartBodyAdapter::_createJointBodyNodePair_free()
    {
        dynamics::BodyNode::Properties _bnProperties;
        _bnProperties.mName = m_bodyPtr->name();

        dynamics::FreeJoint::Properties _fjProperties;
        _fjProperties.mName = m_bodyPtr->name() + "_fjoint";
        _fjProperties.mDampingCoefficients[0] = 0;
        _fjProperties.mDampingCoefficients[1] = 0;
        _fjProperties.mDampingCoefficients[2] = 0;
        _fjProperties.mDampingCoefficients[3] = 0;
        _fjProperties.mDampingCoefficients[4] = 0;
        _fjProperties.mDampingCoefficients[5] = 0;

        auto _pairJointBodyNode = m_dartSkeletonPtr->createJointAndBodyNodePair< dynamics::FreeJoint >( nullptr, _fjProperties, _bnProperties );
        m_dartJointPtr = _pairJointBodyNode.first;
        m_dartBodyNodePtr = _pairJointBodyNode.second;
    }

    extern "C" TIBodyAdapter* simulation_createBodyAdapter( TBody* bodyPtr )
    {
        return new TDartBodyAdapter( bodyPtr );
    }

}