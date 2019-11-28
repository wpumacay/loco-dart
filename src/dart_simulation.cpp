
#include <dart_simulation.h>

using namespace dart;

namespace tysoc 
{

    TDartSimulation::TDartSimulation( TScenario* scenarioPtr )
        : TISimulation( scenarioPtr )
    {
        m_dartWorld = simulation::World::create();
        m_dartWorld->getConstraintSolver()->setCollisionDetector(
                            collision::BulletCollisionDetector::create() );
        // internal timestep (finner detail), repeated n-times if required 60fps
        m_dartWorld->setTimeStep( 0.001 );

        /* create required adapters from the resources in the scenario */
        _createBodyAdapters();
        _createAgentAdapters();
        _createTerrainAdapters();
    }

    TDartSimulation::~TDartSimulation()
    {
        m_dartWorld = nullptr;
    }

    void TDartSimulation::_createBodyAdapters()
    {
        auto _bodies = m_scenarioPtr->getBodies();
        for ( auto _body : _bodies )
        {
            auto _bodyAdapter = new TDartBodyAdapter( _body );
            _body->setAdapter( _bodyAdapter );
            m_bodyAdapters.push_back( _bodyAdapter );

            auto _colliders = _body->collisions();
            for ( auto _collider : _colliders )
            {
                auto _colliderAdapter = new TDartCollisionAdapter( _collider );
                _collider->setAdapter( _colliderAdapter );
                m_collisionAdapters.push_back( _colliderAdapter );
            }
        }
    }

    void TDartSimulation::_createAgentAdapters()
    {
        // @todo: wip - implement required adapters
    }

    void TDartSimulation::_createTerrainAdapters()
    {
        // @todo: wip - implement required adapters
    }

    bool TDartSimulation::_initializeInternal()
    {

        return true;
    }

    void TDartSimulation::_preStepInternal()
    {

    }

    void TDartSimulation::_simStepInternal()
    {
        auto _tstart = m_dartWorld->getTime();
        while ( m_dartWorld->getTime() - _tstart < 1. / 60. )
            m_dartWorld->step();
    }

    void TDartSimulation::_postStepInternal()
    {

    }

    void TDartSimulation::_resetInternal()
    {

    }

}