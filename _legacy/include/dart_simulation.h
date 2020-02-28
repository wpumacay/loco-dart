#pragma once

#include <dart_common.h>
#include <simulation_base.h>

#include <adapters/dart_collision_adapter.h>
#include <adapters/dart_body_adapter.h>

using namespace dart;

namespace tysoc 
{

    class TDartSimulation : public TISimulation
    {

    public :

        TDartSimulation( TScenario* scenarioPtr );
        ~TDartSimulation();

    protected :

        bool _initializeInternal() override;
        void _preStepInternal() override;
        void _simStepInternal() override;
        void _postStepInternal() override;
        void _resetInternal() override;

    private : // helper methods

        void _createBodyAdapters();
        void _createAgentAdapters();
        void _createTerrainAdapters();

    private : // backend-specific resources

        std::shared_ptr< simulation::World > m_dartWorld;

    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioPtr );

}