#pragma once

#include <loco_common_dart.h>
#include <loco_simulation.h>

#include <primitives/loco_single_body_collider_adapter_dart.h>
#include <primitives/loco_single_body_adapter_dart.h>

namespace loco {
namespace dartsim {

    class TDartSimulation : public TISimulation
    {
    public :

        TDartSimulation( TScenario* scenarioRef );

        TDartSimulation( const TDartSimulation& other ) = delete;

        TDartSimulation& operator=( const TDartSimulation& other ) = delete;

        ~TDartSimulation();

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal() override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

    private :

        void _CreateSingleBodyAdapters();
        //// void _CreateCompoundAdapters();
        //// void _CreateKintreeAdapters();
        //// void _CreateTerrainGeneratorAdapters();

    private :

        dart::simulation::WorldPtr m_dartWorld;

    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef );

}}