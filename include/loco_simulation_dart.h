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

        dart::simulation::WorldPtr& dart_world() { return m_DartWorld; }

        const dart::simulation::WorldPtr& dart_world() const { return m_DartWorld; }

    protected :

        bool _InitializeInternal() override;

        void _PreStepInternal() override;

        void _SimStepInternal( const TScalar& dt ) override;

        void _PostStepInternal() override;

        void _ResetInternal() override;

        void _SetTimeStepInternal( const TScalar& time_step ) override;

        void _SetGravityInternal( const TVec3& gravity ) override;

    private :

        void _CreateSingleBodyAdapters();
        //// void _CreateCompoundAdapters();
        //// void _CreateKintreeAdapters();
        //// void _CreateTerrainGeneratorAdapters();

    private :

        dart::simulation::WorldPtr m_DartWorld;

    };

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef );

}}