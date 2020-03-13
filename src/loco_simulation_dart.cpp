
#include <loco_simulation_dart.h>

namespace loco {
namespace dartsim {

    TDartSimulation::TDartSimulation( TScenario* scenarioRef )
        : TISimulation( scenarioRef )
    {
        m_backendId = "DART";

        m_dartWorld = dart::simulation::World::create();
        m_dartWorld->setTimeStep( 0.001 );
        m_dartWorld->setGravity( vec3_to_eigen( { 0, 0, -9.81 } ) );

        // BULLET collision-detector: Faster for meshes, but a bit slower than the ODE version
        m_dartWorld->getConstraintSolver()->setCollisionDetector( dart::collision::BulletCollisionDetector::create() );
        // ODE collision-detector: Faster for primitives, but meshes are awfully slow (should use cvx-hull)
        //// m_dartWorld->getConstraintSolver()->setCollisionDetector( dart::collision::OdeCollisionDetector::create() );

        auto boxed_lcp_constraint_solver = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>( m_dartWorld->getConstraintSolver() );
        // DANTZIG constraint-solver: Seems faster, but breaks in some cases (@todo: test+document failure cases)
        boxed_lcp_constraint_solver->setBoxedLcpSolver( std::make_shared<dart::constraint::DantzigBoxedLcpSolver>() );
        boxed_lcp_constraint_solver->setSecondaryBoxedLcpSolver( std::make_shared<dart::constraint::PgsBoxedLcpSolver>() );
        // PROJ.-GAUSS-SEIDEL constraint-solver: More robust where dantzig fails, but still fails in some cases (@todo: test+document failure cases)
        //// boxed_lcp_constraint_solver->setBoxedLcpSolver( std::make_shared<dart::constraint::PgsBoxedLcpSolver>() );

        _CreateSingleBodyAdapters();
        //// _CreateCompoundAdapters();
        //// _CreateKintreeAdapters();
        //// _CreateTerrainGeneratorAdapters();

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TDartSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TDartSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TDartSimulation::_CreateSingleBodyAdapters()
    {
        auto single_bodies = m_scenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto single_body_adapter = std::make_unique<TDartSingleBodyAdapter>( single_body );
            single_body_adapter->SetDartWorld( m_dartWorld.get() );
            single_body->SetAdapter( single_body_adapter.get() );
            m_singleBodyAdapters.push_back( std::move( single_body_adapter ) );

            auto collider = single_body->collision();
            LOCO_CORE_ASSERT( collider, "TDartSimulation::_CreateSingleBodyAdapters >>> single-body {0} \
                              must have an associated collider", single_body->name() );

            auto collider_adapter = std::make_unique<TDartCollisionAdapter>( collider );
            collider_adapter->SetDartWorld( m_dartWorld.get() );
            collider->SetAdapter( collider_adapter.get() );
            m_collisionAdapters.push_back( std::move( collider_adapter ) );
        }
    }

    TDartSimulation::~TDartSimulation()
    {
        m_dartWorld = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( TLogger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TDartSimulation @ {0}", loco::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TDartSimulation @ " << loco::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    bool TDartSimulation::_InitializeInternal()
    {
        // Collect dart-resources from the adapters and assemble any required resources
        // @todo: implement-me ...

        LOCO_CORE_TRACE( "Dart-backend >>> gravity      : {0}", ToString( vec3_from_eigen( m_dartWorld->getGravity() ) ) );
        LOCO_CORE_TRACE( "Dart-backend >>> time-step    : {0}", std::to_string( m_dartWorld->getTimeStep() ) );
        LOCO_CORE_TRACE( "Dart-backend >>> num-skeletons: {0}", std::to_string( m_dartWorld->getNumSkeletons() ) );

        return true;
    }

    void TDartSimulation::_PreStepInternal()
    {
        // Do nothing here, as call to wrappers is enough (made in base)
    }

    void TDartSimulation::_SimStepInternal()
    {
        const double target_steptime = 1.0 / 60.0;
        const double sim_start = m_dartWorld->getTime();
        while ( m_dartWorld->getTime() - sim_start < target_steptime )
            m_dartWorld->step();
    }

    void TDartSimulation::_PostStepInternal()
    {
        // @todo: run loco-contact-manager here to grab all detected contacts
    }

    void TDartSimulation::_ResetInternal()
    {
        // @todo: reset loco-contact-manager
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef )
    {
        return new loco::dartsim::TDartSimulation( scenarioRef );
    }

}}