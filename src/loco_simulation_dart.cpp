
#include <loco_simulation_dart.h>

namespace loco {
namespace dartsim {

    /***********************************************************************************************
    *                                    Dart Simulation Impl.                                     *
    ***********************************************************************************************/

    TDartSimulation::TDartSimulation( TScenario* scenarioRef )
        : TISimulation( scenarioRef )
    {
        m_BackendId = "DART";

        m_DartWorld = dart::simulation::World::create();
        m_DartWorld->setTimeStep( m_FixedTimeStep );
        m_DartWorld->setGravity( vec3_to_eigen( m_Gravity ) );

        // BULLET collision-detector: Faster for meshes, but a bit slower than the ODE version
        m_DartWorld->getConstraintSolver()->setCollisionDetector( dart::collision::BulletCollisionDetector::create() );
        // ODE collision-detector: Faster for primitives, but meshes are awfully slow (should use cvx-hull)
        //// m_DartWorld->getConstraintSolver()->setCollisionDetector( dart::collision::OdeCollisionDetector::create() );

        auto boxed_lcp_constraint_solver = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>( m_DartWorld->getConstraintSolver() );
        // DANTZIG constraint-solver: Seems faster, but breaks in some cases (@todo: test+document failure cases)
        boxed_lcp_constraint_solver->setBoxedLcpSolver( std::make_shared<dart::constraint::DantzigBoxedLcpSolver>() );
        boxed_lcp_constraint_solver->setSecondaryBoxedLcpSolver( std::make_shared<dart::constraint::PgsBoxedLcpSolver>() );
        // PROJ.-GAUSS-SEIDEL constraint-solver: More robust where dantzig fails, but still fails in some cases (@todo: test+document failure cases)
        //// boxed_lcp_constraint_solver->setBoxedLcpSolver( std::make_shared<dart::constraint::PgsBoxedLcpSolver>() );

        m_DartWorld->getConstraintSolver()->getCollisionOption().collisionFilter = std::make_shared<TDartBitmaskCollisionFilter>();

        _CreateSingleBodyAdapters();
        //// _CreateCompoundAdapters();
        //// _CreateKintreeAdapters();
        //// _CreateTerrainGeneratorAdapters();

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( tinyutils::Logger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Created TDartSimulation @ {0}", tinyutils::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Created TDartSimulation @ " << tinyutils::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    void TDartSimulation::_CreateSingleBodyAdapters()
    {
        auto single_bodies = m_ScenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto single_body_adapter = std::make_unique<TDartSingleBodyAdapter>( single_body );
            single_body->SetBodyAdapter( single_body_adapter.get() );
            m_SingleBodyAdapters.push_back( std::move( single_body_adapter ) );
        }
    }

    TDartSimulation::~TDartSimulation()
    {
        m_DartWorld = nullptr;

    #if defined( LOCO_CORE_USE_TRACK_ALLOCS )
        if ( tinyutils::Logger::IsActive() )
            LOCO_CORE_TRACE( "Loco::Allocs: Destroyed TDartSimulation @ {0}", tinyutils::PointerToHexAddress( this ) );
        else
            std::cout << "Loco::Allocs: Destroyed TDartSimulation @ " << tinyutils::PointerToHexAddress( this ) << std::endl;
    #endif
    }

    bool TDartSimulation::_InitializeInternal()
    {
        for ( auto& single_body_adapter : m_SingleBodyAdapters )
        {
            if ( auto dart_adapter = dynamic_cast<TDartSingleBodyAdapter*>( single_body_adapter.get() ) )
                dart_adapter->SetDartWorld( m_DartWorld.get() );
        }

        // Collect dart-resources from the adapters and assemble any required resources
        // @todo: implement-me ...

        LOCO_CORE_TRACE( "Dart-backend >>> gravity      : {0}", ToString( vec3_from_eigen( m_DartWorld->getGravity() ) ) );
        LOCO_CORE_TRACE( "Dart-backend >>> time-step    : {0}", std::to_string( m_DartWorld->getTimeStep() ) );
        LOCO_CORE_TRACE( "Dart-backend >>> num-skeletons: {0}", std::to_string( m_DartWorld->getNumSkeletons() ) );

        return true;
    }

    void TDartSimulation::_PreStepInternal()
    {
        // Do nothing here, as call to wrappers is enough (made in base)
    }

    void TDartSimulation::_SimStepInternal( const TScalar& dt )
    {
        LOCO_CORE_ASSERT( m_DartWorld, "TDartSimulation::_SimStepInternal >>> \
                          dart-world is required, but got nullptr instead" );
        const double sim_step_time = ( dt <= 0 ) ? m_FixedTimeStep : dt;
        const double sim_start_time = m_DartWorld->getTime();
        while ( m_DartWorld->getTime() - sim_start_time < sim_step_time )
        {
            m_DartWorld->step();
            m_WorldTime += m_FixedTimeStep;
        }
    }

    void TDartSimulation::_PostStepInternal()
    {
        // @todo: run loco-contact-manager here to grab all detected contacts
    }

    void TDartSimulation::_ResetInternal()
    {
        // @todo: reset loco-contact-manager
    }

    void TDartSimulation::_SetTimeStepInternal( const TScalar& time_step )
    {
        LOCO_CORE_ASSERT( m_DartWorld, "TDartSimulation::_SetTimeStepInternal >>> \
                          dart-world is required, but got nullptr instead" );
        m_DartWorld->setTimeStep( time_step );
    }

    void TDartSimulation::_SetGravityInternal( const TVec3& gravity )
    {
        LOCO_CORE_ASSERT( m_DartWorld, "TDartSimulation::_SetGravityInternal >>> \
                          dart-world is required, but got nullptr instead" );
        m_DartWorld->setGravity( vec3_to_eigen( gravity ) );
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef )
    {
        return new loco::dartsim::TDartSimulation( scenarioRef );
    }

}}