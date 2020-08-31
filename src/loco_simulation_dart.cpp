
#include <loco_simulation_dart.h>

namespace loco {

    /***********************************************************************************************
    *                                    Dart Simulation Impl.                                     *
    ***********************************************************************************************/

    TDartSimulation::TDartSimulation( TScenario* scenarioRef )
        : TISimulation( scenarioRef )
    {
        m_BackendId = "DART";

        m_DartWorld = dart::simulation::World::create();
        m_DartWorld->setTimeStep( m_FixedTimeStep );
        m_DartWorld->setGravity( dartsim::vec3_to_eigen( m_Gravity ) );

        // BULLET collision-detector: Faster for meshes, but a bit slower than the ODE version
        m_DartWorld->getConstraintSolver()->setCollisionDetector( dart::collision::BulletCollisionDetector::create() );

        auto boxed_lcp_constraint_solver = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>( m_DartWorld->getConstraintSolver() );
        // DANTZIG constraint-solver: Seems faster, but breaks in some cases (@todo: test+document failure cases)
        boxed_lcp_constraint_solver->setBoxedLcpSolver( std::make_shared<dart::constraint::DantzigBoxedLcpSolver>() );
        boxed_lcp_constraint_solver->setSecondaryBoxedLcpSolver( std::make_shared<dart::constraint::PgsBoxedLcpSolver>() );
        // PROJ.-GAUSS-SEIDEL constraint-solver: More robust where dantzig fails, but still fails in some cases (@todo: test+document failure cases)
        //// boxed_lcp_constraint_solver->setBoxedLcpSolver( std::make_shared<dart::constraint::PgsBoxedLcpSolver>() );

        m_DartWorld->getConstraintSolver()->getCollisionOption().collisionFilter = std::make_shared<dartsim::TDartBitmaskCollisionFilter>();

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
            auto single_body_adapter = std::make_unique<primitives::TDartSingleBodyAdapter>( single_body );
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
            if ( auto dart_adapter = dynamic_cast<primitives::TDartSingleBodyAdapter*>( single_body_adapter.get() ) )
                dart_adapter->SetDartWorld( m_DartWorld.get() );
        }

        // Collect dart-resources from the adapters and assemble any required resources
        // @todo: implement-me ...

        LOCO_CORE_TRACE( "Dart-backend >>> gravity      : {0}", ToString( dartsim::vec3_from_eigen( m_DartWorld->getGravity() ) ) );
        LOCO_CORE_TRACE( "Dart-backend >>> time-step    : {0}", std::to_string( m_DartWorld->getTimeStep() ) );
        LOCO_CORE_TRACE( "Dart-backend >>> num-skeletons: {0}", std::to_string( m_DartWorld->getNumSkeletons() ) );

        return true;
    }

    void TDartSimulation::_CollectContacts()
    {
        LOCO_CORE_ASSERT( m_DartWorld, "TDartSimulation::_CollectContacts >>> dart-world object \
                           is required, but got nullptr instead" );

        std::unordered_map<const dart::dynamics::Shape*, std::string> shape_to_collider;
        auto single_bodies = m_ScenarioRef->GetSingleBodiesList();
        for ( auto single_body : single_bodies )
        {
            auto collider = single_body->collider();
            auto dart_collider_adapter = static_cast<primitives::TDartSingleBodyColliderAdapter*>( collider->collider_adapter() );
            const dart::dynamics::Shape* dart_collider_shape = dart_collider_adapter->collision_shape().get();
            shape_to_collider[dart_collider_shape] = collider->name();
        }

        std::map< std::string, std::vector<TContactData> > detected_contacts;
        const auto& collision_result = m_DartWorld->getLastCollisionResult();
        const size_t num_contacts = collision_result.getNumContacts();
        for ( size_t i = 0; i < num_contacts; i++ )
        {
            const auto& contact_info = collision_result.getContact( i );
            const dart::dynamics::Shape* collider_shape_1 = contact_info.collisionObject1->getShape().get();
            const dart::dynamics::Shape* collider_shape_2 = contact_info.collisionObject2->getShape().get();
            if ( ( shape_to_collider.find( collider_shape_1 ) == shape_to_collider.end() ) ||
                 ( shape_to_collider.find( collider_shape_2 ) == shape_to_collider.end() ) )
            {
                LOCO_CORE_WARN( "TDartSimulation::_CollectContacts >>> a contact is dangling without a contact-pair" );
                continue;
            }

            const std::string collider_1 = shape_to_collider[collider_shape_1];
            const std::string collider_2 = shape_to_collider[collider_shape_2];
            const TVec3 position = dartsim::vec3_from_eigen( contact_info.point );
            const TVec3 normal = dartsim::vec3_from_eigen( contact_info.normal );

            if ( detected_contacts.find( collider_1 ) == detected_contacts.end() )
                detected_contacts[collider_1] = std::vector<TContactData>();
            if ( detected_contacts.find( collider_2 ) == detected_contacts.end() )
                detected_contacts[collider_2] = std::vector<TContactData>();

            TContactData contact_1, contact_2;
            contact_1.position = position;  contact_2.position = position;
            contact_1.normal = normal;      contact_2.normal = normal.scaled( -1.0 );
            contact_1.name = collider_2;    contact_2.name = collider_1;

            detected_contacts[collider_1].push_back( contact_1 );
            detected_contacts[collider_2].push_back( contact_2 );
        }

        for ( auto single_body : single_bodies )
        {
            auto collider = single_body->collider();
            auto collider_name = collider->name();

            collider->contacts().clear();
            if ( detected_contacts.find( collider_name ) != detected_contacts.end() )
                collider->contacts() = detected_contacts[collider_name];
        }
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
        const ssize_t sim_num_substeps = ssize_t(sim_step_time / m_FixedTimeStep);
        for ( ssize_t i = sim_num_substeps - 1; i >= 0; i-- )
        {
            m_DartWorld->step( (i == 0) );
            m_WorldTime += m_FixedTimeStep;
        }
    }

    void TDartSimulation::_PostStepInternal()
    {
        _CollectContacts();
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
        m_DartWorld->setGravity( dartsim::vec3_to_eigen( gravity ) );
    }

    extern "C" TISimulation* simulation_create( TScenario* scenarioRef )
    {
        return new loco::TDartSimulation( scenarioRef );
    }
}