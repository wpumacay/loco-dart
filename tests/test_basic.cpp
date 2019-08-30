
#include <test_interface.h>

#include <dart/common/Timer.hpp>
#include <random>

class AppExample : public dart::ITestApplication
{
    private :

    dart::SimBody* m_samplePlanePtr;

    dart::ShapeData m_sampleBoxShapeData;
    dart::ShapeData m_sampleSphereShapeData;
    dart::ShapeData m_sampleCylinderShapeData;
    dart::ShapeData m_sampleCapsuleShapeData;

    std::map< std::string, int > m_shapeNum;
    std::map< std::string, dart::ShapeData > m_shapeDataOptions;

    std::default_random_engine              m_randomGenerator;
    std::uniform_real_distribution<double>  m_randomUniformDistribution;

    void _createBody( const std::string& type );

    protected :

    void _initScenarioInternal() override;
    void _onApplicationStart() override;
    void _stepInternal() override;

    public :

    AppExample();
    ~AppExample();
};

AppExample::AppExample()
    : dart::ITestApplication()
{
    // nothing here
}

AppExample::~AppExample()
{
    // nothing here
}

void AppExample::_initScenarioInternal()
{
    // create shape data for later creation of some objects
    m_sampleBoxShapeData.name = "box";
    m_sampleBoxShapeData.type = dart::eShapeType::BOX;
    m_sampleBoxShapeData.size = { 0.2, 0.2, 0.2 };
    m_sampleBoxShapeData.color = { 0.7, 0.5, 0.3 };

    m_sampleSphereShapeData.name = "sphere";
    m_sampleSphereShapeData.type = dart::eShapeType::SPHERE;
    m_sampleSphereShapeData.size = { 0.1, 0.1, 0.1 };
    m_sampleSphereShapeData.color = { 0.7, 0.5, 0.3 };

    m_sampleCylinderShapeData.name = "cylinder";
    m_sampleCylinderShapeData.type = dart::eShapeType::CYLINDER;
    m_sampleCylinderShapeData.size = { 0.1, 0.2, 0.1 };
    m_sampleCylinderShapeData.color = { 0.7, 0.5, 0.3 };

    m_sampleCapsuleShapeData.name = "capsule";
    m_sampleCapsuleShapeData.type = dart::eShapeType::CAPSULE;
    m_sampleCapsuleShapeData.size = { 0.1, 0.2, 0.1 };
    m_sampleCapsuleShapeData.color = { 0.7, 0.5, 0.3 };

    m_shapeDataOptions = { { "box", m_sampleBoxShapeData }, 
                           { "sphere", m_sampleSphereShapeData }, 
                           { "cylinder", m_sampleCylinderShapeData },
                           { "capsule", m_sampleCapsuleShapeData } };

    m_shapeNum = { { "box", 0 }, 
                   { "sphere", 0 }, 
                   { "cylinder", 0 },
                   { "capsule", 0 } };

    m_randomUniformDistribution = std::uniform_real_distribution<double>( -2.0, 2.0 );
}

void AppExample::_createBody( const std::string& type )
{
    if ( m_shapeDataOptions.find( type ) == m_shapeDataOptions.end() )
        return;

    // choose a random position
    tysoc::TVec3 _position;
    _position.x = m_randomUniformDistribution( m_randomGenerator );
    _position.y = m_randomUniformDistribution( m_randomGenerator );
    _position.z = 3.0;

    // choose a random orientation
    tysoc::TVec3 _rotation;
    _rotation.x = TYSOC_PI * m_randomUniformDistribution( m_randomGenerator ) / 4.;
    _rotation.y = TYSOC_PI * m_randomUniformDistribution( m_randomGenerator ) / 4.;
    _rotation.z = TYSOC_PI * m_randomUniformDistribution( m_randomGenerator ) / 4.;

    // create a copy of the shape data with the initial position and rotation
    dart::ShapeData _shapeData( m_shapeDataOptions[type] );

    _shapeData.name     = type + "_" + std::to_string( m_shapeNum[type] );
    _shapeData.localPos = _position;
    _shapeData.localRot = tysoc::TMat3::fromEuler( _rotation );

    createSingleBody( _shapeData, true );

    m_shapeNum[type]++;
}

void AppExample::_onApplicationStart()
{
    std::cout << "INFO> Application has started" << std::endl;

    dart::ShapeData _planeShapeData;
    _planeShapeData.name = "gplane";
    _planeShapeData.type = dart::eShapeType::BOX;
    _planeShapeData.size = { 10.0, 10.0, 0.2 };
    _planeShapeData.color = { 0.2, 0.4, 0.6 };
    _planeShapeData.localPos = { 0.0, 0.0, -0.1 };

    m_samplePlanePtr = createSingleBody( _planeShapeData, false );

    for ( size_t i = 0; i < 5; i++ )
        _createBody( "box" );

    for ( size_t i = 0; i < 5; i++ )
        _createBody( "sphere" );

    for ( size_t i = 0; i < 5; i++ )
        _createBody( "cylinder" );

    for ( size_t i = 0; i < 5; i++ )
        _createBody( "capsule" );
}

void AppExample::_stepInternal()
{
    if ( engine::InputSystem::checkSingleKeyPress( GLFW_KEY_C ) )
    {
        std::vector< std::string > _options = { "box", "sphere", "cylinder", "capsule" };
        _createBody( _options[rand() % 4] );
        // _createBody( "sphere" );
    }
}

int main()
{

    auto _app = new AppExample();
    auto _timer = new dart::common::Timer( "cpu-timer" );
    _app->init();
    _app->reset();

    while ( !_app->isTerminated() )
    {
        _timer->start();
        _app->step();
        _timer->stop();

        auto _dt = _timer->getElapsedTime();
        auto _fps = 1. / _dt;

        std::cout << "fps: " << _fps << " - dt(ms): " << (1000. * _dt) << std::endl;
    }

    delete _app;

    return 0;
}