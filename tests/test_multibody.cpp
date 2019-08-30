
#include <test_interface.h>

#include <dart/common/Timer.hpp>
#include <random>

class AppExample : public dart::ITestApplication
{
    private :

    void _createWalkerPlanar( const std::string& name,
                              tysoc::TVec3 position );

    protected :

    void _initScenarioInternal() override;
    void _onApplicationStart() override;

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
    // do nothing for now
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

    createSingleBody( _planeShapeData, false );

    _createWalkerPlanar( "walker_agent", { 0.0, 0.0, 2.0 } );
}

void AppExample::_createWalkerPlanar( const std::string& name,
                                      tysoc::TVec3 position )
{
    auto _walkerAgent = new dart::SimAgent( name, m_dartWorldPtr );

    dart::ShapeData _shapeTorso;
    _shapeTorso.name = "geom_torso";
    _shapeTorso.type = dart::eShapeType::CAPSULE;
    _shapeTorso.size = { 0.07, 0.6, 0.07 };
    _shapeTorso.color = { 0.7, 0.5, 0.3 };

    dart::JointData _jointTorso;
    _jointTorso.name = "jnt_torso_free";
    _jointTorso.type = dart::eJointType::FREE;

    _walkerAgent->addBody( "torso", "none", _shapeTorso, _jointTorso );

    dart::ShapeData _shape_rthigh_jnt;
    _shape_rthigh_jnt.name = "geom_rthigh_jnt";
    _shape_rthigh_jnt.type = dart::eShapeType::NONE;
    
    dart::JointData _joint_rthigh_revolute;
    _joint_rthigh_revolute.name = "jnt_rthigh_revolute";
    _joint_rthigh_revolute.type = dart::eJointType::REVOLUTE;
    _joint_rthigh_revolute.pos  = { 0.0, -0.05, -0.3 };
    _joint_rthigh_revolute.axis = { 0.0, -1.0, 0.0 };

    _walkerAgent->addBody( "rthigh_jnt", "torso", _shape_rthigh_jnt, _joint_rthigh_revolute );

    dart::ShapeData _shape_rthigh;
    _shape_rthigh.name = "geom_rthigh";
    _shape_rthigh.type = dart::eShapeType::CAPSULE;
    _shape_rthigh.size = { 0.05, 0.45, 0.05 };
    _shape_rthigh.color = { 0.7, 0.5, 0.3 };

    dart::JointData _joint_rthigh_fixed;
    _joint_rthigh_fixed.name = "jnt_rthigh_fixed";
    _joint_rthigh_fixed.type = dart::eJointType::FIXED;
    _joint_rthigh_fixed.pos = { 0.0, 0.0, 0.45 / 2. + 0.6 };

    _walkerAgent->addBody( "rthigh", "rthigh_jnt", _shape_rthigh, _joint_rthigh_fixed );

    dart::ShapeData _shape_lthigh_jnt;
    _shape_lthigh_jnt.name = "geom_lthigh_jnt";
    _shape_lthigh_jnt.type = dart::eShapeType::NONE;
    
    dart::JointData _joint_lthigh_revolute;
    _joint_lthigh_revolute.name = "jnt_lthigh_revolute";
    _joint_lthigh_revolute.type = dart::eJointType::REVOLUTE;
    _joint_lthigh_revolute.pos  = { 0.0, 0.05, -0.3 };
    _joint_lthigh_revolute.axis = { 0.0, 1.0, 0.0 };

    _walkerAgent->addBody( "lthigh_jnt", "torso", _shape_lthigh_jnt, _joint_lthigh_revolute );

    dart::ShapeData _shape_lthigh;
    _shape_lthigh.name = "geom_lthigh";
    _shape_lthigh.type = dart::eShapeType::CAPSULE;
    _shape_lthigh.size = { 0.05, 0.45, 0.05 };
    _shape_lthigh.color = { 0.7, 0.5, 0.3 };

    dart::JointData _joint_lthigh_fixed;
    _joint_lthigh_fixed.name = "jnt_lthigh_fixed";
    _joint_lthigh_fixed.type = dart::eJointType::FIXED;
    _joint_lthigh_fixed.pos = { 0.0, 0.0, 0.45 / 2. + 0.6 };

    _walkerAgent->addBody( "lthigh", "lthigh_jnt", _shape_lthigh, _joint_lthigh_fixed );

    addSimAgent( _walkerAgent, position );
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