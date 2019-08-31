
#include <test_interface.h>

#include <dart/common/Timer.hpp>
#include <random>

class AppExample : public dart::ITestApplication
{
    private :

    void _createWalkerPlanar( const std::string& name,
                              tysoc::TVec3 position,
                              tysoc::TMat3 rotation );

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

    _createWalkerPlanar( "walker_agent", { 0.0, 0.0, 2.0 }, tysoc::TMat3::fromEuler( { 0.0, 0.0, 0.0 } ) );
}

void AppExample::_createWalkerPlanar( const std::string& name,
                                      tysoc::TVec3 position,
                                      tysoc::TMat3 rotation )
{
    auto _walkerAgent = new dart::SimAgent( name, m_dartWorldPtr );

    // base
    {
        dart::ShapeData _shapeTorso;
        _shapeTorso.name = "geom_torso";
        _shapeTorso.type = dart::eShapeType::CAPSULE;
        _shapeTorso.size = { 0.07, 0.6, 0.07 };
        _shapeTorso.color = { 0.7, 0.5, 0.3 };

        // compute the transform required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _jointTorso;
        _jointTorso.name = "jnt_torso_free";
        // _jointTorso.type = dart::eJointType::FREE;
        _jointTorso.type = dart::eJointType::PLANAR;
        _jointTorso.tfParentBody2Joint = _tfParentBody2Joint;
        _jointTorso.tfThisBody2Joint = _tfThisBody2Joint;

        _walkerAgent->addBody( "torso", "none", _shapeTorso, _jointTorso );
    }

    // base (using 3 dofs for planar)
    {

    }

    // rthigh - dummy
    {
        dart::ShapeData _shape_rthigh_jnt;
        _shape_rthigh_jnt.name = "geom_rthigh_jnt";
        _shape_rthigh_jnt.type = dart::eShapeType::NONE;
        
        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.05, -0.3 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_rthigh_revolute;
        _joint_rthigh_revolute.name = "jnt_rthigh_revolute";
        _joint_rthigh_revolute.type = dart::eJointType::REVOLUTE;
        _joint_rthigh_revolute.limits = { -TYSOC_PI / 9., TYSOC_PI * 5. / 9. };
        _joint_rthigh_revolute.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_rthigh_revolute.tfThisBody2Joint = _tfThisBody2Joint;
        _joint_rthigh_revolute.axis = { 0.0, -1.0, 0.0 };

        _walkerAgent->addBody( "rthigh_jnt", "torso", _shape_rthigh_jnt, _joint_rthigh_revolute );
    }

    // rthigh - body
    {
        dart::ShapeData _shape_rthigh;
        _shape_rthigh.name = "geom_rthigh";
        _shape_rthigh.type = dart::eShapeType::CAPSULE;
        _shape_rthigh.size = { 0.05, 0.45, 0.05 };
        _shape_rthigh.color = { 0.7, 0.5, 0.3 };

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.0, -0.225 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_rthigh_fixed;
        _joint_rthigh_fixed.name = "jnt_rthigh_fixed";
        _joint_rthigh_fixed.type = dart::eJointType::FIXED;
        _joint_rthigh_fixed.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_rthigh_fixed.tfThisBody2Joint = _tfThisBody2Joint;

        _walkerAgent->addBody( "rthigh", "rthigh_jnt", _shape_rthigh, _joint_rthigh_fixed );
    }

    // rleg - dummy
    {
        dart::ShapeData _shape_rleg_jnt;
        _shape_rleg_jnt.name = "geom_rleg_jnt";
        _shape_rleg_jnt.type = dart::eShapeType::NONE;

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.0, -0.225 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_rleg_revolute;
        _joint_rleg_revolute.name = "jnt_rleg_revolute";
        _joint_rleg_revolute.type = dart::eJointType::REVOLUTE;
        _joint_rleg_revolute.limits = { -TYSOC_PI * 5. / 6., 0.0 };
        _joint_rleg_revolute.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_rleg_revolute.tfThisBody2Joint = _tfThisBody2Joint;
        _joint_rleg_revolute.axis = { 0.0, -1.0, 0.0 };

        _walkerAgent->addBody( "rleg_jnt", "rthigh", _shape_rleg_jnt, _joint_rleg_revolute );
    }

    // rleg - body
    {
        dart::ShapeData _shape_rleg;
        _shape_rleg.name = "geom_rleg";
        _shape_rleg.type = dart::eShapeType::CAPSULE;
        _shape_rleg.size = { 0.04, 0.5, 0.04 };
        _shape_rleg.color = { 0.7, 0.5, 0.3 };

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.0, -0.25 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_rleg_fixed;
        _joint_rleg_fixed.name = "jnt_rleg_fixed";
        _joint_rleg_fixed.type = dart::eJointType::FIXED;
        _joint_rleg_fixed.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_rleg_fixed.tfThisBody2Joint = _tfThisBody2Joint;

        _walkerAgent->addBody( "rleg", "rleg_jnt", _shape_rleg, _joint_rleg_fixed );
    }

    // rfoot - dummy
    {
        dart::ShapeData _shape_rfoot_jnt;
        _shape_rfoot_jnt.name = "geom_rfoot_jnt";
        _shape_rfoot_jnt.type = dart::eShapeType::NONE;

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.0, -0.25 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_rfoot_revolute;
        _joint_rfoot_revolute.name = "jnt_rfoot_revolute";
        _joint_rfoot_revolute.type = dart::eJointType::REVOLUTE;
        _joint_rfoot_revolute.limits = { -TYSOC_PI / 4., TYSOC_PI / 4. };
        _joint_rfoot_revolute.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_rfoot_revolute.tfThisBody2Joint = _tfThisBody2Joint;
        _joint_rfoot_revolute.axis = { 0.0, -1.0, 0.0 };

        _walkerAgent->addBody( "rfoot_jnt", "rleg", _shape_rfoot_jnt, _joint_rfoot_revolute );
    }

    // rfoot - body
    {
        dart::ShapeData _shape_rfoot;
        _shape_rfoot.name = "geom_rfoot";
        _shape_rfoot.type = dart::eShapeType::CAPSULE;
        _shape_rfoot.size = { 0.05, 0.2, 0.05 };
        _shape_rfoot.color = { 0.7, 0.5, 0.3 };

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.06, 0.0, 0.0 } );
        _tfParentBody2Joint.setRotation( tysoc::TMat3( 0.0, 0.0, -1.0,
                                                       0.0, 1.0, 0.0,
                                                       1.0, 0.0, 0.0 ) );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_rfoot_fixed;
        _joint_rfoot_fixed.name = "jnt_rfoot_fixed";
        _joint_rfoot_fixed.type = dart::eJointType::FIXED;
        _joint_rfoot_fixed.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_rfoot_fixed.tfThisBody2Joint = _tfThisBody2Joint;

        _walkerAgent->addBody( "rfoot", "rfoot_jnt", _shape_rfoot, _joint_rfoot_fixed );
    }

    // lthigh - dummy
    {
        dart::ShapeData _shape_lthigh_jnt;
        _shape_lthigh_jnt.name = "geom_lthigh_jnt";
        _shape_lthigh_jnt.type = dart::eShapeType::NONE;
        
        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, -0.05, -0.3 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_lthigh_revolute;
        _joint_lthigh_revolute.name = "jnt_lthigh_revolute";
        _joint_lthigh_revolute.type = dart::eJointType::REVOLUTE;
        _joint_lthigh_revolute.limits = { -TYSOC_PI / 9., TYSOC_PI * 5. / 9. };
        _joint_lthigh_revolute.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_lthigh_revolute.tfThisBody2Joint = _tfThisBody2Joint;
        _joint_lthigh_revolute.axis = { 0.0, -1.0, 0.0 };

        _walkerAgent->addBody( "lthigh_jnt", "torso", _shape_lthigh_jnt, _joint_lthigh_revolute );
    }

    // lthigh - body
    {
        dart::ShapeData _shape_lthigh;
        _shape_lthigh.name = "geom_lthigh";
        _shape_lthigh.type = dart::eShapeType::CAPSULE;
        _shape_lthigh.size = { 0.05, 0.45, 0.05 };
        _shape_lthigh.color = { 0.7, 0.5, 0.3 };

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.0, -0.225 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_lthigh_fixed;
        _joint_lthigh_fixed.name = "jnt_lthigh_fixed";
        _joint_lthigh_fixed.type = dart::eJointType::FIXED;
        _joint_lthigh_fixed.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_lthigh_fixed.tfThisBody2Joint = _tfThisBody2Joint;

        _walkerAgent->addBody( "lthigh", "lthigh_jnt", _shape_lthigh, _joint_lthigh_fixed );
    }

    // lleg - dummy
    {
        dart::ShapeData _shape_lleg_jnt;
        _shape_lleg_jnt.name = "geom_lleg_jnt";
        _shape_lleg_jnt.type = dart::eShapeType::NONE;

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.0, -0.225 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_lleg_revolute;
        _joint_lleg_revolute.name = "jnt_lleg_revolute";
        _joint_lleg_revolute.type = dart::eJointType::REVOLUTE;
        _joint_lleg_revolute.limits = { -TYSOC_PI * 5. / 6., 0.0 };
        _joint_lleg_revolute.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_lleg_revolute.tfThisBody2Joint = _tfThisBody2Joint;
        _joint_lleg_revolute.axis = { 0.0, -1.0, 0.0 };

        _walkerAgent->addBody( "lleg_jnt", "lthigh", _shape_lleg_jnt, _joint_lleg_revolute );
    }

    // lleg - body
    {
        dart::ShapeData _shape_lleg;
        _shape_lleg.name = "geom_lleg";
        _shape_lleg.type = dart::eShapeType::CAPSULE;
        _shape_lleg.size = { 0.04, 0.5, 0.04 };
        _shape_lleg.color = { 0.7, 0.5, 0.3 };

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.0, -0.225 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_lleg_fixed;
        _joint_lleg_fixed.name = "jnt_lleg_fixed";
        _joint_lleg_fixed.type = dart::eJointType::FIXED;
        _joint_lleg_fixed.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_lleg_fixed.tfThisBody2Joint = _tfThisBody2Joint;

        _walkerAgent->addBody( "lleg", "lleg_jnt", _shape_lleg, _joint_lleg_fixed );
    }

    // lfoot - dummy
    {
        dart::ShapeData _shape_lfoot_jnt;
        _shape_lfoot_jnt.name = "geom_lfoot_jnt";
        _shape_lfoot_jnt.type = dart::eShapeType::NONE;

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.0, 0.0, -0.25 } );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_lfoot_revolute;
        _joint_lfoot_revolute.name = "jnt_lfoot_revolute";
        _joint_lfoot_revolute.type = dart::eJointType::REVOLUTE;
        _joint_lfoot_revolute.limits = { -TYSOC_PI / 4., TYSOC_PI / 4. };
        _joint_lfoot_revolute.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_lfoot_revolute.tfThisBody2Joint = _tfThisBody2Joint;
        _joint_lfoot_revolute.axis = { 0.0, -1.0, 0.0 };

        _walkerAgent->addBody( "lfoot_jnt", "lleg", _shape_lfoot_jnt, _joint_lfoot_revolute );
    }

    // lfoot - body
    {
        dart::ShapeData _shape_lfoot;
        _shape_lfoot.name = "geom_lfoot";
        _shape_lfoot.type = dart::eShapeType::CAPSULE;
        _shape_lfoot.size = { 0.05, 0.2, 0.05 };
        _shape_lfoot.color = { 0.7, 0.5, 0.3 };

        // compute the transforms required
        auto _tfParentBody2Joint = tysoc::TMat4();
        _tfParentBody2Joint.setIdentity();
        _tfParentBody2Joint.setPosition( { 0.06, 0.0, 0.0 } );
        _tfParentBody2Joint.setRotation( tysoc::TMat3( 0.0, 0.0, -1.0,
                                                       0.0, 1.0, 0.0,
                                                       1.0, 0.0, 0.0 ) );

        auto _tfThisBody2Joint = tysoc::TMat4();
        _tfThisBody2Joint.setIdentity();

        dart::JointData _joint_lfoot_fixed;
        _joint_lfoot_fixed.name = "jnt_lfoot_fixed";
        _joint_lfoot_fixed.type = dart::eJointType::FIXED;
        _joint_lfoot_fixed.tfParentBody2Joint = _tfParentBody2Joint;
        _joint_lfoot_fixed.tfThisBody2Joint = _tfThisBody2Joint;

        _walkerAgent->addBody( "lfoot", "lfoot_jnt", _shape_lfoot, _joint_lfoot_fixed );
    }

    addSimAgent( _walkerAgent, position, rotation );
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