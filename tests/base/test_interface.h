
#pragma once

// DART API functionality
#include <dart/dart.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
// Rendering functionality from 'cat1' engine
#include <LApp.h>
#include <LFpsCamera.h>
#include <LFixedCamera3d.h>
#include <LLightDirectional.h>
#include <LMeshBuilder.h>
// UI functionality (from Dear ImGui)
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// Base functionality (math and a few helpers) from tysoc-core
#include <tysoc_common.h>
#include <memory>
#include <dirent.h>

#define DEFAULT_DENSITY 1000.0f // density of water, same default as in mujoco

namespace dart
{

    enum class eShapeType
    {
        NONE = 0,
        BOX, 
        PLANE, 
        SPHERE, 
        CYLINDER, 
        CAPSULE, 
        MESH
    };

    enum class eJointType
    {
        FREE = 0,
        REVOLUTE,
        PRISMATIC,
        BALL,
        PLANAR,
        FIXED
    };

    struct ShapeData
    {
        std::string     name;
        tysoc::TVec3    localPos;   // relative position w.r.t. parent body
        tysoc::TMat3    localRot;   // relative rotation w.r.t. parent body
        eShapeType      type;       // type of collision shape of this body
        tysoc::TVec3    size;       // size of the collision shape of this body
        tysoc::TVec3    color;      // color (rgb) of the renderable associated with this shape
    };

    struct JointData
    {
        std::string     name;
        tysoc::TVec3    pos;    // relative position w.r.t. parent body
        eJointType      type;   // type of joint to be constructed
        tysoc::TVec3    axis;   // axis of joint to be constructed
        tysoc::TVec2    limits; // motion range (lo==hi: fixed, lo>hi: continuous, lo<hi: limited)
    };

    Eigen::Vector3d toEigenVec3( const tysoc::TVec3& vec );
    Eigen::Isometry3d toEigenTransform( const tysoc::TMat4& transform );

    tysoc::TVec3 fromEigenVec3( const Eigen::Vector3d& vec );
    tysoc::TMat4 fromEigenTransform( const Eigen::Isometry3d& transform );

    std::vector< std::string > collectAvailableModels( const std::string& folderpath );

    dynamics::ShapePtr createCollisionShape( const ShapeData& shapeData );
    engine::LIRenderable* createRenderableShape( const ShapeData& shapeData );

    Eigen::Vector3d computeCOMoffset( const ShapeData& shapeData );

    /**
    *   Wrapper for a single body with geometries as children
    */
    class SimBody
    {
        protected :

        std::string     m_name;
        tysoc::TVec3    m_worldPos;
        tysoc::TMat3    m_worldRot;
        tysoc::TMat4    m_worldTransform;

        engine::LIRenderable*   m_graphicsObj;      // reference a renderable representing the body
        dynamics::ShapePtr      m_dartShapePtr;     // reference to the collision shape of this body
        dynamics::ShapeNodePtr  m_dartShapeNodePtr; // reference to the collision node of this body
        dynamics::BodyNodePtr   m_dartBodyNodePtr;  // reference to the actual bodynode resource
        dynamics::JointPtr      m_dartJointPtr;     // reference to the joint that links it to the skeleton
        dynamics::SkeletonPtr   m_dartSkeletonPtr;  // reference to the skeleton-holder of the body

        SimBody* m_parent; // reference to the parent simbody in the skeleton

        public :

        /* Constructs the body wrapper given the name of the body */
        SimBody( const std::string& name,
                 dynamics::SkeletonPtr dartSkeletonPtr );

        /* Frees|unlinks all related resources of the currently wrapped body */
        ~SimBody() {}

        /* creates the low-level resources for this body wrapper in simulation */
        void build( SimBody* parent,
                    const ShapeData& shapeData,
                    const JointData& jointData );

        /* Updates all internal information by quering the backend */
        void update();

        /* Returns the unique name of the wrapped body */
        std::string name() { return m_name; }

        /* Returns all meshes linked to each geometry */
        engine::LIRenderable* graphics() { return m_graphicsObj; }

        /* Resets the body to some configuration */
        void reset() {}

        /* Prints some debug information */
        void print() {}

        /* Returns the parent simbody */
        SimBody* parent() { return m_parent; }

        /* Returns the dart reference to the bodynode */
        dynamics::BodyNodePtr node() { return m_dartBodyNodePtr; }

        /* Returns the world-position of the body */
        tysoc::TVec3 position() { return m_worldPos; }

        /* Returns the world-orientation of the body */
        tysoc::TMat3 orientation() { return m_worldRot; }

        /* Returns the world transform of the body */
        tysoc::TMat4 worldTransform() { return m_worldTransform; }
    };

    class SimJoint
    {
        protected :




        public :


    };

    class SimAgent
    {
        protected :

        std::string                             m_name;
        std::vector< SimBody* >                 m_simBodies;
        std::map< std::string, SimBody* >       m_simBodiesMap;
        std::vector< SimJoint* >                m_simJoints;
        std::map< std::string, SimJoint* >      m_simJointsMap;

        std::shared_ptr<dynamics::Skeleton>     m_dartSkeletonPtr;
        std::shared_ptr<simulation::World>      m_dartWorldPtr;

        public :

        SimAgent( const std::string& name,
                  std::shared_ptr<simulation::World> dartWorldPtr );

        ~SimAgent() {}

        /**
        *   Creates a body-node wrapper linked by a joint (of some 
        *   given data) to another body (parent). Both child and
        *   parent could be even massless, to handle multi-dof cases.
        */
        SimBody* addBody( const std::string& name,
                          const std::string& parentName,
                          const ShapeData& shapeData,
                          const JointData& jointData );

        void update();

        void reset() {}

        /* returns the name of this skeleton */
        std::string name() { return m_name; }

        /* Return the wrapped dart-skeleton */
        dynamics::SkeletonPtr skeleton() { return m_dartSkeletonPtr; }

        /* Returns all bodies belonging to this agent */
        std::vector< SimBody* > bodies() { return m_simBodies; }

        /* Return all joints belonging to this agent */
        std::vector< SimJoint* > joints() { return m_simJoints; }

    };


    class ITestApplication
    {

        protected :

        std::shared_ptr<simulation::World> m_dartWorldPtr;

        engine::LApp* m_graphicsApp;
        engine::LScene* m_graphicsScene;

        std::vector< SimBody* >             m_simBodies;
        std::map< std::string, SimBody* >   m_simBodiesMap;

        std::vector< SimAgent* >            m_simAgents;
        std::map< std::string, SimAgent* >  m_simAgentsMap;

        bool m_isRunning;
        bool m_isTerminated;

        void _initScenario();
        void _initPhysics(); 
        void _initGraphics();

        void _renderUi();
        void _renderUiAgents();

        // User should override this and create what he wants
        virtual void _initScenarioInternal() = 0;
        // Special functionality used after taking a step
        virtual void _stepInternal() {}
        // Special functionality used before calling reset
        virtual void _resetInternal() {}
        // UI functionality
        virtual void _renderUiInternal() {}
        // Entry-point for when the application is fully created
        virtual void _onApplicationStart() {}

        public :

        ITestApplication();
        ~ITestApplication();

        void init();
        void reset();
        void step();
        void togglePause();

        bool isTerminated() { return m_isTerminated; }

        SimBody* createSingleBody( const ShapeData& shapeData, bool isFree = true );

        void addSimAgent( SimAgent* simAgentPtr, tysoc::TVec3& position );

        /* Returns a body-wrapper of a body with a specific name in the simulation */
        SimBody* getBody( const std::string& name );
    };

}