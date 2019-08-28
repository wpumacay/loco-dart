
#pragma once

// DART API functionality
#include <dart/dart.hpp>
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

    std::vector< std::string > collectAvailableModels( const std::string& folderpath );

    /**
    *   Wrapper for a single body with geometries as children
    */
    class SimBody
    {
        protected :

        std::string     m_bodyName;
        tysoc::TVec3    m_bodyWorldPos;
        tysoc::TMat3    m_bodyWorldRot;
        tysoc::TMat4    m_bodyWorldTransform;

        std::vector< engine::LIRenderable* > m_geomsGraphics;

        /* Grabs all geometries associated with this body */
        void _grabGeometries();

        /* Builds a geometry's graphics */
        engine::LIRenderable* _buildGeomGraphics( const std::string& type,
                                                  const tysoc::TVec3& size,
                                                  const tysoc::TVec4& color );

        public :

        /* Constructs the body wrapper given the name of the body */
        SimBody( const std::string& bodyName );

        /* Frees|unlinks all related resources of the currently wrapped body */
        ~SimBody() {}

        /* Updates all internal information by quering the backend */
        void update() {}

        /* Returns the unique name of the wrapped body */
        std::string name() { return m_bodyName; }

        /* Returns all meshes linked to each geometry */
        std::vector< engine::LIRenderable* > geomsGraphics() { return m_geomsGraphics; }

        /* Resets the body to some configuration */
        void reset() {}

        /* Prints some debug information */
        void print() {}

        /* Returns the world-position of the body */
        tysoc::TVec3 position() { return m_bodyWorldPos; }

        /* Returns the world-orientation of the body */
        tysoc::TMat3 orientation() { return m_bodyWorldRot; }

        /* Returns the world transform of the body */
        tysoc::TMat4 worldTransform() { return m_bodyWorldTransform; }
    };

    class ITestApplication
    {

        protected :

        std::shared_ptr<dart::simulation::World> m_dartWorldPtr;

        engine::LApp* m_graphicsApp;
        engine::LScene* m_graphicsScene;

        std::vector< SimBody* >             m_simBodies;
        std::map< std::string, SimBody* >   m_simBodiesMap;

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

        /* Returns a body-wrapper of a body with a specific name in the simulation */
        SimBody* getBody( const std::string& name );
    };

}