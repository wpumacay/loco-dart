
#include <test_interface.h>

class AppExample : public dart::ITestApplication
{
    protected :

    void _initScenarioInternal() override;

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

int main()
{

    auto _app = new AppExample();

    _app->init();
    _app->reset();

    while ( !_app->isTerminated() )
        _app->step();

    delete _app;

    return 0;
}