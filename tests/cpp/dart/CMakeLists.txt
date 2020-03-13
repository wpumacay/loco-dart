message( "LOCO::DART::tests::cpp::dart >>> Configuring C/C++ loco-dart-specific tests" )

include_directories( "${LOCO_DART_INCLUDE_DIRS}" )

function( FcnBuildDartTestSpecific pSourcesList pExecutableName )
    add_executable( ${pExecutableName} ${pSourcesList} )
    target_link_libraries( ${pExecutableName} locoPhysicsDART loco_core gtest_main )
    add_test( NAME "${pExecutableName}_test" COMMAND "${pExecutableName}" )
    target_compile_definitions( ${pExecutableName} PRIVATE UNIQUE_PTR_EXTENSION=1 )
endfunction()

FILE( GLOB TestDartSources *.cpp )

foreach( testDartFile ${TestDartSources} )
    string( REPLACE ".cpp" "" executableLongName ${testDartFile} )
    get_filename_component( execName ${executableLongName} NAME )
    FcnBuildDartTestSpecific( ${testDartFile} ${execName} )
endforeach( testDartFile )

