message( "LOCO::DART::tests::cpp >>> Configuring C/C++ loco-dart tests" )

include_directories( "${LOCO_DART_INCLUDE_DIRS}" )

function( FcnBuildDartTest pSourcesList pExecutableName )
    add_executable( ${pExecutableName} ${pSourcesList} )
    target_link_libraries( ${pExecutableName} loco_core gtest_main )
    add_test( NAME "${pExecutableName}_test" COMMAND "${pExecutableName}" )
endfunction()

FILE( GLOB TestDartSources *.cpp )

foreach( testDartFile ${TestDartSources} )
    string( REPLACE ".cpp" "" executableLongName ${testDartFile} )
    get_filename_component( execName ${executableLongName} NAME )
    FcnBuildDartTest( ${testDartFile} ${execName} )
endforeach( testDartFile )