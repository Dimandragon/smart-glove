file(GLOB SOURCE_FILES
        src/*.cpp
        *.cpp 
        smart-glove-proto/cpp/glove.pb.cc
        fusion/Fusion/*.c
    )

idf_component_register(SRCS 
                        ${SOURCE_FILES}
                        INCLUDE_DIRS "include"
                        "smart-glove-proto/cpp")

                        
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++23 -Wno-format -Wno-missing-field-initializers -Wno-write-strings -Wno-return-type -Wno-reorder")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format -Wno-missing-field-initializers -Wno-write-strings -fexceptions -Wno-return-type -Wno-reorder)