cmake_minimum_required(VERSION 3.4)
project(franka_control
    VERSION 0.8.0
    LANGUAGES CXX
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
# # find_package(Franka 0.8.0 REQUIRED)

add_library(panda_library SHARED
    src/joint_states.cpp
)

add_library(panda::library ALIAS panda_library)

target_include_directories(panda_library
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)


add_executable(joint_states
    src/joint_states_main.cpp
)

# # target_link_libraries(franka_control
# #     PRIVATE
# #     Franka::Franka
# # )

target_link_libraries(joint_states 
    PRIVATE
    panda::library
    Eigen3::Eigen
    ## libfranka-common
)


### ****************************

# cmake_minimum_required(VERSION 3.4)
# project(franka_control
#     VERSION 0.8.0
#     LANGUAGES CXX
# )

# list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# if(MSVC)
#   add_compile_options(/W0)
# else()
#   add_compile_options(-Wall -Wextra)
# endif()

# set(THIRDPARTY_SOURCES_DIR "${CMAKE_SOURCE_DIR}/3rdparty" CACHE PATH
#   "Directory for third-party sources")

# ## Dependencies
# find_package(Poco REQUIRED COMPONENTS Net Foundation)
# find_package(Eigen3 REQUIRED)

# set(THREADS_PREFER_PTHREAD_FLAG ON)
# find_package(Threads REQUIRED)

# ## Build options
# option(STRICT "Treat warnings as errors" OFF)
# if(STRICT)
#   if(MSVC)
#     add_compile_options(/WX)
#   else()
#     add_compile_options(-Werror)
#   endif()
# endif()

# option(BUILD_COVERAGE "Build with code coverage" OFF)
# if(BUILD_COVERAGE)
#   add_compile_options(--coverage)
#   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
#   set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
# endif()

# ## Submodules
# ## add_subdirectory(common)

# add_library(libfranka-common INTERFACE)
# set_target_properties(libfranka-common PROPERTIES
#   INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/include
# )

# ## find_package(Poco REQUIRED COMPONENTS Net Foundation)
# find_package(Eigen3 REQUIRED)
# # # find_package(Franka 0.8.0 REQUIRED)

# add_library(panda_library SHARED
#     src/joint_states.cpp
# )

# add_library(panda::library ALIAS panda_library)

# target_include_directories(panda_library
#     PUBLIC
#         ${PROJECT_SOURCE_DIR}/include
# )


# add_executable(joint_states
#     src/joint_states_main.cpp
# )

# # # target_link_libraries(franka_control
# # #     PRIVATE
# # #     Franka::Franka
# # # )

# target_link_libraries(joint_states 
#     PRIVATE
#     panda::library
#     Eigen3::Eigen
#     ## libfranka-common
# )