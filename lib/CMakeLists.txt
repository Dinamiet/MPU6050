cmake_minimum_required(VERSION 3.20.0)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
	Vector
	GIT_REPOSITORY
		git@bitbucket.org:dinamiet/vector.git
	GIT_TAG
		development
	SOURCE_DIR
		${CMAKE_SOURCE_DIR}/lib/Vector
)

FetchContent_Declare(
	Quaternion
	GIT_REPOSITORY
		git@bitbucket.org:dinamiet/quaternion.git
	GIT_TAG
		development
	SOURCE_DIR
		${CMAKE_SOURCE_DIR}/lib/Quaternion
)


FetchContent_MakeAvailable(
	Vector
	Quaternion
)
