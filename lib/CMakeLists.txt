cmake_minimum_required(VERSION 3.20.0)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
	Vector
	GIT_REPOSITORY
		git@github.com:Dinamiet/Vector.git
	GIT_TAG
		main
	SOURCE_DIR
		${CMAKE_SOURCE_DIR}/lib/Vector
)

FetchContent_Declare(
	Quaternion
	GIT_REPOSITORY
		git@github.com:Dinamiet/Quaternion.git
	GIT_TAG
		main
	SOURCE_DIR
		${CMAKE_SOURCE_DIR}/lib/Quaternion
)

FetchContent_Declare(
	PID
	GIT_REPOSITORY
		git@github.com:Dinamiet/PID.git
	GIT_TAG
		main
	SOURCE_DIR
		${CMAKE_SOURCE_DIR}/lib/PID
)


FetchContent_MakeAvailable(
	Vector
	Quaternion
	PID
)
