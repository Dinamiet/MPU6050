cmake_minimum_required(VERSION 3.20.0)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
	Template
	GIT_REPOSITORY
		git@bitbucket.org:dinamiet/template.git
	GIT_TAG
		master
	SOURCE_DIR
		${CMAKE_SOURCE_DIR}/lib/Template
)

FetchContent_MakeAvailable(
	# Template
)
