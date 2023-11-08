CXX = g++
CXXFLAGS = `pkg-config --cflags opencv4`
LDLIBS = `pkg-config --libs opencv4` -ltiff

motion_planning: env.cpp
	$(CXX) $(CXXFLAGS) -o motion_planning main.cpp rrt_connect.cpp collision.cpp env.cpp robot.cpp $(LDLIBS)