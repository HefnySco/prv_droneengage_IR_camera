CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g $(shell pkg-config --cflags opencv4)
LIBS = $(shell pkg-config --libs opencv4)
TARGETS = test_hotspot

.PHONY: all clean test

all: $(TARGETS)

test_hotspot: test_hotspot.cpp ../src/ir_camera/hotspot_detector.hpp
	$(CXX) $(CXXFLAGS) -o $@ test_hotspot.cpp $(LIBS)

test: $(TARGETS)
	./test_hotspot

clean:
	rm -f $(TARGETS)
