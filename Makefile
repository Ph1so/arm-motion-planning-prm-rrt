CXX      = g++
CXXFLAGS = -O2 -std=c++17

all: build/planner build/verifier

build/planner: src/planner.cpp | build
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

build/verifier: src/verifier.cpp | build
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: all clean
