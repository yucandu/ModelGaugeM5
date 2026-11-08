# Host-side tests: no Arduino toolchain needed.
CXX ?= g++
CXXFLAGS ?= -std=gnu++11 -Wall -Wextra -Wpedantic -g -Imock -I../src

test: test_gauge
	./test_gauge

test_gauge: test_gauge.cpp ../src/ModelGaugeM5.cpp ../src/ModelGaugeM5_Features.cpp $(wildcard ../src/*.h) mock/Arduino.h mock/Wire.h
	$(CXX) $(CXXFLAGS) -o $@ test_gauge.cpp ../src/ModelGaugeM5.cpp ../src/ModelGaugeM5_Features.cpp

# Also build with a newer standard to catch C++11/17 differences
test17:
	$(MAKE) CXXFLAGS="-std=gnu++17 -Wall -Wextra -g -Imock -I../src" test

clean:
	rm -f test_gauge
