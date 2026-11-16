CXX      = g++
CXXFLAGS = -Wall -O3
LIBS     = -lsfml-graphics -lsfml-window -lsfml-system -lnlopt
TARGET   = app
SRC      = main.cpp
# весь код в заголовках, поэтому пересобираем при изменении любого из них
HEADERS  = $(shell find include -name '*.h')

.PHONY: all run clean init bench multi

all: $(TARGET)

$(TARGET): $(SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LIBS)

run: $(TARGET)
	./$(TARGET)

# бенчмарк планировщиков на случайных сценах: make bench N=1000 SEED=42 MODE=normal (или tight, moving)
N    ?= 1000
SEED ?= 42
MODE ?= normal
bench-app: bench.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) bench.cpp -o bench-app $(LIBS)

bench: bench-app
	./bench-app $(N) $(SEED) $(MODE)

# симуляция нескольких своих роботов: make multi N=200 SEED=42 SCENARIO=swap (или random, cross, zone).
# Библиотеки не нужны: sampler.h не тянет SFML и nlopt
SCENARIO ?= swap
multi-app: multi.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) multi.cpp -o multi-app

multi: multi-app
	./multi-app $(if $(filter 1000,$(N)),200,$(N)) $(SEED) $(SCENARIO)

clean:
	rm -f $(TARGET) bench-app multi-app

# зависимости для Ubuntu/Debian
init:
	sudo apt install -y g++ libsfml-dev libnlopt-dev
