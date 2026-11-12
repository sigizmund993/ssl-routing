CXX      = g++
CXXFLAGS = -Wall -O3
LIBS     = -lsfml-graphics -lsfml-window -lsfml-system -lnlopt
TARGET   = app
SRC      = main.cpp
# весь код в заголовках, поэтому пересобираем при изменении любого из них
HEADERS  = $(shell find include -name '*.h')

.PHONY: all run clean init bench

all: $(TARGET)

$(TARGET): $(SRC) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LIBS)

run: $(TARGET)
	./$(TARGET)

# бенчмарк минимизации на случайных сценах: make bench N=1000 SEED=42
N    ?= 1000
SEED ?= 42
bench-app: bench.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) bench.cpp -o bench-app $(LIBS)

bench: bench-app
	./bench-app $(N) $(SEED)

clean:
	rm -f $(TARGET) bench-app

# зависимости для Ubuntu/Debian
init:
	sudo apt install -y g++ libsfml-dev libnlopt-dev
