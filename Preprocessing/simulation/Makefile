CC ?= gcc

# Detect Operating System
ifeq ($(OS),Windows_NT)
    PLATFORM := WINDOWS
    TARGET := gps_simulation.exe
    TEST_RUNNER := test_runner.exe
    TEST_PF := test_pathfinder.exe
    CFLAGS ?= -std=c99 -Wall -Wextra -O2 -Iinclude -Isrc
    LDFLAGS ?= -lraylib -lopengl32 -lgdi32 -lwinmm -lm
    RM := del /Q /F
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        PLATFORM := MACOS
        TARGET := gps_simulation
        TEST_RUNNER := test_runner
        TEST_PF := test_pathfinder
        CFLAGS ?= -std=c99 -Wall -Wextra -O2 -Iinclude -Isrc
        LDFLAGS ?= -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -lm
        RM := rm -f
    else
        PLATFORM := LINUX
        TARGET := gps_simulation
        TEST_RUNNER := test_runner
        TEST_PF := test_pathfinder
        RM := rm -f

        # Raylib configuration: Check pkg-config, system headers, or local deps/raylib
        RAYLIB_PKG := $(shell pkg-config --exists raylib 2>/dev/null && echo "yes")
        ifeq ($(RAYLIB_PKG),yes)
            RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib)
            RAYLIB_LIBS   := $(shell pkg-config --libs raylib)
        else ifneq ($(wildcard /usr/include/raylib.h),)
            RAYLIB_CFLAGS :=
            RAYLIB_LIBS   := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
        else ifneq ($(wildcard deps/raylib/include/raylib.h),)
            RAYLIB_CFLAGS := -Ideps/raylib/include
            RAYLIB_LIBS   := deps/raylib/lib/libraylib.a -lGL -lm -lpthread -ldl -lrt -lX11
        else
            RAYLIB_CFLAGS :=
            RAYLIB_LIBS   := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
        endif

        CFLAGS ?= -std=c99 -Wall -Wextra -O2 -Iinclude -Isrc $(RAYLIB_CFLAGS)
        LDFLAGS ?= $(RAYLIB_LIBS)
    endif
endif

SRC = src/main.c src/geo_math.c src/map_camera.c src/tile_cache.c src/ui.c src/astar_heap.c src/tile_graph_cache.c src/pathfinder.c
OBJ = $(SRC:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

test: $(TEST_RUNNER) $(TEST_PF)
	./$(TEST_RUNNER)
	./$(TEST_PF)
	$(RM) $(TEST_RUNNER) $(TEST_PF)

$(TEST_RUNNER): tests/test_universal_algo.c src/geo_math.c src/map_camera.c
	$(CC) -std=c99 -Wall -Wextra -O2 -Iinclude -Isrc tests/test_universal_algo.c src/geo_math.c src/map_camera.c -o $@ -lm

$(TEST_PF): tests/test_pathfinder.c src/geo_math.c src/tile_graph_cache.c src/astar_heap.c src/pathfinder.c
	$(CC) -std=c99 -Wall -Wextra -O2 -Iinclude -Isrc tests/test_pathfinder.c src/geo_math.c src/tile_graph_cache.c src/astar_heap.c src/pathfinder.c -o $@ -lm

clean:
	$(RM) $(OBJ) $(TARGET) gps_simulation gps_simulation.exe $(TEST_RUNNER) $(TEST_PF)

.PHONY: all clean test
