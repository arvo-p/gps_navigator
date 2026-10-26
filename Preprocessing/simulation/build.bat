@echo off
setlocal

echo ===================================================
echo  Building GPS Embedded Simulation (Raylib + C)
echo ===================================================

set CC=gcc
set CFLAGS=-std=c99 -Wall -Wextra -O2 -Iinclude -Isrc
set LDFLAGS=-lraylib -lopengl32 -lgdi32 -lwinmm -lm
set SRC=src/main.c src/geo_math.c src/map_camera.c src/tile_cache.c src/ui.c src/astar_heap.c src/tile_graph_cache.c src/pathfinder.c
set OUT=gps_simulation.exe

echo Compiling source files:
echo %CC% %CFLAGS% %SRC% -o %OUT% %LDFLAGS%

%CC% %CFLAGS% %SRC% -o %OUT% %LDFLAGS%

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ===================================================
    echo  Build successful! Output: %OUT%
    echo ===================================================
) else (
    echo.
    echo [ERROR] Build failed with error code %ERRORLEVEL%
)

endlocal
