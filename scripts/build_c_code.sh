#!/bin/bash

# Compile the C files into a shared library
gcc -shared -o src/c_code/control_system.so -fPIC src/c_code/error_calculations.c src/c_code/matrix_and_vector_ops.c src/c_code/control_simulation.c src/c_code/utilities.c src/c_code/control_system.c src/c_code/real_system.c
# gcc -shared -o src/c_code/control_system.exe -fPIC src/c_code/error_calculations.c src/c_code/matrix_and_vector_ops.c src/c_code/control_simulation.c src/c_code/utilities.c src/c_code/control_system.c src/c_code/real_system.c
