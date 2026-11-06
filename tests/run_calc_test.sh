#!/usr/bin/env bash
# Compares the component's temperature maths with the Melexis reference driver.
# Needs: bash, curl, gcc and g++. Usage: tests/run_calc_test.sh
set -euo pipefail
cd "$(dirname "$0")"

# Pinned Melexis mlx90640-library commit (Apache-2.0)
MELEXIS_COMMIT=f6be7ca1d4a55146b705f3d347f84b773b29cc86
REF=build/melexis
BASE="https://raw.githubusercontent.com/melexis/mlx90640-library/${MELEXIS_COMMIT}"

mkdir -p "$REF"
for f in functions/MLX90640_API.c headers/MLX90640_API.h headers/MLX90640_I2C_Driver.h; do
  [ -f "$REF/$(basename "$f")" ] || curl -sSfL -o "$REF/$(basename "$f")" "$BASE/$f"
done

gcc -std=c99 -O2 -c -I"$REF" "$REF/MLX90640_API.c" -o build/melexis_api.o
g++ -std=c++17 -O2 -Wall -Wextra -I"$REF" \
  calc_test.cpp ../components/mlx90640/mlx90640_calc.cpp build/melexis_api.o -lm -o build/calc_test
./build/calc_test
