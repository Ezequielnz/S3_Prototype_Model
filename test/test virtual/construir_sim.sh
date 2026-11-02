#!/bin/bash
# Extrae del .ino las secciones independientes del hardware y compila el simulador de PC.
set -e
cd "$(dirname "$0")"
INO=../firmware/datalyx_nodo/datalyx_nodo.ino
python3 - "$INO" > nodo_extraido.inc <<'EOF'
import sys
L = open(sys.argv[1], encoding='utf-8').read().split('\n')
out, keep, n_mpu, n_red = [], False, 0, 0
for line in L:
    if line.startswith('// =='):
        if 'PINES' in line: keep = True
        elif 'MPU6050' in line:
            n_mpu += 1
            if n_mpu == 2: keep = False
        elif 'CRUDO: CU' in line: keep = True
        elif ' RED ' in line:
            n_red += 1
            if n_red == 2: keep = False
        elif 'SETUP' in line: keep = False
    if keep: out.append(line)
print('#include "../firmware/datalyx_nodo/dlx_proto.h"')
print('#include "../firmware/datalyx_nodo/dlx_dsp.h"')
print('\n'.join(out))
EOF
g++ -std=c++17 -O2 -Wall -Wno-unused-function -Wno-unused-variable -o sim_nodo sim_nodo.cpp -lm
echo "OK: ./sim_nodo [segundos] [tiempo_real 0/1]"
