# Build

```powershell
g++ --std=c++17 -O3 -fopenmp -march=native src/main.cpp -o build/main
```

# Run

```powershell
build/main > out/result.ppm
```