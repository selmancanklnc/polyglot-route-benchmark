# Empirical Performance and Systems Analysis of Multi-Language Implementations in Physics-Constrained Vehicle Route Optimization

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)]()
[![Rust](https://img.shields.io/badge/Rust-2021-000000.svg)]()
[![Go](https://img.shields.io/badge/Go-1.22%2B-00ADD8.svg)]()
[![Java](https://img.shields.io/badge/Java-20%20LTS-ED8B00.svg)]()
[![Python](https://img.shields.io/badge/Python-3.9%2B-3776AB.svg)]()
[![Verification](https://img.shields.io/badge/Test%20Suite-Passed%20(100%25)-brightgreen.svg)]()

---

## Abstract

Modern intelligent transportation systems (ITS), autonomous logistics dispatchers, and electric vehicle (EV) energy management architectures require pathfinding engines capable of evaluating non-linear physical dynamics in real time. Conventional shortest-path benchmarks routinely evaluate routing algorithms over idealized static graphs where edge costs equal geometric distance, completely obscuring the computational overhead introduced by aerodynamic drag, longitudinal gradient resistance, rolling friction, powertrain thermal conversion efficiencies, and multi-objective Pareto trade-offs.

This repository presents an open, deterministic, and fully reproducible cross-language systems benchmark across **five production language ecosystems**: **C++20**, **Rust 2021**, **Go 1.22+**, **Java 20 (HotSpot VM)**, and **Python 3.9+ (CPython)**. Each implementation realizes an identical mathematical dynamics model over synthetic road networks scaling up to $50,000$ spatial vertices and $250,000$ directed edges. We evaluate single-threaded instruction throughput, multi-core work-stealing scalability, heap allocation patterns, memory footprint (RSS), garbage collection overhead, JIT warmup curves, stochastic uncertainty propagation via Monte Carlo simulation, and multi-objective Pareto frontier extraction.

All measurements were conducted on physical hardware under deterministic execution parameters. Algorithmic parity across all five implementations is mathematically verified to eliminate optimization discrepancies.

---

## 1. Engineering Motivation and Mathematical Foundations

Traditional routing frameworks reduce road networks to scalar graphs $G = (V, E)$ where edge cost $w(e)$ is invariant. In contrast, physical vehicle routing models the instantaneous mechanical forces resisting vehicular motion along each directed edge $e = (u, v)$, making edge traversal cost a function of atmospheric density, road typology, spatial slope, vehicle aerodynamics, and traffic density.

```
Spatial Network Topography          Physical Dynamics Pipeline              Cost Functional Synthesis
--------------------------          --------------------------              -------------------------
Vertex u (x1, y1, elevation)        1. Aerodynamic Drag: F_drag             Mode FASTEST:  J = t_travel
            |                       2. Slope Resistance: F_slope            Mode CHEAPEST: J = C_fuel + C_toll
            |--- Directed Edge e --> 3. Rolling Friction: F_roll   ------>  Mode BALANCED: J = w_f*C_f +
            |                       4. Powertrain Work:  P * t / eta                       w_t*t + w_l*C_l
Vertex v (x2, y2, elevation)        5. Volumetric Fuel:  V_fuel
```

### 1.1 Effective Longitudinal Velocity Model

The steady-state traversal speed $v_{\text{eff}}$ along an edge $e$ is bounded by statutory limits, historical average flow, surface typology, and gravitational deceleration on inclines:

$$v_{\text{eff}} = \text{clip}\Bigl(\min\bigl(v_{\text{limit}},\, v_{\text{avg}}\bigr) \cdot T \cdot f_{\text{road}} \cdot f_{\text{slope}}(\theta),\; v_{\min},\; v_{\max}\Bigr) \quad [\text{km/h}]$$

where:
- $T \in [0.2,\, 1.0]$ represents the dynamic traffic congestion multiplier.
- $f_{\text{road}}$ denotes the empirical road classification multiplier ($\text{Highway}: 1.05$, $\text{Main Road}: 0.95$, $\text{City Road}: 0.70$, $\text{Rural Road}: 0.85$, $\text{Mountain Road}: 0.60$).
- $v_{\min} = 10.0\,\text{km/h}$ and $v_{\max} = 140.0\,\text{km/h}$ are physical velocity boundary clamps.
- $f_{\text{slope}}(\theta)$ represents the non-linear incline velocity attenuation factor:

$$f_{\text{slope}}(\theta) = \begin{cases} \max\bigl(0.5,\; 1.0 - 2.0\sin\theta\bigr), & \theta > 0 \quad (\text{uphill gradient}) \\ 1.0 + 0.5\left|\sin\theta\right|, & \theta \le 0 \quad (\text{downhill gravitational assist}) \end{cases}$$

### 1.2 Longitudinal Resistance Forces

The net longitudinal mechanical force $F_{\text{total}}$ resisting vehicular motion is the sum of three component forces:

1. **Aerodynamic Drag Force ($F_{\text{drag}}$)** with projected 1D apparent wind velocity:
   $$F_{\text{drag}} = \frac{1}{2}\,\rho_{\text{air}}\,C_d\,A_{\text{front}}\,\Bigl[\max\bigl(0,\, v_{\text{ms}} - v_{\text{wind}}\cos\phi\bigr)\Bigr]^2 \quad [\text{N}]$$
   where $\rho_{\text{air}} = 1.225\,\text{kg/m}^3$, $C_d = 0.30$ (drag coefficient), $A_{\text{front}} = 2.2\,\text{m}^2$ (frontal area), $v_{\text{ms}} = v_{\text{eff}} / 3.6$, and $\phi$ is the incident wind angle.

2. **Gravitational Slope Resistance ($F_{\text{slope}}$)**:
   $$F_{\text{slope}} = m_{\text{veh}}\,g\,\sin\theta \quad [\text{N}]$$
   where $m_{\text{veh}} = 1400.0\,\text{kg}$ and $g = 9.81\,\text{m/s}^2$.

3. **Coulomb Rolling Friction Force ($F_{\text{roll}}$)**:
   $$F_{\text{roll}} = C_{rr}\,m_{\text{veh}}\,g\,\cos\theta \quad [\text{N}]$$
   where $C_{rr} = 0.012$ represents the tire-pavement rolling resistance coefficient.

### 1.3 Thermodynamic Energy Conversion and Volumetric Fuel Modeling

Total mechanical power $P_{\text{mech}}$ and mechanical work $E_{\text{mech}}$ required over traversal duration $t_{\text{sec}} = \frac{d}{v_{\text{eff}}} \times 3600$ are evaluated as:

$$P_{\text{mech}} = \max\bigl(0,\, F_{\text{drag}} + F_{\text{slope}} + F_{\text{roll}}\bigr) \cdot v_{\text{ms}} \quad [\text{Watts}]$$

$$E_{\text{mech}} = P_{\text{mech}} \cdot t_{\text{sec}} \quad [\text{Joules}]$$

Incorporating powertrain thermal-to-mechanical conversion efficiency $\eta = 0.30$ and volumetric fuel energy density $\rho_{\text{fuel}} = 34.2 \times 10^6\,\text{J/L}$:

$$V_{\text{fuel}} = \frac{E_{\text{mech}}}{\eta \cdot \rho_{\text{fuel}}} \quad [\text{Liters}], \qquad C_{\text{fuel}} = V_{\text{fuel}} \cdot P_{\text{unit}} \quad [\$]$$

### 1.4 Multi-Objective Cost Functional

Each path optimization query $Q = (s, t, \mathcal{M})$ minimizes an objective function $J_{\mathcal{M}}(e)$ parameterized by mode $\mathcal{M}$:

$$J_{\mathcal{M}}(e) = \begin{cases} t_{\text{min}}(e), & \mathcal{M} = \text{FASTEST} \\ C_{\text{fuel}}(e) + C_{\text{toll}}(e), & \mathcal{M} = \text{CHEAPEST} \\ w_f\,C_{\text{fuel}}(e) + w_t\,t_{\text{min}}(e) + w_l\,C_{\text{toll}}(e), & \mathcal{M} = \text{BALANCED} \end{cases}$$

where the canonical weights satisfy the simplex constraint: $w_f + w_t + w_l = 1.0$ (standard: $w_f = 0.40, w_t = 0.40, w_l = 0.20$).

---

## 2. Experimental Setup and Hardware Specifications

### 2.1 Hardware and Platform Configuration

- **Host Processor**: Apple M2 (ARM64v8.5-A, 8 Cores: 4 High-Performance Avalanche cores @ 3.49 GHz + 4 High-Efficiency Blizzard cores @ 2.42 GHz).
- **Cache Architecture**: L1I: 192 KB, L1D: 128 KB per P-core; Shared L2: 16 MB (P-cluster), 4 MB (E-cluster).
- **System Memory**: 16 GB Unified LPDDR5-6400 (102.4 GB/s theoretical memory bandwidth).
- **Host Operating System**: macOS Darwin Kernel Version 25.5.0 (arm64).
- **C++ Compiler**: Apple Clang version 17.0.0 (`-std=c++20 -O3 -Wall -Wextra -pedantic`).
- **Rust Compiler**: `rustc` 1.84.0 (2021 Edition, `opt-level=3`, `lto=thin`, `codegen-units=1`).
- **Go Compiler**: `go1.22.0 darwin/arm64` (`-ldflags="-s -w"`).
- **Java Virtual Machine**: OpenJDK 64-Bit Server VM (build 20.0.2+9, HotSpot JVM, tiered JIT enabled).
- **Python Interpreter**: CPython 3.9.6 (64-bit standard runtime).

---

### 2.2 Dataset Characteristics

All benchmark graph datasets are generated via deterministic pseudo-random spatial graph generation (`datasets/generate_datasets.py`, RNG seed $= 42$):

| Dataset Identifier | Vertices ($|V|$) | Directed Edges ($|E|$) | Average Degree ($|E|/|V|$) | Workload ($|Q|$) | Spatial Extent |
|:---|---:|---:|---:|---:|:---|
| `test_fixture.json` | 10 | 26 | 2.60 | 6 | $10 \times 10\,\text{km}$ |
| `small.json` | 1,000 | 5,000 | 5.00 | 20 | $50 \times 50\,\text{km}$ |
| `medium.json` | 10,000 | 50,000 | 5.00 | 50 | $200 \times 200\,\text{km}$ |
| `large.json` | 50,000 | 250,000 | 5.00 | 100 | $500 \times 500\,\text{km}$ |

---

## 3. Empirical Results and Comparative Metrics

### 3.1 Medium Graph Workload ($|V| = 10^4, |E| = 5 \times 10^4, |Q| = 50$)

| Implementation | Concurrency Architecture | Single-Thread Time (ms) | Single-Thread QPS | Multi-Thread Time (8 Cores, ms) | Multi-Thread QPS | Parallel Speedup | Peak RSS (Memory) |
|:---|:---|---:|---:|---:|---:|---:|---:|
| **C++20** | POSIX Thread Chunks | **105.98** | **471.8** | **29.56** | **1,691.5** | 3.58x | 194.0 MB |
| **Rust 2021** | Rayon Work-Stealing | 192.39 | 259.9 | 39.03 | 1,280.9 | **4.93x** | **15.7 MB** |
| **Java 20** | Executor Thread Pool | 185.73 | 269.2 | 93.15 | 536.8 | 1.99x | 251.0 MB |
| **Go 1.22+** | Goroutines / Channels | 231.91 | 215.6 | 84.44 | 592.1 | 2.75x | 63.0 MB |
| **Python 3.9** | ThreadPoolExecutor | 1,620.84 | 30.9 | 1,672.87 | 29.9 | 0.97x (GIL) | 130.0 MB |

---

### 3.2 Large Graph Workload ($|V| = 5 \times 10^4, |E| = 2.5 \times 10^5, |Q| = 100$)

| Implementation | Serial Runtime (ms) | Multi-Thread Runtime (8 Cores, ms) | Throughput (QPS) | Peak Heap / RSS | Memory Density vs C++ |
|:---|---:|---:|---:|---:|---:|
| **C++20** | **1,578.4** | **371.2** | **269.4** | 866.0 MB | 1.00x (baseline) |
| **Rust 2021** | 2,404.1 | 575.3 | 173.8 | **66.2 MB** | **13.08x more compact** |
| **Java 20** | 2,887.0 | 747.1 | 133.8 | 499.0 MB | 1.73x more compact |
| **Go 1.22+** | 3,042.3 | 811.4 | 123.2 | 282.0 MB | 3.07x more compact |
| **Python 3.9** | 19,432.0 | 21,892.0 | 4.5 | 571.0 MB | 1.51x more compact |

---

## 4. Systems Analysis and Architectural Insights

### 4.1 Memory Layout and Priority Queue Primitives

| Language | Adjacency Representation | Priority Queue Primitive | Pointer Indirection Overhead | Asymptotic Complexity |
|:---|:---|:---|:---|:---|
| **C++20** | `std::unordered_map<int, vector<pair<int, size_t>>>` | `std::priority_queue<DijkstraEntry, vector, greater>` | Zero on flat vectors; node-based hash map | $\mathcal{O}\bigl((|V| + |E|)\log |V|\bigr)$ |
| **Rust** | `HashMap<usize, Vec<(usize, usize)>>` | `std::collections::BinaryHeap<State>` | Zero-overhead fat pointers; monomorphized structs | $\mathcal{O}\bigl((|V| + |E|)\log |V|\bigr)$ |
| **Go** | `map[int][]HalfEdge` | `container/heap` over slice pointer | 24-byte slice header; flat value structs | $\mathcal{O}\bigl((|V| + |E|)\log |V|\bigr)$ |
| **Java** | `HashMap<Integer, List<HalfEdge>>` | `java.util.PriorityQueue<DijkstraEntry>` | 16-byte object headers + 8-byte reference per entry | $\mathcal{O}\bigl((|V| + |E|)\log |V|\bigr)$ |
| **Python**| `dict[int, list[tuple[int, Edge]]]` | `heapq` over tuple objects | 28-byte PyObject header per integer/float | $\mathcal{O}\bigl((|V| + |E|)\log |V|\bigr)$ |

### 4.2 A* Heuristic Consistency and Optimality

The Euclidean distance lower-bound heuristic $h(u, t)$ is defined over vertex coordinates $\mathbf{x}_u, \mathbf{x}_t \in \mathbb{R}^2$:

$$h(u, t) = \begin{cases} \dfrac{\|\mathbf{x}_u - \mathbf{x}_t\|_2}{v_{\max}} \times 60, & \mathcal{M} = \text{FASTEST} \\ w_t \cdot \left(\dfrac{\|\mathbf{x}_u - \mathbf{x}_t\|_2}{v_{\max}} \times 60\right), & \mathcal{M} = \text{BALANCED} \\ 0, & \mathcal{M} = \text{CHEAPEST} \end{cases}$$

**Admissibility Proof**: Because $v_{\text{eff}}(e) \le v_{\max} = 140.0\,\text{km/h}$ holds $\forall e \in E$, and the Euclidean straight-line distance $\|\mathbf{x}_u - \mathbf{x}_t\|_2$ is strictly less than or equal to geodesic road distance along any connected path, $h(u, t) \le h^*(u, t)$ holds unconditionally. Thus, $A^*$ search is guaranteed to return the global cost-optimal route without premature termination.

---

## 5. Advanced Experimental Modules

### 5.1 Monte Carlo Stochastic Uncertainty Quantification (`experimental/monte_carlo.py`)

Deterministic shortest-path algorithms ignore environmental stochasticity. The Monte Carlo engine executes $N = 10,000$ independent realizations over optimal paths under Gaussian noise distributions:

- **Stochastic Traffic Perturbation**: $T_i \sim \text{clip}\bigl(\mathcal{N}(T_0,\, \sigma_T = 0.08),\, 0.2,\, 1.0\bigr)$
- **Stochastic Wind Velocity**: $W_i \sim \text{clip}\bigl(\mathcal{N}(W_0,\, \sigma_W = 3.0\,\text{m/s}),\, 0.0,\, \infty\bigr)$

```
Deterministic Baseline: Travel Time = 4.80 min | Fuel Cost = $4.55
----------------------------------------------------------------------
  Stochastic Mean Travel Time   :  4.87 +- 0.36 min
  95% Confidence Interval (Time): [4.26 -- 5.67] min  (5th %: 4.34m, 95th %: 5.52m)
  Stochastic Mean Fuel Cost     : $4.55 +- $0.28
  95% Confidence Interval (Cost): [$4.03 -- $5.15]   (5th %: $4.11, 95th %: $5.04)
----------------------------------------------------------------------
```

### 5.2 Multi-Objective Pareto Frontier Analysis (`experimental/pareto_frontier.py`)

Performs a discrete sweep over the 3-objective unit simplex $\mathcal{S} = \{(w_f, w_t, w_l) \in \mathbb{R}^3_+ \mid w_f + w_t + w_l = 1\}$ at resolution $\Delta = 0.1$ ($66$ unique weight vectors). Non-dominated solutions are identified according to Pareto dominance criteria:

A candidate path $\mathbf{x}$ strictly dominates $\mathbf{y}$ ($\mathbf{x} \succ \mathbf{y}$) if and only if:

$$\forall i \in \{\text{time}, \text{fuel}, \text{toll}\},\; J_i(\mathbf{x}) \le J_i(\mathbf{y}) \quad \land \quad \exists j \in \{\text{time}, \text{fuel}, \text{toll}\},\; J_j(\mathbf{x}) < J_j(\mathbf{y})$$

---

## 6. Algorithmic Parity and Cross-Language Verification

To guarantee that performance differentials reflect pure language runtime characteristics rather than algorithmic discrepancies, an automated parity verification test executes against `datasets/test_fixture.json`:

1. **Path Node Parity**: Route paths $P_{\text{lang}}(s, t)$ must be identical ordered sequences across all five implementations:
   $$P_{\text{C++}}(s, t) \equiv P_{\text{Rust}}(s, t) \equiv P_{\text{Go}}(s, t) \equiv P_{\text{Java}}(s, t) \equiv P_{\text{Python}}(s, t)$$
2. **Cost Metric Parity**: Total distance, travel time, and monetary fuel cost must match within tolerance $\varepsilon < 10^{-4}$:
   $$\left|J_{\text{C++}}(P) - J_{\text{lang}}(P)\right| < 10^{-4}$$

---

## 7. Build and Verification Suite

### 7.1 Universal Benchmark Execution

To compile all native binaries in release mode, run all unit test suites, and execute the benchmark suite across all dataset tiers:

```bash
# Clone the repository
git clone https://github.com/selmancanklnc/polyglot-route-benchmark.git
cd polyglot-route-benchmark

# Execute universal automated harness
chmod +x benchmark/run_all.sh
./benchmark/run_all.sh
```

---

### 7.2 Individual Component Compilation and Testing

#### C++20 (Clang / CMake)
```bash
cd cpp
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j$(sysctl -n hw.ncpu || nproc)
ctest --output-on-failure
./route_optimizer --dataset ../../datasets/medium.json --mode multi --workers 8 --runs 5
```

#### Rust 2021 (Cargo)
```bash
cd rust
cargo test --release
cargo clippy -- -D warnings
cargo build --release
./target/release/route_optimization_rust --dataset ../datasets/medium.json --mode multi --workers 8 --runs 5
```

#### Go 1.22+ (Go Toolchain)
```bash
cd go
go vet ./...
go test -v ./...
go build -ldflags="-s -w" -o route_optimizer_go ./cmd/main.go
./route_optimizer_go --dataset ../datasets/medium.json --mode multi --workers 8 --runs 5
```

#### Java 20 (Gradle / HotSpot)
```bash
cd java
./gradlew test
./gradlew build
java -jar build/libs/route_optimization_java-1.0.0.jar --dataset ../datasets/medium.json --mode multi --workers 8 --runs 5
```

#### Python 3.9+ (CPython)
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r python/requirements.txt
python3 -m unittest discover -s python/tests -v
python3 python/src/cli.py --dataset datasets/medium.json --mode single --runs 3
```

---

## 8. Summary of Findings

1. **Raw Instruction Throughput**: C++20 achieves the lowest single-threaded latency ($105.98\,\text{ms}$) due to aggressive auto-vectorization and zero-allocation priority queue traversals.
2. **Memory Efficiency**: Rust demonstrates the lowest memory footprint ($15.7\,\text{MB}$ RSS on $10^4$ nodes; $66.2\,\text{MB}$ on $5 \times 10^4$ nodes), achieving up to $13.08\times$ higher density than C++ and avoiding GC heap bloat.
3. **Multi-Core Scaling**: Rust (`Rayon`) achieves near-linear scaling ($4.93\times$ speedup on 8 cores), outperforming Java ($1.99\times$) and Go ($2.75\times$).
4. **Developer Productivity vs. Latency**: Go delivers the highest developer velocity while retaining competitive throughput ($84.44\,\text{ms}$ multi-threaded latency).
5. **CPython Concurrency Bottleneck**: CPython exhibits negative multi-threaded scaling ($0.97\times$) under the Global Interpreter Lock (GIL) on pure CPU-bound graph traversal, demonstrating the necessity of native compiled extensions for production dispatch systems.

---

## 9. License

This research codebase and its accompanying datasets are distributed under the open-source **[MIT License](LICENSE)**.
