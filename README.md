# Knapsack-Based Opportunistic Routing with Dynamic Link Duration Estimation in Delay Tolerant Networks (DTN)

[![Java 21](https://img.shields.io/badge/Java-21%20LTS-orange.svg?style=flat-square&logo=openjdk)](https://www.oracle.com/java/)
[![Simulator](https://img.shields.io/badge/Simulator-The%20ONE%20v1.6.0-blue.svg?style=flat-square)](https://github.com/akeranen/the-one)
[![Domain](https://img.shields.io/badge/Domain-Opportunistic%20%26%20DTN%20Routing-emerald.svg?style=flat-square)](#)
[![Status](https://img.shields.io/badge/Research-Undergraduate%20Thesis-purple.svg?style=flat-square)](#)
[![License](https://img.shields.io/badge/License-GPL%20v3-red.svg?style=flat-square)](LICENSE.txt)

> **Undergraduate Thesis Project (Skripsi)**  
> **Author:** Anselmus Erik Kalis Jati  
> **Department:** Department of Informatics, Faculty of Science and Technology, Universitas Sanata Dharma, Yogyakarta

---

## 📌 Abstract & Overview

In **Delay Tolerant Networks (DTN)** and Opportunistic Mobile Networks, end-to-end communication paths are rarely continuous. Instead, nodes forward messages across intermittent contacts using the *store-carry-and-forward* paradigm. 

A critical challenge in urban opportunistic networks is that **contact windows (*Link Duration*) between moving mobile nodes are often brief and unpredictable**. Conventional flooding protocols (e.g., *Epidemic Routing*) blindly transmit buffered messages in First-In-First-Out (FIFO) order, which frequently leads to:
1. Transmissions prematurely aborted due to link teardowns.
2. Heavy network congestion and redundant replicas.
3. Severe buffer exhaustion and massive packet drop rates.

This repository implements **KnapsackDP**, an intelligent opportunistic routing engine that transforms message forwarding under ephemeral contacts into a **0/1 Knapsack Optimization Problem** solved through **Dynamic Programming (DP)**. By dynamically predicting the contact duration based on real-time mobility kinematics and evaluating the marginal utility of each message, the router selects the mathematically optimal combination of messages to pack into the available bandwidth window.

---

## 🚀 Key Architectural Innovations

```
           Node Encounter (Connection Up)
                         │
                         ▼
        ┌───────────────────────────────────┐
        │   Link Duration Estimator (LDE)   │ ──► Mobility Kinematics (v, θ, r)
        │   Wu et al. Analytical Model      │ ──► Contact Duration (T_ij)
        └───────────────────────────────────┘
                         │
                         ▼
           Knapsack Capacity Calculation
           Capacity = (T_ij × Speed) / 1000  (KB)
                         │
                         ▼
        ┌───────────────────────────────────┐
        │       Utility Metric Engine       │
        │  • RAPID (Expected Delay Red.)    │ ──► Profit Calculation
        │  • PRoPHET (Delivery Probability) │ ──► Non-Negative Transform
        │  • Random / Uniform Baselines     │
        └───────────────────────────────────┘
                         │
                         ▼
        ┌───────────────────────────────────┐
        │      0/1 Knapsack DP Solver       │ ──► Global Utility Maximization
        │      O(n × C) Time Complexity     │ ──► Optimal Subset Selection
        └───────────────────────────────────┘
                         │
                         ▼
        Opportunistic Transmission Schedule
           (Guaranteed Completion in Window)
```

### 1. Dynamic Link Duration Estimation (LDE)
* Implemented in [`LinkDurationEstimationRouter.java`](src/routing/LinkDurationEstimationRouter.java).
* Based on the analytical kinematic model by *Wu et al. (IEEE Transactions on Vehicular Technology, 2009)*.
* Calculates the relative velocity vector ($\vec{V}_{12} = \vec{V}_2 - \vec{V}_1$), relative speed ($V_{ij}$), and incident angle ($\beta$) across the transmission radius ($r$) to predict the contact duration:
  $$T_{ij} = \frac{D_{ij}}{V_{ij}} = \frac{2r \cos(\beta)}{V_{ij}}$$
* **Velocity Vector Caching**: Implements tick-level velocity and unit-direction vector caching to maintain robust estimates even when nodes pause at waypoints (where instant velocity would otherwise evaluate to zero).

### 2. Contact-Adaptive Knapsack Capacity
* Rather than imposing artificial message count limits, the router dynamically sizes the knapsack restriction ($C$) for each contact:
  $$C = \min\left(\text{MaxBuffer}, \left\lfloor \frac{T_{ij} \times \text{TransmitSpeed}}{1000} \right\rfloor\right) \text{ [KB]}$$
* Prevents partial transfers and aborted connections before physical disconnection occurs.

### 3. Multi-Strategy Utility Evaluation
* **RAPID Utility (`AVERAGE_DELAY` - Primary Proposed Scheme)**:
  * Leverages past encounter distributions (`DelayTable` & `MeetingEntry`) to estimate the expected delivery delay $D(i)$.
  * Baseline utility: $U(i) = -D(i)$.
  * Employs a non-negative translation $P(i) = U(i) - D_{\max}$ to ensure compatibility with 0/1 Knapsack Dynamic Programming.
  * Evaluates marginal utility $\Delta U / \text{size}(i)$ to maximize delay reduction per byte transferred.
* **PRoPHET Utility (`PROPHET`)**:
  * Utilizes delivery predictability ($P \in [0, 1]$) with encounter aging ($\gamma = 0.98$), direct contact increment ($P_{\text{init}} = 0.75$), and transitive predictability scaling ($\beta = 0.25$).
* **Random & Uniform Baselines (`RANDOM`, `UNIFORM`)**:
  * Provides stochastic and uniform priority baselines to empirically isolate the contribution of the utility metric.

### 4. 0/1 Knapsack Dynamic Programming Optimization
* Formulates message relay selection as:
  $$\max \sum_{i=1}^n P(i) \cdot x_i \quad \text{subject to} \quad \sum_{i=1}^n w_i \cdot x_i \le C, \quad x_i \in \{0, 1\}$$
* Guarantees optimal subset selection in pseudopolynomial time $O(n \cdot C)$.

### 5. Unlimited Storage Model
* Overrides `makeRoomForMessage()` to evaluate message scheduling efficiency purely constrained by contact duration and bandwidth, isolating transmission efficiency from buffer drop artifacts.

---

## 📊 Experimental Results & Benchmarks

The proposed routing protocols were rigorously benchmarked in **The ONE Simulator** under an extensive **12-Hour (43,200 simulated seconds)** scenario with **100 mobile nodes** moving under the **Random Waypoint (RWP)** model on a $5000 \times 5000\text{ m}^2$ plane using **Bluetooth** interfaces ($250\text{ kB/s}$, $100\text{m}$ range).

A total of **1,459 messages** ($500\text{ kB} - 1\text{ MB}$) were generated with Poisson/uniform intervals ($25 - 35\text{ s}$).

### 🏆 Comparative Performance Table

| Routing Protocol | Delivery Probability | Delivered / Created | Average Latency (s) | Overhead Ratio | Dropped Packets |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **KnapsackDP + RAPID (`AVERAGE_DELAY`)** | **78.82%** | **1,150 / 1,459** | **2,533.17** | **32.59** | **0** |
| **KnapsackDP + PRoPHET** | 65.85% | 961 / 1,459 | 3,235.04 | 47.83 | 0 |
| **KnapsackDP + Random** | 49.83% | 727 / 1,459 | 7,657.30 | 68.76 | 0 |
| **Epidemic Router (Baseline)** | 31.14% | 454 / 1,459 | 8,247.48 | 197.75 | 73,954 |

### 🔍 Key Findings & Research Insights
1. **Dramatic Delivery Enhancement**: **KnapsackDP + RAPID achieved a 78.82% delivery probability**, representing an **increase of +153.1%** compared to the standard Epidemic baseline (31.14%).
2. **Massive Overhead Reduction**: Network replication overhead was slashed from **197.75 down to 32.59** (**-83.5% reduction**), preserving channel bandwidth and avoiding wasteful redundant transmissions.
3. **Significant Latency Drop**: Average message delivery latency decreased from **8,247.48 seconds (~137 minutes) to 2,533.17 seconds (~42 minutes)**, speeding up packet delivery by **69.3%**.
4. **Utility Strategy Impact**: Even with identical Knapsack DP mechanisms, RAPID's expected delay utility outperformed PRoPHET (+12.97% delivery) and Random selection (+28.99% delivery), validating that **delay-reduction optimization provides superior routing guidance in opportunistic topologies**.

---

## 📂 Repository Structure

```
.
├── src/
│   ├── routing/
│   │   ├── KnapsackDP.java                   # Core Knapsack 0/1 DP Router implementation
│   │   ├── LinkDurationEstimationRouter.java # Analytical Link Duration Estimator (Wu et al.)
│   │   ├── LinkDurationRouter.java           # Base router with link duration calculation hooks
│   │   ├── EpidemicRouterMod.java            # Modified Epidemic comparison router
│   │   └── rapid/                            # RAPID utility subsystem
│   │       ├── DelayTable.java               # Distributed delay and meeting history table
│   │       ├── DelayEntry.java               # Per-message expected delay container
│   │       └── MeetingEntry.java             # Inter-node meeting probability tracker
│   ├── report/
│   │   └── LinkDurationComparisonReport.java # Custom validation report: Real vs. Estimated Link Duration
│   ├── core/                                 # The ONE simulation engine (DTNHost, Message, SimClock)
│   ├── movement/                             # Mobility models (RandomWaypoint, MapBasedMovement)
│   └── gui/                                  # Graphical simulation visualizer
├── settings/
│   ├── Knapsack_new_3.cfg                    # Primary research simulation scenario (KnapsackDP + RAPID)
│   ├── test_prophet.cfg                      # 12-Hour benchmark scenario (KnapsackDP + PROPHET)
│   ├── test_random.cfg                       # 12-Hour benchmark scenario (KnapsackDP + RANDOM)
│   ├── test_epidemic.cfg                     # 12-Hour benchmark scenario (Epidemic Baseline)
│   └── knapsack.cfg                          # Modular DecisionEngineRouter configuration
├── reports/                                  # Simulation output logs (.csv / .txt)
├── lib/                                      # External JAR libraries for The ONE
├── compile.bat                               # Windows automated compiler script
├── one.bat                                   # Windows simulation runner script
└── README.md                                 # Project documentation
```

---

## 🛠️ Getting Started

### Prerequisites
* **Java Development Kit (JDK) 21 (LTS)** or higher.
* Verify your Java installation:
  ```powershell
  java -version
  javac -version
  ```

### 1. Build / Compile
Run the included automated compilation batch script from the root directory:
```powershell
.\compile.bat
```
Upon completion, compiled `.class` files will be placed into the `target/` directory.

### 2. Run Simulation

#### Option A: Interactive Graphical Mode (GUI)
Run the primary Knapsack scenario with real-time map visualization:
```powershell
.\one.bat 0 settings\Knapsack_new_3.cfg
```
*Click the **▶ Play** button on the simulator window to begin.*

#### Option B: Headless Batch Mode (Fast Execution)
To run automated 12-hour simulation benchmarks without GUI overhead:
```powershell
# Run KnapsackDP with RAPID Utility
.\one.bat -b 1 settings\Knapsack_new_3.cfg

# Run KnapsackDP with PRoPHET Utility
.\one.bat -b 1 settings\test_prophet.cfg

# Run KnapsackDP with Random Utility
.\one.bat -b 1 settings\test_random.cfg

# Run Epidemic Baseline
.\one.bat -b 1 settings\test_epidemic.cfg
```

### 3. Analyze Results
Simulation reports are automatically generated and saved in the `reports/` directory:
* `[ScenarioName]_MessageStatsReport.csv`: High-level metrics (Delivery Probability, Latency, Overhead Ratio, Relayed count).
* `[ScenarioName]_ContactTimesReport.csv`: Physical contact duration statistics.
* `[ScenarioName]_LinkDurationComparisonReport.csv`: Empirical accuracy evaluation of estimated vs. actual link durations.

---

## 📖 Citation & Academic Context

If you refer to or build upon this implementation in your research, please cite:

```bibtex
@misc{jati2026knapsackdtn,
  author       = {Anselmus Erik Kalis Jati},
  title        = {Optimization of Message Routing in Delay Tolerant Networks Using Dynamic Link Duration Estimation and 0/1 Knapsack Algorithm},
  year         = {2026},
  publisher    = {GitHub},
  howpublished = {\url{https://github.com/AnselmusErik/Skripsi_Full}},
  institution  = {Department of Informatics, Universitas Sanata Dharma}
}
```

---

## 📄 License
This project is licensed under the **GNU General Public License v3.0 (GPLv3)** in compliance with The ONE Simulator open-source licensing. See [LICENSE.txt](LICENSE.txt) for details.
