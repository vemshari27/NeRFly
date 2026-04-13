# NeRFly

**Energy-Efficient UAV Trajectory Planning for NeRF-Based 3D Reconstruction**

> *Find the minimum-energy flight path for a camera-equipped UAV that achieves high-quality 3D scene reconstruction using Neural Radiance Fields.*
> 

---

## 🎯 Project Goal

Design and train an autonomous UAV agent that plans **energy-efficient trajectories** to capture 2D images of a scene, object, or building — collecting only the views needed to produce a high-fidelity NeRF reconstruction, while minimizing battery consumption.

---

## 🧠 Core Idea

Most UAV reconstruction pipelines follow fixed flight patterns (circular, grid, oblique) that are wasteful — they capture redundant views and ignore the UAV's energy budget. NeRFly frames this as a **Reinforcement Learning problem**, where the agent learns to fly smarter, not longer.

**The key tradeoff:** More views → better reconstruction, but more energy. NeRFly learns *which* views matter most and *how* to reach them efficiently.

---

## 🗂️ Problem Formulation

| Component | Description |
| --- | --- |
| **State** | Partial NeRF reconstruction + uncertainty volume + UAV pose + remaining battery |
| **Action** | Next waypoint in 6-DOF (x, y, z, roll, pitch, yaw) |
| **Reward** | Δ(reconstruction quality) − λ · energy consumed |
| **Termination** | Quality threshold reached OR battery depleted |
| **Scene Types** | Objects, buildings, outdoor scenes |

---

## 🔬 Research Positioning

NeRFly sits at the intersection of three active research areas:

- **Active Perception** — Next-Best-View (NBV) planning
- **Neural Scene Representations** — NeRF / 3D Gaussian Splatting
- **Energy-Constrained UAV Planning** — Battery-aware trajectory optimization

**The gap we fill:** Existing NBV methods optimize for reconstruction quality or path length — *none jointly optimize for UAV energy consumption with a learned, generalizable policy.*

---

## 📚 Key Related Work

| Paper | Venue | Relevance |
| --- | --- | --- |
| ActiveNeRF (Pan et al.) | ECCV 2022 | Uncertainty-guided view selection in NeRF |
| GenNBV (Chen et al.) | CVPR 2024 | RL-based NBV with 5D drone action space |
| RL-NBV (Hill et al.) | — | Deep RL for unknown object reconstruction |
| ActiveImplicitRecon | RA-L 2023 | Gradient-based continuous NBV optimization |
| Drone-NeRF | IMAVIS 2024 | NeRF framework designed for drone imagery |
| Bircher et al. | ICRA 2016 | Receding horizon information-gain planning |

---

## 💻 Key Codebases

| Repo | Purpose |
| --- | --- |
| `LeapLabTHU/ActiveNeRF` | Uncertainty estimation in NeRF |
| `zjwzcx/GenNBV` | RL-based generalizable NBV baseline |
| `MingFengHill/RL-NBV` | RL-NBV with point cloud state |
| `HITSZ-NRSL/ActiveImplicitRecon` | Continuous manifold NBV optimization |
| `nerfstudio-project/nerfstudio` | Modular NeRF backbone |

---

[Tasks Tracker](https://www.notion.so/3412b07e874380a58432db483edc93a9?pvs=21)