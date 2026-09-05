# 🛸 Autonomous UAV Navigation with PID Velocity Control

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Completed-success.svg)]()
[![Pure Python](https://img.shields.io/badge/Dependencies-Standard%20Library-orange.svg)]()

An autonomous flight navigation and velocity regulation simulation for an Unmanned Aerial Vehicle (UAV). The system guides the drone through real-world GPS checkpoints towards a target destination using a **Profile-Guided discrete PID Controller** with anti-windup clamping, slew-rate limiting, and spherical Earth geodesy (Haversine & Great-Circle Dead Reckoning).

---

## 📌 Mission Objectives & Constraints

The goal is to navigate the UAV across designated GPS coordinates while dynamically adjusting velocity according to strict safety and flight envelopes:

| Constraint / Parameter | Requirement | Simulation Result | Status |
| :--- | :--- | :--- | :--- |
| **Max Flight Speed** | $\le 80.0\text{ m/s}$ | $70.0\text{ m/s}$ cruise | ✅ Compliant |
| **Safe Reach Radius** | $10.0\text{ m}$ around waypoints | $10.0\text{ m}$ threshold | ✅ Compliant |
| **Target Arrival Velocity** | **$< 10.0\text{ m/s}$** within $10\text{ m}$ radius | **$8.94\text{ m/s}$** | ✅ **Passed (Safe)** |
| **Telemetry Logging** | Every $5\text{ seconds}$ | Automated 5-second interval | ✅ Compliant |
| **External Dependencies** | None (Python Standard Library) | `math`, `datetime`, `os` | ✅ Zero-dependency |

---

## 🗺️ Flight Waypoints (WGS-84)

The flight plan consists of two primary navigation legs across the Delhi region:

| Waypoint | Latitude (°N) | Longitude (°E) | Leg Distance | Role |
| :--- | :--- | :--- | :--- | :--- |
| **Takeoff** | `28.748611` | `77.117222` | — | Mission Launch |
| **Checkpoint 1** | `28.744444` | `77.138056` | $2,083.3\text{ m}$ | Intermediate Turn Point |
| **Target Location** | `28.723611` | `77.113333` | $3,343.2\text{ m}$ | Final Safe Destination |
| **Total Route** | — | — | **$5,426.5\text{ m}$ ($5.43\text{ km}$)** | Total Mission Flight |

