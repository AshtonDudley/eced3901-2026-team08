# captain-phillips
ECED3901 Winter 2026

Dalhousie University 

Team 8: Captain Phillips

## Table of Contents
- [Subsystems and Team Member Responsible](#subsystems-and-team-member-responsible)
- [Project Overview](#project-overview)
- [Key Requirements](#key-requirements)
- [Navigation & Environment](#navigation--environment)
- [Mission Objectives](#mission-objectives)
- [Scoring](#scoring)

## Subsystems and Team Member Responsible:
1. Lighting safety system (Liam Legge*)
2. Navigation System (Megan Neville)
3. Cargo handling system (Ashton Dudley)
4. Rescue system (Lucas Melanson)
5. Pirate deterrent system (Lorne MacLean)

## Project Overview
This project involves designing and programming an autonomous robot to navigate a simulated shipping lane, avoid moving pirate obstacles, transport cargo, and perform rescue operations. The robot must operate fully autonomously and comply with strict safety and hardware constraints.

## Key Requirements
- Autonomy: Robot must operate without manual control after deployment
- Size Limit: Maximum diameter and height of 50 cm
- Power: No voltages above 24V
- Actuation: No pneumatic, hydraulic, gas, liquid, or aerosol systems
- Hardware: Only kit parts, approved purchases, small hardware, and 3D-printed components allowed
- Safety: Robot must pass a pre-inspection and avoid physical contact with obstacles
- Attempts: Maximum of 3 runs or 15 minutes total
- Deployment: Must work from all starting positions (Team A or B)

## Navigation & Environment
- Teams choose between:
  1. Coastal Path: Longer but pirate-free
  2. Open Ocean: Shorter but includes moving pirate obstacles
- Pirates move up to 0.1 m/s and are detectable by LiDAR
- Teams can control pirate movement using FSK communication and optical signaling

## Mission Objectives
- Reach target port
- Drop off initial cargo
- Pick up return cargo
- Rescue a randomly placed person overboard
- Return cargo to home port

## Scoring
| Task                                   | Points |
|----------------------------------------|--------|
| Safety lighting system                 | +25    |
| Reach target port                      | +25    |
| Drop off original cargo in target port                | +25    |
| Pick up new cargo                      | +10    |
| Rescue person overboard                | +10    |
| Deliver & return cargos < 3 minutes  | +5     |
| Lost cargo (cumulative, max 2)               | -5     |
| Physical contact (not cumulative)              | -10    |

All information in this repository is proprietary, and lawsuits will be pursued aggressively. We will make it our mission to make sure your grandkid's grandkids pay for real or perceived discretions against this legendary project.

*Indicates team captain
