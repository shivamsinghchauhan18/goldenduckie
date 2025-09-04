# README

This repository contains code and resources for the Duckiebot detection and navigation project.

For a detailed explanation of the project, implementation details, and results, please refer to the accompanying report: **duckietown.pdf**.

## Contents

- Source code for Duckiebot detection and navigation nodes
- Scripts and configuration files
- Proof of concept videos
- `duckietown.pdf` — Project report

## Getting Started

To understand the motivation, methodology, terminology and findings, start by reading **duckietown.pdf**.

## How to Build the Container

To build the development container, run:

```bash
$ docker -H <DUCKIEBOT_NAME>.local pull jassermarzougui/safe_lfv:lastest
```

## How to Run the Container

To start the container with the project files mounted:

```bash
$ dts duckiebot demo --demo_name lfv_start --package_name pure_pursuit_lfv --duckiebot_name <DUCKIEBOT_NAME> --image jassermarzougui/safe_lfv:lastest
```
