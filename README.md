# UIC Campus Navigation Tool

A C++ application that models the University of Illinois Chicago campus as a directed, weighted graph and computes shortest walking paths between buildings using Dijkstra’s algorithm.

## Overview

This project converts real OpenStreetMap (OSM) data into a graph representation of the UIC campus, enabling efficient route computation and visualization while handling constraints such as restricted nodes.

## Features

- Builds an adjacency-list graph from OpenStreetMap JSON data (buildings, waypoints, footways)
- Implements Dijkstra’s algorithm for shortest path routing between locations
- Supports avoidance of restricted nodes (e.g., buildings)
- Provides:
  - Console interface for selecting buildings and viewing routes
  - Local web server for visualizing paths on a campus map

## Technologies

- C++
- nlohmann::json (JSON parsing)
- OpenStreetMap (OSM) data
- Google Test / Google Mock
- Unix / Makefile build system

## Project Structure

- `graph.h` – Generic adjacency-list graph implementation
- `application.cpp` – Graph construction, Dijkstra’s algorithm, and application logic
- `dist.h / dist.cpp` – Geographic coordinate utilities and distance calculations
- `*.json` – Campus datasets (sample and full OSM data)
- `tests/` – Unit tests using Google Test

## Skills Demonstrated

- Graph data structures and shortest-path algorithms
- Real-world geospatial data parsing and processing
- Modular C++ design with separation of concerns
- Unit testing and software validation
- Unix-based build system (Makefile)
