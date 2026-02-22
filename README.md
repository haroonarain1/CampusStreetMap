UIC Campus Navigation Tool

This project is a C++ campus navigation tool that models the University of Illinois Chicago campus as a weighted graph using OpenStreetMap data and computes shortest walking paths between buildings with Dijkstra’s algorithm.

Features:
-Parses OpenStreetMap JSON files to load buildings, waypoints, and walkways, then constructs an adjacency‑list graph to model the campus map as a weighted network.
-Implements a generic directed, weighted graph class and uses Dijkstra’s algorithm to find shortest paths between buildings while avoiding restricted nodes such as other buildings.

Provides both:
-A console interface for selecting buildings and viewing paths and distances.
-A local web server for visualizing the computed routes on a campus map.

Technologies Used:
-C++
-nlohmann::json for parsing OSM JSON files
​-OpenStreetMap campus data (buildings, waypoints, footways)
-Unix/Makefile build system
-Google Test / GMock for unit tests

Project Structure:
-graph.h – Generic adjacency‑list graph implementation (vertices, edges, neighbors, counts).
-application.cpp – buildGraph (loads OSM JSON into the graph) and dijkstra (shortest‑path algorithm), plus console application logic.
-dist.h / dist.cpp – Utilities for geographic coordinates and distance calculations.
-small_buildings.json, uic-fa25.osm.json – Sample and full campus datasets in JSON format.
-tests/ – Google Test–based unit tests for the graph, data loading, and Dijkstra.

Skills Demonstrated:
-Graph data structures and shortest‑path algorithms (Dijkstra).

Parsing and processing real‑world OpenStreetMap data in JSON format.

Memory‑safe C++ programming with unit testing and Unix build tooling.
