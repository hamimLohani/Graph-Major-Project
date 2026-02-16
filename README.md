# Graph Major Project

A C++ project implementing graph algorithms to find optimal routes and paths using geographic data from Dhaka city.

## Project Overview

This project uses Dijkstra's shortest path algorithm combined with geographic coordinates (latitude/longitude) to find optimal routes across different transportation modes including walking, cars, buses, and metro systems. Results are visualized as KML files for Google Earth.

## Project Structure

```
Graph-Major-Project/
├── Problem 1/
│   ├── Problem-1.cpp
│   ├── Problem-1.kml
│   ├── map.png
│   ├── output.png
│   └── input.txt
├── Problem 2/
│   ├── Problem-2.cpp
│   ├── Problem-2.kml
│   ├── map.png
│   ├── output.png
│   └── input.txt
├── Problem 3/
│   ├── Problem-3.cpp
│   ├── Problem-3.kml
│   ├── map.png
│   ├── output.png
│   └── input.txt
├── Problem 4/
│   ├── Problem-4.cpp
│   ├── Problem-4.kml
│   ├── map.png
│   ├── output.png
│   └── input.txt
├── Problem 5/
│   ├── Problem-5.cpp
│   ├── Problem-5.kml
│   ├── map.png
│   ├── output.png
│   └── input.txt
├── Problem 6/
│   ├── Problem-6.cpp
│   ├── Problem-6.kml
│   ├── map.png
│   ├── output.png
│   └── input.txt
├── Dhaka Graph Assignment - Problem Set.pdf # Problem
├── Roadmap-Dhaka.csv                        # Road network data for Dhaka
├── Routemap-BikolpoBus.csv                  # Bus routes data
├── Routemap-DhakaMetroRail.csv              # Metro rail routes data
└── Routemap-UttaraBus.csv                   # Uttara bus routes data
```

## Features

- **Dijkstra's Algorithm**: Finds shortest paths in weighted graphs
- **Haversine Formula**: Calculates distances between geographic coordinates
- **Multi-modal Transportation**: Supports different transportation modes (walking, car, bus, metro)
- **KML Export**: Generates KML files for visualization in Google Earth
- **Time-based Optimization**: Problem 4 includes waiting times and operational hours

## Compilation

```bash
# Navigate to a problem directory
cd "Problem 1"

# Compile
g++ Problem-1.cpp -o Problem-1

```

## Running

### Problem 1 - Basic Shortest Path

```bash
cd "Problem 1"
./Problem-1 < input.txt
```

