# Route Movement & Rerouting Test

## Overview

This test validates that the navigation app correctly:
1. **Calculates routes** between points
2. **Detects movement** along the route without triggering unnecessary reroutes
3. **Detects deviation** from the route (>100m) and triggers automatic rerouting

## Test Files

### 1. `test-route-movement.js` (Automated Integration Test)
- **Purpose**: Automated backend/protocol level test
- **What it tests**:
  - Initial route calculation via WebSocket
  - Movement along route coordinates (should NOT trigger reroutes)
  - Movement 150m perpendicular to route (SHOULD trigger reroute)
  - Verifies reroute requests are sent by the backend

- **Running the test**:
  ```bash
  # Make sure backend is running on localhost:18080
  node test-route-movement.js
  ```

- **Expected output**:
  ```
  ✓ Preload done
  ✓ Received initial route with 170 nodes
  ✓ All on-route positions verified as within threshold
  ✓ Off-route position correctly exceeds 100m threshold
  ✓ Reroute request returned with 145 nodes
  ✓ Test passed successfully!
  ```

### 2. `test-route-movement.html` (Visual Browser Test)
- **Purpose**: Interactive browser-based test
- **What it does**:
  - Opens the map at the backend location
  - Calculates an initial route
  - Animates a marker moving along the calculated route
  - Shows real-time position and distance-to-route metrics
  - Allows manual deviation testing
  - Visualizes rerouting behavior

- **How to use**:
  1. Open `http://localhost:18080/test-route-movement.html` in a browser
  2. Click "Start Route Test" to calculate the initial route
  3. Click "Play" to animate movement along the route
  4. Watch the blue route line and real-time metrics
  5. Click "Deviate +150m" to move the marker 150m off-route and trigger reroute

- **What to watch for**:
  - The marker moves smoothly along the blue route line
  - Distance-to-route stays below 100m while following the route
  - Route remains unchanged while following (no flickering)
  - When deviating >100m, a new route is calculated (blue line updates)
  - Travel time updates appropriately

## Test Scenarios

### Scenario 1: Movement ON Route
- **Starting position**: Gothenburg area (57.7089, 11.9746)
- **Target**: Nearby location (57.7000, 11.9500)
- **Expected route**: ~170 nodes, ~2 min travel time
- **Test**: When marker is at any route coordinate, distance-to-route = 0m
- **Result**: ✓ No rerouting triggered

### Scenario 2: Movement OFF Route (>100m)
- **Starting position**: 150m perpendicular to initial route start
- **Expected**: Backend detects deviation >100m threshold
- **Result**: ✓ New route calculated, marker snaps to new path

## Key Thresholds

- **OFF_ROUTE_THRESHOLD_M**: 100 meters
- **REROUTE_SUPPRESSION**: 5 seconds after drawing route
- **Route deviation check**: Runs on every GPS position update

## Frontend Logic

The app maintains these key variables:

```javascript
let routeActive = false;           // Is user following a route?
let routeCoords = [];              // Current route as [lat,lon] pairs
let routeJustDrawn = false;        // 5-sec suppression flag
const OFF_ROUTE_THRESHOLD_M = 100; // Reroute if deviation > this

// On each position update (from geolocation):
if (routeActive && !routeJustDrawn) {
  const distToRoute = minDistanceToRoute(currentPos, routeCoords);
  if (distToRoute > OFF_ROUTE_THRESHOLD_M) {
    sendRouteRequest(); // Trigger reroute
  }
}
```

## Running Both Tests

```bash
# Terminal 1: Start the backend
cd /home/cbengt21/repos/my-navi-app/build
./backend

# Terminal 2: Run automated test
cd /home/cbengt21/repos/my-navi-app
node test-route-movement.js

# Terminal 3 (Browser):
# 1. Visit http://localhost:18080/ for main app
# 2. Or visit http://localhost:18080/test-route-movement.html for visual test
```

## Verification Checklist

- [ ] Backend starts and runs http://localhost:18080/
- [ ] Automated test: `node test-route-movement.js` returns ✓ PASS
- [ ] Browser test: Route is calculated and displayed
- [ ] Browser test: Marker animates along route smoothly
- [ ] Browser test: Distance-to-route metric shows ~0m while on route
- [ ] Browser test: Clicking "Deviate" triggers reroute
- [ ] Browser test: Blue line updates after reroute
- [ ] Console logs show reroute messages when deviating >100m
