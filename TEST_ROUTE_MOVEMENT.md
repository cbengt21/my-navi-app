# Route Movement & Rerouting Test

## Overview

The test setup now uses:
1. Automated protocol test: test-route-movement.js
2. Dynamic visual test harness: test-dynamic-route.html

## Test Files

### 1) test-route-movement.js (Automated Integration Test)

Purpose:
- Validate route_request and preload flow over WebSocket
- Validate on-route movement does not reroute
- Validate off-route movement (>100 m) triggers reroute

Run:

```bash
cd /home/cbengt21/repos/my-navi-app
node test-route-movement.js
```

### 2) test-dynamic-route.html (Visual Browser Test)

Purpose:
- Run visual movement/reroute test against the real frontend (index.html)

Open:

```text
http://localhost:18080/test-dynamic-route.html
```

Flow:
1. Harness loads app in iframe via /?testMode=true
2. Click Calculate Route
3. Route is injected into app (injectRoute)
4. Click Simulate Movement to inject positions
5. Blue line should trim forward as position moves
6. Click Deviate +150m to trigger reroute

## Key Thresholds

- OFF_ROUTE_THRESHOLD_M = 100
- MIN_TRIM_ADVANCE_M = 15

## Running Tests

```bash
# Terminal 1
cd /home/cbengt21/repos/my-navi-app/build
./backend

# Terminal 2
cd /home/cbengt21/repos/my-navi-app
node test-route-movement.js
```

Browser:
- http://localhost:18080/
- http://localhost:18080/test-dynamic-route.html

## Verification Checklist

- [ ] Backend reachable on localhost:18080
- [ ] Automated test passes
- [ ] Dynamic visual test draws initial blue route
- [ ] Blue route trims during movement simulation
- [ ] Off-route deviation triggers reroute
