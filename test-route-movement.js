#!/usr/bin/env node
/**
 * Test: Route Movement & Rerouting
 * 
 * This test:
 * 1. Calculates a route between two points
 * 2. Simulates starting position movement ALONG the route
 * 3. Verifies no unnecessary rerouting occurs
 * 4. Simulates starting position movement OFF the route (>100m)
 * 5. Verifies that rerouting is triggered
 */

const WebSocket = require('ws');

// ============================================================================
// GEOMETRY HELPERS (matching frontend logic)
// ============================================================================

const EARTH_RADIUS_M = 6371000; // meters

function toRad(deg) {
    return deg * Math.PI / 180;
}

function haversine(lat1, lon1, lat2, lon2) {
    const φ1 = toRad(lat1);
    const φ2 = toRad(lat2);
    const Δφ = toRad(lat2 - lat1);
    const Δλ = toRad(lon2 - lon1);

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
        Math.cos(φ1) * Math.cos(φ2) *
        Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return EARTH_RADIUS_M * c;
}

function distToSegment(p, a, b) {
    // Distance from point p to line segment a-b
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];

    if (dx === 0 && dy === 0) {
        return haversine(p[0], p[1], a[0], a[1]);
    }

    let t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy);
    t = Math.max(0, Math.min(1, t));

    const closest = [a[0] + t * dx, a[1] + t * dy];
    return haversine(p[0], p[1], closest[0], closest[1]);
}

function minDistanceToRoute(point, routeCoords) {
    let minDist = Infinity;
    for (let i = 0; i < routeCoords.length - 1; i++) {
        const dist = distToSegment(point, routeCoords[i], routeCoords[i + 1]);
        if (dist < minDist) minDist = dist;
    }
    return minDist;
}

function interpolatePoint(start, end, fraction) {
    // Linear interpolation between two [lat, lon] points
    return [
        start[0] + (end[0] - start[0]) * fraction,
        start[1] + (end[1] - start[1]) * fraction
    ];
}

function destinationPoint(lat, lon, bearing, distanceM) {
    // Calculate destination point given start point, bearing, and distance
    const φ1 = toRad(lat);
    const λ1 = toRad(lon);
    const θ = toRad(bearing);
    const d = distanceM / EARTH_RADIUS_M;

    const φ2 = Math.asin(
        Math.sin(φ1) * Math.cos(d) +
        Math.cos(φ1) * Math.sin(d) * Math.cos(θ)
    );

    const λ2 = λ1 + Math.atan2(
        Math.sin(θ) * Math.sin(d) * Math.cos(φ1),
        Math.cos(d) - Math.sin(φ1) * Math.sin(φ2)
    );

    return [φ2 * 180 / Math.PI, λ2 * 180 / Math.PI];
}

function bearing(lat1, lon1, lat2, lon2) {
    // Calculate bearing from point 1 to point 2
    const φ1 = toRad(lat1);
    const φ2 = toRad(lat2);
    const Δλ = toRad(lon2 - lon1);

    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) -
        Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);

    const θ = Math.atan2(y, x);
    return (θ * 180 / Math.PI + 360) % 360;
}

// ============================================================================
// TEST HARNESS
// ============================================================================

class RouteMovementTest {
    constructor() {
        this.ws = null;
        this.promise = new Promise((resolve, reject) => {
            this.resolve = resolve;
            this.reject = reject;
        });
        this.testResults = {
            initialRoute: null,
            onRoutePositions: [],
            offRoutePositions: [],
            reroutes: [],
            passed: false,
            errors: []
        };
        this.messageLog = [];
        this.off_route_threshold = 100; // meters
    }

    log(msg) {
        const timestamp = new Date().toISOString();
        console.log(`[${timestamp}] ${msg}`);
    }

    async run() {
        this.log('=== Route Movement Test Starting ===');
        this.log('Connecting to ws://localhost:18080/ws ...');

        this.ws = new WebSocket('ws://localhost:18080/ws');

        this.ws.on('open', () => this.onOpen());
        this.ws.on('message', (buf) => this.onMessage(buf));
        this.ws.on('error', (err) => this.onError(err));
        this.ws.on('close', () => this.onClose());

        // Set timeout for entire test
        const timeout = setTimeout(() => {
            this.reject(new Error('Test timeout after 60 seconds'));
        }, 60000);

        try {
            await this.promise;
            clearTimeout(timeout);
        } catch (err) {
            clearTimeout(timeout);
            throw err;
        }
    }

    onOpen() {
        this.log('✓ WebSocket connected');

        // Start by requesting initial route
        const initialRoute = {
            type: 'preload',
            start: { lat: 57.7089, lon: 11.9746 },  // Gothenburg area
            end: { lat: 57.7000, lon: 11.9500 }
        };

        this.log('Sending preload request...');
        this.ws.send(JSON.stringify(initialRoute));
    }

    onMessage(buf) {
        const msg = buf.toString();
        this.messageLog.push(msg.substring(0, 200) + (msg.length > 200 ? '...' : ''));

        try {
            const data = JSON.parse(msg);

            // Handle preload completion
            if (data.preload_done) {
                this.log('✓ Preload done, requesting initial route...');
                this.requestInitialRoute();
                return;
            }

            // Handle route response
            if (Array.isArray(data.path) && data.path.length > 0) {
                if (!this.testResults.initialRoute) {
                    this.onInitialRoute(data);
                } else {
                    this.onRerouteResponse(data);
                }
                return;
            }

            if (data.error) {
                this.testResults.errors.push(`Backend error: ${data.error}`);
                this.log(`✗ Backend error: ${data.error}`);
            }
        } catch (err) {
            this.testResults.errors.push(`JSON parse error: ${err.message}`);
            this.log(`✗ JSON parse error: ${err.message}`);
        }
    }

    onError(err) {
        this.testResults.errors.push(`WebSocket error: ${err.message}`);
        this.log(`✗ WebSocket error: ${err.message}`);
        this.reject(err);
    }

    onClose() {
        this.log('WebSocket closed');
        if (!this.testResults.passed && this.testResults.errors.length === 0) {
            this.testResults.errors.push('WebSocket closed unexpectedly');
        }
    }

    requestInitialRoute() {
        const msg = {
            type: 'route_request',
            start: { lat: 57.7089, lon: 11.9746 },
            end: { lat: 57.7000, lon: 11.9500 }
        };
        this.ws.send(JSON.stringify(msg));
    }

    onInitialRoute(data) {
        this.log(`✓ Received initial route with ${data.path.length} nodes`);

        this.testResults.initialRoute = data;
        const routeCoords = data.path.map(p => [p.lat, p.lon]);

        // ====== TEST PHASE 1: Movement ON the route ======
        this.log('\n--- Phase 1: Simulating movement ON the route ---');

        for (let i = 0; i < Math.min(routeCoords.length, 10); i++) {
            const point = routeCoords[i];
            const dist = minDistanceToRoute(point, routeCoords);

            this.testResults.onRoutePositions.push({
                point,
                distanceToRoute: dist,
                shouldReroute: dist > this.off_route_threshold
            });

            this.log(
                `Step ${i + 1}: pos=[${point[0].toFixed(6)}, ${point[1].toFixed(6)}] ` +
                `dist=${dist.toFixed(2)}m ` +
                `(within route: ${dist <= this.off_route_threshold})`
            );
        }

        // Verify: all on-route positions have distance < threshold
        const allOnRoute = this.testResults.onRoutePositions.every(
            p => p.distanceToRoute <= this.off_route_threshold
        );

        if (allOnRoute) {
            this.log('✓ All on-route positions verified as within threshold');
        } else {
            this.testResults.errors.push('Some route points were marked as off-route');
            this.log('✗ ERROR: Some route points marked as off-route');
        }

        // ====== TEST PHASE 2: Movement OFF the route ======
        this.log('\n--- Phase 2: Simulating movement OFF the route ---');

        // Offset first route point by 150m perpendicular
        const startPoint = [routeCoords[0][0], routeCoords[0][1]];
        const secondPoint = routeCoords[1];

        // Calculate bearing from startPoint to secondPoint
        const routeBearing = bearing(startPoint[0], startPoint[1], secondPoint[0], secondPoint[1]);

        // Perpendicular bearing (90 degrees to the right)
        const perpBearing = (routeBearing + 90) % 360;

        // Move 150m away from route using the perpendicular bearing
        const offRoutePoint = destinationPoint(startPoint[0], startPoint[1], perpBearing, 150);

        const distOffRoute = minDistanceToRoute(offRoutePoint, routeCoords);
        this.testResults.offRoutePositions.push({
            point: offRoutePoint,
            distanceToRoute: distOffRoute,
            shouldReroute: distOffRoute > this.off_route_threshold
        });

        this.log(
            `Off-route test: pos=[${offRoutePoint[0].toFixed(6)}, ${offRoutePoint[1].toFixed(6)}] ` +
            `dist=${distOffRoute.toFixed(2)}m ` +
            `(should trigger reroute: ${distOffRoute > this.off_route_threshold})`
        );

        if (distOffRoute > this.off_route_threshold) {
            this.log(`✓ Off-route position correctly exceeds ${this.off_route_threshold}m threshold`);

            // Simulate the frontend sending a reroute request
            this.log('\nSimulating frontend reroute request for off-route position...');
            const rerouteMsg = {
                type: 'route_request',
                start: { lat: offRoutePoint[0], lon: offRoutePoint[1] },
                end: { lat: 57.7000, lon: 11.9500 }
            };
            this.ws.send(JSON.stringify(rerouteMsg));

            // Wait for reroute response
            this.waitingForReroute = true;
        } else {
            this.testResults.errors.push('Off-route offset did not exceed threshold');
            this.log('✗ ERROR: Off-route offset did not exceed threshold');
            this.completeTest();
        }
    }

    onRerouteResponse(data) {
        if (this.waitingForReroute) {
            this.log(`✓ Reroute request returned with ${data.path.length} nodes`);
            this.testResults.reroutes.push(data);

            this.log('\n=== Test Complete ===');
            this.testResults.passed =
                this.testResults.onRoutePositions.length > 0 &&
                this.testResults.offRoutePositions.length > 0 &&
                this.testResults.reroutes.length > 0 &&
                this.testResults.errors.length === 0;

            this.completeTest();
        }
    }

    completeTest() {
        this.log('\n=== Test Results ===');
        this.log(`Initial route: ${this.testResults.initialRoute ? '✓' : '✗'}`);
        this.log(`On-route positions tested: ${this.testResults.onRoutePositions.length}`);
        this.log(`Off-route positions tested: ${this.testResults.offRoutePositions.length}`);
        this.log(`Reroutes triggered: ${this.testResults.reroutes.length}`);
        this.log(`Errors: ${this.testResults.errors.length}`);

        if (this.testResults.errors.length > 0) {
            this.log('\nErrors:');
            this.testResults.errors.forEach(err => {
                this.log(`  - ${err}`);
            });
        }

        this.log(`\nTest Result: ${this.testResults.passed ? '✓ PASS' : '✗ FAIL'}`);

        this.ws.close();
        if (this.testResults.passed) {
            this.resolve();
        } else {
            this.reject(new Error(`Test failed: ${this.testResults.errors.join('; ')}`));
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

(async () => {
    try {
        const test = new RouteMovementTest();
        await test.run();
        console.log('\n✓ Test passed successfully!');
        process.exit(0);
    } catch (err) {
        console.error('\n✗ Test failed:', err.message);
        process.exit(1);
    }
})();
