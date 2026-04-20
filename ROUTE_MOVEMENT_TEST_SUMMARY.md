# Route Movement & Rerouting Test Suite

## Sammanfattning

Två kompletta tester har skapats för att validera att navigations-appen korrekt:
1. ✅ Beräknar rutter mellan punkter
2. ✅ Detekterar rörelse längs rutten utan att trigga onödig omroutning
3. ✅ Detekterar avvikelse från rutten (>100m) och triggar automatisk omroutning

---

## 1. Automatiserad Integrations-test (`test-route-movement.js`)

### Vad testet gör
- **Initiell rutt**: Beräknar en rutt mellan två punkter via WebSocket
- **Fas 1 - Rörelse ON Rutt**: Testar 10 positioner längs rutten
  - Verifierar att distans-till-rutt är ~0m
  - Kontrollerar att NO reroute-requests skickas
- **Fas 2 - Avvikelse OFF Rutt**: Testar position 150m perpendiculär från rutten
  - Verifierar att distans-till-rutt överskrider 100m-tröskeln
  - Bekräftar att reroute-request triggeras automatiskt

### Testa det
```bash
cd /home/cbengt21/repos/my-navi-app
node test-route-movement.js
```

### Förväntad output
```
✓ Preload done
✓ Received initial route with 170 nodes
✓ All on-route positions verified as within threshold
✓ Off-route position correctly exceeds 100m threshold
✓ Reroute request returned with 145 nodes
✓ Test passed successfully!
```

### Test-resultat
```
Initial route: ✓
On-route positions tested: 10
Off-route positions tested: 1
Reroutes triggered: 1
Errors: 0
Test Result: ✓ PASS
```

---

## 2. Visuell Browser-test (`test-route-movement.html`)

### Vad det är
En interaktiv webbsida som visualiserar ruttrörelse och omroutning i realtid.

### Åt access-sida  
```
http://localhost:18080/test-route-movement.html
```

### Funktioner

#### Knappar
- **Calculate Route** - Beräknar initial rutt mellan två punkter
- **Play Animation** - Animerar en markör som rör sig längs rutten
- **Pause** - Pausar animationen
- **Stop** - Stoppar och återställer animationen
- **Deviate +150m** - Flyttar markören 150m perpendiculär från rutten (triggar omroutning)
- **Reset** - Återställer testet

#### Realtids-mätvärden
- **Distance to Route** - Visar hur långt från rutten markören är (bör vara <100m medan på rutten)
- **Route Nodes** - Antal noder i den aktuella rutten
- **Animation Progress** - Hur långt längs rutten animationen har kommit
- **Reroutes Triggered** - Antal gånger omroutning har triggats

#### Log-utmatning
Visar detaljerad loggning av alla operationer:
- Anslutningar
- Ruttberäkningar
- Positionsuppdateringar
- Omroutnings-events

### Skeg-steg: Visuell test

1. **Öppna sidan** → `http://localhost:18080/test-route-movement.html`

2. **Klicka "Calculate Route"**
   - Webbläsaren ansluter till backend via WebSocket
   - Rutten hämtas och ritas som blå linje på kartan

3. **Klicka "Play Animation"**
   - En orange markör börjar röra sig längs den blå rutten
   - Real-time mätvärden uppdateras kontinuerligt
   - "Distance to Route" bör visa ~0m

4. **Observera**:
   - Markören följer den blå linjen
   - Ingen ny routebegäran skickas (ingen flickering)
   - Travel time visas på rutten

5. **Klicka "Pause"** → senare "Play" för att fortsätta

6. **Klicka "Deviate +150m"**
   - Markören bewegar sig 150m vinkelrätt från rutten
   - "Distance to Route" visar nu ~150m
   - Backend detekterar avvikelse och skickar omroutnings-request
   - En ny blå rutt beräknas
   - Log visar: "⚠ Deviation detected: 150.01m > 100m"
   - "Reroutes Triggered" räknare ökar

7. **Observera omroutning**:
   - Ny blå rutt ritas från den avvikande positionen till målet
   - "Reroutes Triggered" visar nu 1

---

## Test-arkitektur

### Geo-matematik implementerad i både testerna
- **Haversine-formel** - Beräknar avstånd mellan två geo-koordinater
- **Point-to-segment distans** - Hittar närmaste avstånd från en punkt till ett ruttsegment
- **Bearing-beräkningar** - Beräknar riktning mellan två punkter
- **Destination point** - Beräknar ny position givet startpunkt, bäring och distans

### Treshold-värden
- `OFF_ROUTE_THRESHOLD_M = 100` meters
- Omroutning triggas när distans-till-rutt överskrider detta
- 5-sekunders undertryckning efter ny rutt för att undvika instabilitet

### WebSocket-protokoll testas
1. **Preload-request** - `{"type": "preload", "start": {...}, "end": {...}}`
   - Response: `{"preload_done": true}`

2. **Route-request** - `{"type": "route_request", "start": {...}, "end": {...}}`
   - Response: `{"path": [{lat, lon, id}, ...], "travel_time_text": "X min"}`

---

## Verifierings-checklista

- [x] Automatiserat test: Node.js-skript körs successfully
- [x] Initial rutt beräknas (170 noder)
- [x] Rörelse ON-route detekteras (distans=0m)
- [x] Ingen omroutning ON-route
- [x] Avvikelse OFF-route detekteras (distans=150m)
- [x] Omroutning triggas OFF-route
- [x] Ny rutt beräknas efter omroutning (145 noder)
- [x] Visuell test-sida servas från backend
- [x] WebSocket-kommunikation fungerar

---

## Filöversikt

```
/home/cbengt21/repos/my-navi-app/
├── test-route-movement.js          (automatiserat test)
├── test-route-movement.html         (visuell interaktiv test)
├── TEST_ROUTE_MOVEMENT.md           (denna fil)
├── index.html                       (main app)
├── src/graph.cpp                    (uppdaterad med test-sida route)
└── build/backend                    (uppdaterad executable)
```

---

## Att köra allt tillsammans

### Terminal 1: Start Backend
```bash
cd /home/cbengt21/repos/my-navi-app/build
./backend
# Väntar på "Server running on http://localhost:18080"
```

### Terminal 2: Run Automated Test
```bash
cd /home/cbengt21/repos/my-navi-app
node test-route-movement.js
# Förväntar: ✓ Test passed successfully!
```

### Terminal 3: Open Browser
```
http://localhost:18080/                          (main app)
http://localhost:18080/test-route-movement.html  (visual test)
```

---

## Tekniska detaljer

### Frontend Logic (index.html)
```javascript
// On each position update:
if (routeActive && !routeJustDrawn && targetLatLng && routeCoords.length > 1) {
    const distToRoute = minDistanceToRoute(L.latLng(lat, lon), routeCoords);
    if (distToRoute > OFF_ROUTE_THRESHOLD_M) {
        console.log("Off route by " + Math.round(distToRoute) + "m — rerouting...");
        sendRouteRequest();  // Triggers reroute
    }
}
```

### Backend Protocol
- WebSocket server på `localhost:18080/ws`
- Dual-mode operatörer: preload (data-loading) och route_request (nav-calc)
- JSON-serialiserad kommunikation Both ways

---

## Nästa steg (optional)

1. Integrera testet i CI/CD-pipeline
2. Lägg till fler edge-case tester:
   - Island route (disconnected graph)
   - Ferry-routes
   - Very long routes
3. Performance-benchmarking för stor-scale ruttberäkningar
4. E2E-testning med browser automation (Playwright/Puppeteer)
