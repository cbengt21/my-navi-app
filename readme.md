# Summary

This project implements a backend server for a navigation system using Crow (C++ web framework) and Osmium (for processing OpenStreetMap data). It uses WebSockets to communicate between the frontend and backend.

## Prerequisites

Ensure the following libraries are installed on your system:

- **Crow** (C++ Web Framework)
- **Osmium** (C++ Library for OpenStreetMap data)
- **Pthreads** (for threading support in C++)

Additionally, you'll need to have the **CMake** build system installed.

## Installation Instructions

1. **Install Dependencies**

   If you haven't already installed Crow and Osmium, you can install them using your system's package manager or compile them from source.

   For example, on Ubuntu:
   ```bash
   sudo apt-get install libboost-all-dev libpthread-stubs0-dev

## run
mkdir build
cd build
cmake ..
make
./backend
open index.html

## Alternativ:
Alternativ 1: Separat webbaserad frontend (Rekommenderat)
I detta scenario körs JavaScript som en vanlig webbapplikation i en webbläsare (eller en webbläsarliknande miljö som Electron). Den kommunicerar med backend-processen i C++ via IPC (t.ex. WebSocket, HTTP eller en annan protokoll).

Implementering:
Frontend som en fristående webapp:

Bygg din frontend med ett ramverk som React, Vue eller Angular, eller använd ren HTML/JavaScript.
Servera frontend-filerna via en statisk filserver, eller låt backend i C++ servera filerna.
IPC mellan frontend och backend:

Använd WebSocket eller HTTP för kommunikation.
Backend i C++ lyssnar på en port och tar emot anrop från frontend.
Exempel på flöde:

Frontend gör en HTTP- eller WebSocket-anrop till C++-backend.
Backend i C++ hanterar logik och skickar svar till frontend.
Fördelar:
Lätt att utveckla och underhålla frontend och backend separat.
Frontend kan distribueras på valfri plattform.
Enkel skalbarhet.


