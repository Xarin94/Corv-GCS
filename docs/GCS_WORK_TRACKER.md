# GCS MAVLink Migration - Work Tracker

## Stato Generale
- **Progetto**: CORV SYSTEMS v16 -> GCS MAVLink
- **Approccio**: node-mavlink + serialport nel main process Electron
- **Protocollo legacy**: CORV Binary mantenuto come opzione
- **Stato**: Fase 0-5 implementate, Fase 6 (SITL/Polish) da completare

---

## Fase 0: Fondamenta MAVLink
| Task | Stato | File |
|------|-------|------|
| Installare node-mavlink e serialport | [x] | package.json |
| Electron rebuild per native modules | [x] | @electron/rebuild |
| Estendere STATE con campi MAVLink | [x] | js/core/state.js |
| Creare MAVLinkManager.js | [x] | js/mavlink/MAVLinkManager.js |
| Creare MAVLinkStateMapper.js | [x] | js/mavlink/MAVLinkStateMapper.js |
| Creare ConnectionManager.js | [x] | js/mavlink/ConnectionManager.js |
| Creare CommandSender.js | [x] | js/mavlink/CommandSender.js |
| Aggiungere IPC handlers main process | [x] | main.js + main-mavlink.js |
| Aggiungere API mavlink al preload | [x] | preload.js |
| Heartbeat timer 1Hz | [x] | main-mavlink.js |

---

## Fase 1: Barra Comandi Inferiore
| Task | Stato | File |
|------|-------|------|
| HTML command bar | [x] | html/index.html |
| CSS command bar | [x] | css/command-bar.css |
| CommandBarController | [x] | js/ui/CommandBarController.js |
| Layout adjustments (bottom: 48px) | [x] | css/layout.css |
| Import CSS in style.css | [x] | css/style.css |
| Wire ARM/DISARM command | [x] | js/ui/CommandBarController.js |
| Wire flight mode change | [x] | js/ui/CommandBarController.js |
| Wire TAKEOFF/RTL/LAND | [x] | js/ui/CommandBarController.js |
| Wire mission speed | [x] | js/ui/CommandBarController.js |

---

## Fase 2: Sidebar GCS Verticale
| Task | Stato | File |
|------|-------|------|
| HTML GCS sidebar | [x] | html/index.html |
| CSS GCS sidebar | [x] | css/gcs-sidebar.css |
| GCSSidebarController | [x] | js/ui/GCSSidebarController.js |
| Parameter read/write/cache (inline) | [x] | js/ui/GCSSidebarController.js |
| Sezione ACTIONS | [x] | js/ui/GCSSidebarController.js |
| Sezione PARAMETERS | [x] | js/ui/GCSSidebarController.js |
| Sezione MISSION | [x] | js/ui/GCSSidebarController.js |
| Sezione GEOFENCE | [x] | js/ui/GCSSidebarController.js |
| Sezione RALLY POINTS | [x] | js/ui/GCSSidebarController.js |
| MissionManager dedicato | [ ] | js/mavlink/MissionManager.js (futuro) |
| GeofenceManager dedicato | [ ] | js/mavlink/GeofenceManager.js (futuro) |
| RallyManager dedicato | [ ] | js/mavlink/RallyManager.js (futuro) |

---

## Fase 3: Navigazione a Tab
| Task | Stato | File |
|------|-------|------|
| HTML tab bar | [x] | html/index.html |
| CSS tab bar | [x] | css/tabs.css |
| TabController | [x] | js/ui/TabController.js |
| Container FLIGHT DATA | [x] | html/index.html |
| Container FLIGHT PLAN | [x] | html/index.html |
| Container INITIAL SETUP | [x] | html/index.html |
| Container CONFIG/TUNING | [x] | html/index.html |
| Container SIMULATION | [x] | html/index.html |

---

## Fase 4: Mission Planning (integrato nei Tab)
| Task | Stato | File |
|------|-------|------|
| Leaflet map nel tab FLIGHT PLAN | [x] | js/ui/TabController.js |
| Waypoint drag-and-drop | [x] | js/ui/TabController.js |
| Right-click add waypoint | [x] | js/ui/TabController.js |
| Lista waypoint laterale | [x] | html/index.html |
| Upload/Download placeholder | [x] | html/index.html |

---

## Fase 5: Config/Tuning (integrato nei Tab)
| Task | Stato | File |
|------|-------|------|
| PID tuning grid (Roll/Pitch/Yaw) | [x] | html/index.html + css/tabs.css |
| Write All PID button | [x] | js/ui/TabController.js |
| Parameter tree con ricerca | [x] | js/ui/TabController.js |
| Parameter edit inline | [x] | js/ui/TabController.js |

---

## Fase 6: SITL e Polish
| Task | Stato | File |
|------|-------|------|
| SITL connect nel tab Simulation | [x] | js/ui/TabController.js |
| Selettore tipo connessione (command bar) | [x] | html/index.html |
| Selettore tipo connessione (Initial Setup) | [x] | html/index.html |
| Testing con ArduPilot SITL | [ ] | - |
| Verifica protocollo CORV Binary legacy | [ ] | js/serial/SerialHandler.js |

---

## File Creati
| File | Tipo | Descrizione |
|------|------|-------------|
| main-mavlink.js | Main Process | MAVLink handler, serial/UDP, heartbeat, IPC |
| js/mavlink/MAVLinkManager.js | Renderer | Router messaggi, event dispatcher |
| js/mavlink/MAVLinkStateMapper.js | Renderer | Mappatura messaggi -> STATE |
| js/mavlink/ConnectionManager.js | Renderer | Gestione connessioni |
| js/mavlink/CommandSender.js | Renderer | Encoding e invio comandi |
| js/ui/CommandBarController.js | Renderer | Barra comandi inferiore |
| js/ui/GCSSidebarController.js | Renderer | Sidebar GCS verticale |
| js/ui/TabController.js | Renderer | Navigazione a tab |
| css/command-bar.css | CSS | Stili barra comandi |
| css/gcs-sidebar.css | CSS | Stili sidebar GCS |
| css/tabs.css | CSS | Stili tab navigation + setup panels |
| docs/GCS_WORK_TRACKER.md | Doc | Questo file |

## File Modificati
| File | Modifiche |
|------|-----------|
| package.json | +node-mavlink, +serialport |
| main.js (root) | +require main-mavlink, +initMAVLinkHandlers, +cleanup |
| preload.js | +window.mavlink API bridge |
| js/core/state.js | +30 campi MAVLink/GCS |
| js/main.js (renderer) | +import MAVLink/GCS, +init calls, +update loop |
| css/style.css | +import command-bar, gcs-sidebar, tabs |
| css/layout.css | +bottom:48px per command bar, +height calc per split |
| html/index.html | +tab bar, +tab containers, +command bar, +GCS sidebar |

---

## Note Implementazione
- MAVLink parsing nel main process (CommonJS/lazy-load), dati al renderer via IPC
- Tutti i nuovi JS renderer file sono ES6 modules con import espliciti
- serialport caricato lazy per evitare crash ABI al startup
- Pattern sidebar: replica NDController.js / nd-sidebar
- Event pattern: `serialUpdate` CustomEvent condiviso tra binary e MAVLink
- ArduPilot flight modes: STABILIZE=0, ACRO=1, ALT_HOLD=2, AUTO=3, GUIDED=4, LOITER=5, RTL=6, LAND=9
- Electron rebuild necessario: `npx @electron/rebuild`
