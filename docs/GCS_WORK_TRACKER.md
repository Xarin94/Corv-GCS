# CORV GCS — Work Tracker

## Stato Generale
- **Progetto**: CORV SYSTEMS v16 → CORV GCS (MAVLink + CORV binario + MSP)
- **Approccio**: node-mavlink + serialport nel main process Electron; ogni protocollo non-MAVLink
  viene decodificato nel main process e ri-emesso come MAVLink sintetico
- **Versione corrente**: 1.7.2
- **Stato**: Fasi 0-6 (migrazione MAVLink) completate; Fasi 7-11 aggiunte dopo la migrazione
- **Ultimo aggiornamento**: 2026-09-23

---

# PARTE I — Migrazione MAVLink (Fasi 0-6, completata)

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

## Fase 1: Barra Comandi Inferiore
| Task | Stato | File |
|------|-------|------|
| HTML + CSS command bar | [x] | html/index.html, css/command-bar.css |
| CommandBarController | [x] | js/ui/CommandBarController.js |
| Wire ARM/DISARM, modo di volo, TAKEOFF/RTL/LAND, velocità missione | [x] | js/ui/CommandBarController.js |

## Fase 2: Sidebar GCS Verticale
| Task | Stato | File |
|------|-------|------|
| HTML + CSS sidebar | [x] | html/index.html, css/gcs-sidebar.css |
| GCSSidebarController | [x] | js/ui/GCSSidebarController.js |
| Sezioni ACTIONS / PARAMETERS / MISSION / GEOFENCE / RALLY | [x] | js/ui/GCSSidebarController.js |
| MissionManager dedicato | [ ] | mai servito: l'editor missione sta in TabController |
| GeofenceManager dedicato | [ ] | js/mavlink/GeofenceManager.js (futuro) |
| RallyManager dedicato | [ ] | js/mavlink/RallyManager.js (futuro) |

## Fase 3: Navigazione a Tab
| Task | Stato | File |
|------|-------|------|
| Tab bar + CSS + 5 container | [x] | html/index.html, css/tabs.css |
| TabController | [x] | js/ui/TabController.js |

## Fase 4: Mission Planning
| Task | Stato | File |
|------|-------|------|
| Leaflet map nel tab FLIGHT PLAN | [x] | js/ui/TabController.js |
| Waypoint drag-and-drop, right-click add, lista laterale | [x] | js/ui/TabController.js |
| Poligono survey → griglia waypoint automatica | [x] | js/ui/TabController.js |
| Profilo altimetrico lungo la rotta | [x] | js/ui/TabController.js |
| Upload missione al veicolo | [x] | js/mavlink/CommandSender.js `uploadMission()` |
| Download missione dal veicolo | [ ] | la richiesta parte, la risposta non è gestita nel renderer |

## Fase 5: Config/Tuning
| Task | Stato | File |
|------|-------|------|
| PID tuning grid + Write All | [x] | html/index.html, js/ui/TabController.js |
| Parameter tree con ricerca ed edit inline | [x] | js/ui/TabController.js |

## Fase 6: SITL e Polish
| Task | Stato | File |
|------|-------|------|
| SITL launcher (download binari + WSL su Windows) | [x] | sitl-manager.js |
| Selettori tipo connessione | [x] | html/index.html |
| Testing con ArduPilot SITL | [x] | sitl-defaults/ |
| Verifica protocollo CORV Binary legacy | [x] | decoder spostato nel main process (main-mavlink.js) |

---

# PARTE II — Evoluzione post-migrazione (Fasi 7-11)

## Fase 7: Motore 3D, terreno e strumenti di volo — v1.3.x
| Task | Stato | File |
|------|-------|------|
| Terreno SRTM con LOD a chunk + 4 Web Worker | [x] | js/terrain/ |
| Hillshade nello shader del terreno + sole per ora del giorno | [x] | js/terrain/TerrainManager.js, js/engine/SunPosition.js |
| Overhaul prestazioni rendering (LOD, throttle) | [x] | js/terrain/TerrainManager.js, js/engine/Scene3D.js |
| Simbologia HUD IFR + nastro traiettoria inclinato | [x] | js/hud/HUDRenderer.js |
| Marker HUD prospettici conformi + FPV air-relative | [x] | js/hud/HUDRenderer.js |
| Vista horizon-lock | [x] | js/engine/Scene3D.js |
| G totale da tutti e tre gli assi (live + SITL) | [x] | js/mavlink/MAVLinkStateMapper.js |
| Variometro a energia totale accanto al VSI | [x] | js/hud/HUDRenderer.js |
| Schematico ROTOR LOAD nella schermata flight data | [x] | js/ui/UIController.js |
| Banner DEMO al posto di DISARMED in simulazione | [x] | js/ui/CommandBarController.js |
| Modelli 3D per airframe scelti da MAV_TYPE | [x] | js/main.js, models/ |
| Scorciatoie vista: T tilt, P traiettoria, M satellite, L luce | [x] | js/ui/UIController.js |
| Corridoio di traiettoria predetta | [x] | js/engine/TrajectoryPredictor.js, TrajectoryCorridor3D.js |
| Downloader offline tile + SRTM1, cache IndexedDB | [x] | js/maps/OfflineDownloader.js, TileCache.js |

## Fase 8: Log, replay, RTK e video — v1.3.x
| Task | Stato | File |
|------|-------|------|
| Registrazione .tlog automatica all'aggancio | [x] | main-mavlink.js |
| Replay .tlog + .bin DataFlash (20 Hz) | [x] | log-replay-manager.js, log-replay-bin-parser.js |
| Traiettoria replay da AHR2 (EKF) invece del GPS grezzo | [x] | log-replay-bin-parser.js |
| RTK: base seriale F9P + iniezione GPS_RTCM_DATA | [x] | rtk-manager.js |
| Client NTRIP con sourcetable | [x] | rtk-manager.js |
| Telemetry forwarding LTM / MAVLink / mirror UDP | [x] | telforward-manager.js |
| Camera FPV RTSP nativo → WebCodecs (fallback VLC → MJPEG) | [x] | fpv-manager.js, rtsp-client.js |
| Pin rate ATTITUDE/VFR_HUD via SET_MESSAGE_INTERVAL | [x] | js/mavlink/CommandSender.js |
| Traffico ADS-B (OpenSky) | [x] | js/adsb/ADSBManager.js |

## Fase 9: Parametri a richiesta singola — 2026-08-06
Motivazione: su link ad alta latenza e basso datarate (SiK 19200, LoRa) un
`PARAM_REQUEST_LIST` completo sono minuti di airtime e affama gli stream di telemetria.

| Task | Stato | File |
|------|-------|------|
| `fetchParameter()` con attesa PARAM_VALUE e 3 tentativi | [x] | js/mavlink/CommandSender.js |
| Catalogo nomi parametro per classe veicolo (~700 nomi) | [x] | js/ui/ParamCatalog.js |
| Apprendimento nomi da veicolo e da file .param, persistito | [x] | js/ui/ParamCatalog.js |
| Preferiti (★) e filtro per gruppo | [x] | js/ui/ParamCatalog.js |
| Pannello laterale con coda serializzata e stato per riga | [x] | js/ui/ParametersPageController.js |
| Timeout per richiesta selezionabile (2/4/8/15 s) | [x] | html/index.html |
| Barra di progresso riservata al solo READ ALL | [x] | js/ui/ParametersPageController.js |
| Gating UI su link senza servizio parametri | [ ] | serve `STATE.caps` (vedi Debito noto) |

## Fase 10: Missioni — undo/redo e libreria locale — 2026-08-06
| Task | Stato | File |
|------|-------|------|
| History a snapshot con deep copy | [x] | js/mission/MissionHistory.js |
| `commitMission()` su tutti gli 11 punti di modifica | [x] | js/ui/TabController.js |
| Pulsanti UNDO/REDO + Ctrl+Z / Ctrl+Y / Ctrl+Shift+Z | [x] | js/ui/TabController.js, html/index.html |
| Data root `<install>/data` con fallback userData | [x] | mission-store.js |
| `index.json` ricostruito dal contenuto delle cartelle | [x] | mission-store.js |
| Salvataggio tmp+rename (sovrascrittura sicura) | [x] | mission-store.js |
| Libreria: load / overwrite / rename / delete / save-as | [x] | js/mission/MissionLibrary.js |
| Prompt di testo in-page (Electron non ha `window.prompt`) | [x] | js/mission/MissionLibrary.js |
| Log spostati in `data/logs/` accanto alle missioni | [x] | main-mavlink.js |
| Elenco log nella libreria | [x] | js/mission/MissionLibrary.js |
| Associazione volo ↔ missione + statistiche (logbook) | [ ] | futuro |

## Fase 11: MSP / INAV — 2026-08-06
| Task | Stato | File |
|------|-------|------|
| Framing MSP v1 (`$M`, XOR) e v2 (`$X`, CRC8 DVB-S2) | [x] | msp-manager.js |
| Trasporto seriale + TCP (INAV SITL) | [x] | msp-manager.js |
| Scheduler a polling, una richiesta in volo alla volta | [x] | msp-manager.js |
| Profili di poll: normale e slow link | [x] | msp-manager.js, html/index.html |
| Back-off: comando droppato dopo 3 mancate risposte | [x] | msp-manager.js |
| Traduzione in MAVLink sintetico (30/74/24/33/27/1/147/65/0) | [x] | msp-manager.js |
| Modo di volo dai box attivi via MSP_BOXNAMES | [x] | msp-manager.js |
| Tabella modi INAV lato renderer | [x] | js/mavlink/MAVLinkStateMapper.js |
| Tipi connessione `msp-serial` / `msp-tcp` | [x] | js/mavlink/ConnectionManager.js, js/ui/TabController.js |
| Home polling disattivato su link MSP | [x] | js/mavlink/ConnectionManager.js |
| Parametri/setting MSP (0x1003/0x1004/0x1007) | [ ] | richiede nomi minuscoli e >16 caratteri |
| Missioni MSP (`MSP_WP` / `MSP2_INAV_MISSION_ITEM`) | [ ] | futuro |
| RC override MSP (`MSP_SET_RAW_RC`) | [ ] | futuro |
| Verifica su INAV reale o INAV SITL | [ ] | finora testato solo contro un FC simulato |

## Fase 12: Annunciatori di stato e lingua cinese — 2026-09-16
| Task | Stato | File |
|------|-------|------|
| Colonna annunciatori a lato del tape airspeed (rossi dall'alto, ambra dal basso sopra la minimappa) | [x] | html/index.html, css/components.css, css/layout.css |
| Regole di Mission Planner: bit SYS_STATUS, system_status, EKF, VIBRATION, GPS, MAG_CAL, STATUSTEXT | [x] | js/ui/AnnunciatorPanel.js |
| Nuovi campi STATE + mapping 191/192/193/441 | [x] | js/core/state.js, js/mavlink/MAVLinkStateMapper.js |
| Dialetto `development` registrato solo per gli id non stabili (GNSS_INTEGRITY) | [x] | main-mavlink.js |
| RSSI/LQ sotto LINK, satelliti+HDOP sotto gli alert GPS | [x] | js/ui/AnnunciatorPanel.js |
| Sequenza di allarmi casuali nella demo | [x] | js/ui/AnnunciatorPanel.js |
| Selettore lingua EN / 中文 in SYS CONFIG, persistito | [x] | html/index.html, js/core/i18n.js |
| Traduzione DOM via dizionario + MutationObserver, ritorno esatto all'inglese | [x] | js/core/i18n.js, js/core/lang-zh.js |
| Fallback font CJK con `html[lang=zh-CN]` | [x] | css/variables.css |
| `GNSS_INTEGRITY` da ArduPilot stable | [ ] | il firmware non lo emette ancora; jam/spoof oggi solo via STATUSTEXT |

## Fase 13: Link radio nel Flight Plan — 2026-09-22
| Task | Stato | File |
|------|-------|------|
| Preset radio (RFD, SiK, Herelink, Microhard, Doodle, ELRS, Crossfire) + campi: frequenza, potenza, guadagni, perdite, sensibilità, budget minimo, altezza antenna, raggio | [x] | js/mission/RouteModel.js |
| Fisica: FSPL, zona di Fresnel, diffrazione knife-edge ITU-R P.526, curvatura 4/3, classi GOOD/DEGRADED/MARGINAL/NONE | [x] | js/mission/RadioLink.js |
| Analisi lungo la rotta (run per classe, tratti persi, punto peggiore) e raster polare di copertura time-sliced | [x] | js/mission/RadioLink.js |
| Overlay immagine Web Mercator, alone lungo la rotta, linea LOS, banda + inset LINK PROFILE nel profilo | [x] | js/ui/RadioLinkView.js |
| Tasto radio accanto a camera, popover, tool "operatore" (O) + tasto PLACE + menu contestuale, layer LINK, chip e warning | [x] | js/ui/FlightPlanController.js, html/index.html, css/flight-plan.css |
| Radio ricordata in localStorage per le nuove rotte; operatore e radio salvati con la missione | [x] | js/ui/FlightPlanController.js, js/mission/RouteModel.js |
| Test offline su terreno sintetico (cresta, curvatura, antenna bassa, cancellazione) | [x] | scripts/test-radio-link.js |
| Antenne direzionali / pattern | [ ] | oggi omnidirezionali: il guadagno è costante in azimut |
| Tratte rettilinee tra i waypoint come verità (AGL = quota sul terreno sotto il waypoint); i waypoint intermedi di terrain following diventano opzione di rotta, off di default | [x] | js/mission/RouteCompiler.js, js/mission/RouteModel.js |

---

## File Creati (post-migrazione)
| File | Tipo | Descrizione |
|------|------|-------------|
| msp-manager.js | Main Process | Adapter MSP/MSP2 → MAVLink sintetico |
| mission-store.js | Main Process | Data root, libreria missioni, index.json |
| log-replay-manager.js | Main Process | Motore di replay .tlog/.bin |
| log-replay-bin-parser.js | Main Process | Parser DataFlash .bin |
| rtk-manager.js | Main Process | RTCM3 + NTRIP |
| fpv-manager.js | Main Process | RTSP nativo → WebCodecs, fallback VLC → MJPEG |
| telforward-manager.js | Main Process | LTM / MAVLink / mirror UDP |
| sitl-manager.js | Main Process | Launcher SITL ArduPilot |
| js/ui/ParamCatalog.js | Renderer | Catalogo nomi parametro + apprendimento |
| js/mission/MissionHistory.js | Renderer | Undo/redo a snapshot |
| js/mission/MissionLibrary.js | Renderer | UI libreria missioni |
| js/adsb/ADSBManager.js | Renderer | Traffico ADS-B |
| js/maps/OfflineDownloader.js | Renderer | Download offline tile + SRTM |
| js/mission/RadioLink.js | Renderer | Link budget, Fresnel, diffrazione, copertura polare |
| js/ui/RadioLinkView.js | Renderer | Overlay copertura, alone rotta, inset LINK PROFILE |

---

## Note Implementazione
- **Il contratto**: ogni protocollo si decodifica nel main process e si emette come MAVLink sul
  canale `mavlink-message`. Il renderer non conosce i protocolli. Vale per CORV binario, MSP e replay.
- MAVLink parsing nel main process (CommonJS/lazy-load); `serialport` caricato lazy per evitare
  crash ABI allo startup. `npx @electron/rebuild` necessario dopo un cambio di versione Electron.
- Tutti i JS del renderer sono ES module con import espliciti.
- `confirm()` in questa app è **asincrono** (modale in-page definita in html/index.html) e
  `window.prompt()` **non esiste** in Electron: usare `promptText()` di MissionLibrary.js.
- Il numero di modo INAV emesso da `msp-manager.js` e la tabella `INAV_MODES` in
  `MAVLinkStateMapper.js` sono due metà dello stesso contratto: vanno cambiate insieme.
- Modi ArduPilot Copter: STABILIZE=0, ACRO=1, ALT_HOLD=2, AUTO=3, GUIDED=4, LOITER=5, RTL=6, LAND=9.

## Debito noto
1. **Nessun capability model.** Con link MSP la pagina parametri, l'upload missione e i pulsanti
   ARM/modo provano comunque a inviare MAVLink: falliscono senza danni ma non sono disabilitati
   in UI. Serve un `STATE.caps` popolato per protocollo.
2. **Download missione dal veicolo** incompleto: la richiesta parte, MISSION_COUNT/MISSION_ITEM
   non vengono gestiti nel renderer.
3. **Il limite di 16 caratteri** sui nomi parametro è un vincolo MAVLink, non universale: blocca
   i setting INAV, che sono minuscoli e possono essere più lunghi.
4. **I .tlog non registrano i messaggi sintetici** (CORV, MSP): `writeTlogPacket()` serializza il
   frame MAVLink grezzo, che per quelle sorgenti non esiste.
5. **Log precedenti alla 1.5.0** restano in `%APPDATA%\CORV GCS\logs\` e non compaiono nell'indice.
