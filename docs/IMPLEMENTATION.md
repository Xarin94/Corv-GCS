<!-- Copia del documento di implementazione (Claude Doc "CORV GCS su Qt/C++: descrizione di implementazione"), esportata il 2026-11-14. Si aggiorna insieme al documento. -->
# CORV GCS su Qt/C++: descrizione di implementazione

Nov 14, 2026 · @Alle

## Scopo e perimetro

L'obiettivo è un solo eseguibile nativo Qt 6 / C++ per Windows, macOS e Linux, con le stesse funzioni della versione Electron 1.8.8. Il codice deve restare Apache-2.0 e lasciare aperta la strada a tablet Android e iOS.

Il lavoro sostituisce circa 55.000 righe: 42.000 di JavaScript nella finestra, 13.000 nel processo principale, 7.000 di CSS e 3.700 di HTML. Three.js, Leaflet, roslib e Chromium escono dal prodotto.

| Voce | Electron 1.8.8 | Qt/C++ atteso |
| --- | --- | --- |
| Installer Windows | 140 MB | 40–70 MB (stima, da misurare alla fase 1) |
| Spazio installato | 407 MB, di cui 50 MB dell'app | 100–150 MB (stima) |
| Processi | Main, renderer, GPU, utility | Uno solo, con thread dedicati |
| Frame rate | 60 fps, limitato dal vsync | Uguale: il margine c'era già |
| Tablet | Non possibile | Possibile, con lavoro a parte |

Resta fuori dalla prima versione nativa:

- Il ripiego VLC per il video: la decodifica passa a FFmpeg dentro il programma.
- Il lancio di SITL su tablet.
- Qualsiasi funzione nuova: fino alla parità si porta, non si aggiunge.

La versione Electron resta quella rilasciata finché la nativa non supera la verifica di parità. Le due leggono la stessa cartella dati.

## Decisioni di base

La scelta che condiziona tutto è la licenza: per restare Apache-2.0 si usano solo moduli Qt LGPLv3, collegati dinamicamente. Questo esclude Qt Quick 3D, Qt Graphs e Qt Canvas Painter, che sono GPLv3 o commerciali.

La conseguenza pratica è che il 3D si scrive su QRhi, l'astrazione grafica di Qt Gui, e non su Qt Quick 3D. Il prototipo in `prototypes/qt-terrain` usa Qt Quick 3D e quindi non è la base del porting: resta valido solo il suo lettore CRVG.

- **Qt 6.12 LTS**, uscita il 30 settembre 2026 ([note di rilascio](https://doc.qt.io/qt-6/qt-releases.html)).
- **C++20**, CMake, un solo repository.
- **Qt Quick / QML** per l'interfaccia; la logica sta in C++, QML fa solo presentazione.
- **QRhi dentro `QQuickRhiItem`** per il 3D: stesso codice su Direct3D 11/12, Metal, Vulkan e OpenGL ([QQuickRhiItem](https://doc.qt.io/qt-6/qquickrhiitem.html)).

| Modulo Qt | Uso | Licenza |
| --- | --- | --- |
| Core, Gui, Qml, Quick, Quick Controls, Network, Concurrent, Sql, Svg, Test | Base, interfaccia, rete, SQLite, prove | LGPLv3 (da confermare modulo per modulo alla fase 1) |
| Serial Port | Link seriali | LGPLv3 (da confermare) |
| WebSockets | rosbridge | LGPLv3 (da confermare) |
| Shader Tools, libreria | Non serve a runtime | LGPLv3 ([fonte](https://doc.qt.io/qt-6/qtshadertools-index.html)) |
| Shader Tools, comando `qsb` | Compila gli shader in fase di build | GPLv3: è uno strumento, non entra nel prodotto |
| Quick 3D | Escluso | GPLv3 o commerciale ([fonte](https://doc.qt.io/qt-6/qtquick3d-index.html)) |
| Graphs | Escluso; Charts è obsoleto | GPLv3 o commerciale ([fonte](https://doc.qt.io/qt-6/qtgraphs-index.html)) |
| Canvas Painter | Escluso | GPLv3 o commerciale ([fonte](https://doc.qt.io/qt-6/qtcanvaspainter-index.html)) |
| Location | Non usato: vedi Mappa 2D | LGPLv3 ([fonte](https://doc.qt.io/qt-6/qtlocation-index.html)) |
| WebEngine | Escluso: riporterebbe Chromium | — |

QRhi ha una garanzia di compatibilità limitata: Qt può cambiarne le API nelle versioni minori. Si resta su 6.12 per tutta la durata del porting e si isola QRhi in una sola libreria.

| Bisogno | Libreria | Licenza | Sostituisce |
| --- | --- | --- | --- |
| MAVLink | `mavlink/c_library_v2`, dialetto ardupilotmega più `corv.xml` | MIT | node-mavlink |
| Ed25519, AES-256-GCM, HMAC | OpenSSL 3 | Apache-2.0 | `crypto` di Node |
| Decodifica H.264 / H.265 | FFmpeg, solo libavcodec e libavutil, build LGPL | LGPLv2.1+ | WebCodecs e VLC |
| Joystick | SDL3 | zlib | Gamepad API |
| Modelli GLB | cgltf | MIT | GLTFLoader |
| Compressione BC1 | stb\_dxt | Dominio pubblico / MIT | Port del worker JS attuale, se più fedele |
| gzip (HGT, PMTiles) | zlib | zlib | `zlib` di Node |
| Portachiavi di sistema | QtKeychain | BSD-3 | `safeStorage` |

FFT, PMTiles, RTSP, RTCM, MSP, LTM, DataFlash e Livox SDK2 sono già codice proprio in JavaScript: si portano riga per riga, senza librerie nuove.

LGPLv3 impone tre obblighi nel pacchetto: Qt come librerie dinamiche sostituibili, testo della licenza, indicazione di dove ottenere i sorgenti di Qt. Su iOS il collegamento dinamico è un problema noto e va valutato a parte.

## Architettura

Il programma è un solo processo. Ogni lavoro che oggi sta nel main process o in un Web Worker diventa un thread con un proprietario unico dei suoi dati.

&#91;embedded content: architettura · un processo, thread e flusso dei dati\]

Il thread link è l'unico che tocca i socket; il thread render è l'unico che tocca la GPU. Tra i due passano solo dati immutabili.

| Thread | Possiede | Sostituisce |
| --- | --- | --- |
| GUI | QML, controller, editor della missione, impostazioni | Thread principale della finestra |
| Render (di Qt Quick) | Risorse QRhi e passate di disegno | WebGL e processo GPU |
| Link | Socket, seriali, parser, scheduler MSP, FTP, scrittura tlog, iniezione RTK, inoltro telemetria | Main process di Electron |
| Pool terreno, fino a 4 | Decodifica HGT, quote dei chunk, composizione tile, BC1 | I 4 tipi di Web Worker del terreno |
| Rete | Download di tile, SRTM, geozone, celle, ADS-B | `fetch` nella finestra e nel main |
| LiDAR | UDP Livox, georeferenziazione, mappa a voxel, PLY | `lidar-worker.js` |
| ROS | WebSocket rosbridge, decodifica, superfici | `RosWorker.js` |
| Video | RTSP e decodifica | `rtsp-client.js`, WebCodecs, VLC |
| Log | Indice e replay di tlog e bin, FFT | Main process, dove oggi blocca 1,25 s |

Tre regole tengono insieme il disegno:

1. **Una sola forma di messaggio.** Link MAVLink, CORV binario, MSP e replay emettono tutti lo stesso `MavMessage`. È il contratto della sezione 5.8 di `ARCHITECTURE.md` e resta invariato.
2. **Lo stato si legge, non si interroga.** Il thread link pubblica uno snapshot dello stato del veicolo. Render lo prende a ogni frame, GUI a 30 Hz. Nessun componente parla direttamente con l'autopilota per leggere.
3. **I comandi vanno in coda.** GUI chiama `CommandSender`, che accoda sul thread link e restituisce un esito asincrono con ACK, tentativi e timeout come oggi.

## Struttura del progetto

Il codice si divide in librerie statiche con dipendenze in un solo verso; solo `gpu` e `ui` conoscono la grafica. Tutto il resto si compila e si prova senza finestra.

```text
corv-gcs-qt/
├── CMakeLists.txt
├── libs/
│   ├── core/       geodesia, RingBuffer, LRU, parser di espressioni, impostazioni, log di debug
│   ├── link/       trasporti, MAVLink, CORV, MSP, LTE, FTP, demux, tlog, RTK, inoltro
│   ├── vehicle/    VehicleState, StateMapper, modi di volo, CommandSender, PoseSmoother, EkfMonitor
│   ├── mission/    RouteModel, RouteCompiler, RadioLink, CameraFootprint, INAV, libreria missioni
│   ├── terrain/    HGT, chunk e LOD, cache tile, composizione satellite, BC1, download offline
│   ├── scene/      TerrainData, CameraData, PointCloudData, pacchetto CRVG (nessuna API grafica)
│   ├── gpu/        renderer QRhi: passate, shader, risorse (unica libreria che include QRhi)
│   ├── sensors/    Livox, ROS
│   ├── video/      RTSP, decodifica
│   ├── services/   geozone, celle LTE, ADS-B, SITL, replay dei log, spettro FFT
│   └── ui/         modulo QML, view model C++, HUD, mappa 2D
├── app/            main.cpp e collegamento dei componenti
├── shaders/        sorgenti GLSL compilati da qsb
├── tests/          una cartella per libreria, più i dati di riferimento
├── third_party/
└── packaging/
```

`scene` è la traduzione diretta di `js/render`: già oggi non importa Three, Electron o il DOM. Il confine tra modello e GPU introdotto nella 1.8.1 diventa il confine tra `scene` e `gpu`.

| Oggi | Righe | Diventa |
| --- | --- | --- |
| File `.js` nella radice (main process) | 13.000 | `link`, `services`, `sensors`, `video`, `core` |
| `js/core`, `js/mavlink` | 6.200 | `core`, `vehicle` |
| `js/mission` | 3.200 | `mission` |
| `js/terrain`, `js/render` | 4.100 | `terrain`, `scene` |
| `js/engine`, `js/lidar`, parte 3D di `js/ros` | 5.700 | `gpu` |
| `js/hud`, `js/maps` | 2.800 | `ui` (item C++) |
| `js/ui`, `js/joystick`, `js/logging` | 14.900 | `ui` (view model C++ e QML) |
| `html/`, `css/` | 10.800 | QML e un file di tema |

Le righe C++ attese sono dello stesso ordine di quelle JavaScript per logica e protocolli. L'interfaccia si accorcia: QML sostituisce HTML, CSS e gran parte del codice che oggi aggiorna il DOM a mano.

## Link e protocolli

Tutto ciò che parla con il veicolo vive nella libreria `link` e gira sul thread link. Il comportamento sul filo deve restare identico byte per byte: è la parte già provata in volo.

**Trasporti.** Un'interfaccia `Transport` con `open`, `close`, `write` e il segnale `received(QByteArray)`.

| Trasporto | Classe Qt | Da conservare |
| --- | --- | --- |
| Seriale | `QSerialPort` | DTR tenuto basso alla chiusura (1.8.8); 460800 baud per CORV |
| UDP, TCP | `QUdpSocket`, `QTcpSocket` | Riconnessione; evento di chiusura dal lato veicolo |
| Cellulare | `QSslSocket` più OpenSSL | Pin SHA-256 del certificato, accesso CRV3 con firma Ed25519, record AES-256-GCM, una chiave per direzione, nonce = id sessione + contatore, controllo anti-replay |

**Demux.** `LinkDemux` separa MAVLink v1/v2 e pacchetti CORV-2 sullo stesso flusso accettando solo frame con CRC valido. È il port di `link-demux.js`, con gli stessi vettori di prova.

**MAVLink.** Parser e codifica vengono dagli header generati di `c_library_v2`.

- Identità della GCS: sysid 255, compid 190, heartbeat a 1 Hz.
- `CORV_CELL_INFO` (id 52200, CRC\_EXTRA 119) si genera da `corv.xml` con mavgen, insieme al dialetto ardupilotmega.
- Il tipo che circola è `MavMessage`: il `mavlink_message_t` grezzo più l'istante di arrivo. Chi lo consuma lo decodifica con la funzione generata del suo messaggio.

**Sorgenti non MAVLink.** CORV binario, MSP e replay dei `.bin` costruiscono veri `mavlink_message_t` con le funzioni `pack`. Oggi producono oggetti JavaScript "finti"; in C++ diventano frame reali, quindi registrabili e riproducibili.

| Sorgente | Port di | Punti delicati |
| --- | --- | --- |
| CORV v7/v8 | parte di `main-mavlink.js`, `SerialHandler.js` | Frame `A5 5A TYPE LEN PAYLOAD CRC16`; uscita sui messaggi 30, 33, 74, 26, 24, 0 |
| MSP / MSP2 | `msp-manager.js` | Scheduler a 20 ms, una richiesta in volo per volta, comando scartato dopo 3 silenzi, missioni INAV nello stesso slot |
| FTP MAVLink | `mavlink-ftp.js` | BurstReadFile con riempimento dei buchi, decodifica di `@PARAM/param.pck`, ripiego su PARAM\_REQUEST\_LIST |
| RTK | `rtk-manager.js` | Parser RTCM3, client NTRIP, iniezione con GPS\_RTCM\_DATA |
| Inoltro | `telforward-manager.js` | LTM (frame G, A, S, O), MAVLink passante, specchio UDP |
| Terreno al veicolo | `TerrainFeeder.js` | Risposta a TERRAIN\_REQUEST a 20 messaggi/s, blocchi con quote ignote non inviati |

**Registrazione.** Il `.tlog` mantiene il formato attuale: 8 byte di tempo in microsecondi little-endian, poi il frame. Parte alla connessione e si ferma alla disconnessione, come oggi.

**Traccia.** `mavlink-trace.js` diventa `LinkTrace`: eventi di link e righe ogni 10 s nel log di debug, stesso testo, così i log delle due versioni si leggono allo stesso modo.

Due difetti noti si correggono nel port invece di copiarli: la chiusura TCP dal lato veicolo deve azzerare lo stato di connessione, e la barra dei comandi deve leggere la tabella delle capacità anche su link MSP.

## Stato del veicolo e ponte verso QML

L'oggetto globale `STATE` diventa una struct `VehicleState` pubblicata come snapshot immutabile. Chi disegna la legge senza lock; chi la scrive è solo il thread link.

**Scrittura.** `StateMapper::apply(const MavMessage&, VehicleState&)` è il port di `MAVLinkStateMapper.js` come funzione pura. Porta con sé tre comportamenti da non perdere:

- La catena di priorità di posizione e velocità: GLOBAL\_POSITION\_INT per prima, VFR\_HUD solo se quella è ferma da più di 2 s, GPS\_RAW\_INT per ultima.
- Le tabelle dei modi di volo per Copter, Plane, Rover, Sub e INAV, dove il protocollo del link vince sul `MAV_TYPE`.
- I tempi di arrivo di posizione e assetto, che servono a `PoseSmoother`.

**Pubblicazione.** Dopo ogni messaggio il thread link scambia un `std::shared_ptr<const VehicleState>` atomico. La struct pesa pochi kilobyte: copiarla a qualche centinaio di messaggi al secondo non costa nulla di misurabile.

**Lettura dal render.** A ogni frame il thread render prende l'ultimo snapshot e applica `PoseSmoother`, che proietta la posa fino a 0,5 s. È la correzione della 1.8.5 per i passi a 25 Hz e va portata con i suoi test.

**Lettura da QML.** Un oggetto `Vehicle` espone proprietà raggruppate (assetto, posizione, batteria, GPS, EKF, missione, link). Un timer a 30 Hz confronta lo snapshot nuovo con il precedente e notifica solo i gruppi cambiati. Non si emette un segnale per campo a ogni messaggio.

| Oggi | In Qt |
| --- | --- |
| `STATE` letto dal ciclo a 60 fps | Snapshot letto da render e da `Vehicle` |
| `CustomEvent` (`commandAck`, `mavlinkConnectionState`, `vehicleConnected`, `connectionLost`, `missionUpdated`, `platformChanged`) | Segnali Qt su oggetti con nome, connessioni accodate tra thread |
| `onMessage(msgId, handler)` | `MessageBus::subscribe(msgid, receiver, slot)` |
| Canali IPC `corv-debug`, `corv-raw-sensor`, `corv-config-response` | Segnali dedicati del link CORV |
| `dataBuffer`: RingBuffer da 1200 campioni, 8 canali | Stessa classe in C++ su `double` |
| `STATE.mode` LIVE / FREEZE / demo | Enum della sorgente; il volo demo è una sorgente come le altre |
| `localStorage` | Classe `Settings` su un file JSON nella cartella dati |

**Origine della scena.** `SceneOrigin` resta com'è: l'origine va al primo fix di ogni flusso e poi alla HOME\_POSITION. Serve perché la GPU lavora in float32, e vale anche con QRhi. I moduli che tengono metri mondo si ricostruiscono al segnale di cambio origine.

**Navigazione relativa.** `RelativeNav` si porta intera, compresa l'integrazione sul tempo di `ATTITUDE.time_boot_ms`, così un replay dà lo stesso percorso del volo.

## Motore 3D

Il 3D è un renderer proprio su QRhi dentro un `QQuickRhiItem`, e riproduce le tecniche attuali una per una. La scena di oggi è già quasi tutta shader scritti a mano (circa 3.800 righe in 8 file), quindi Three.js si sostituisce con poco più di camera, matrici e caricamento dei modelli.

**Come si innesta.** `SceneItem` deriva da `QQuickRhiItem`. In `synchronize()`, con il thread GUI fermo, copia nel renderer l'ultimo stato del veicolo, i comandi di camera e le modifiche ai dati di scena. In `render()` disegna, sul thread render. Nessun altro punto del programma tocca QRhi.

**Shader.** Si scrivono in GLSL stile Vulkan, uno per file in `shaders/`, e CMake li compila con `qsb` verso HLSL, Metal e GLSL. Tre regole cambiano rispetto a WebGL: gli uniform stanno in blocchi std140, i sampler hanno un binding esplicito, `gl_VertexID` diventa `gl_VertexIndex`.

**Convenzioni.** Il contratto resta quello di `ARCHITETTURA-RENDERING.md`: metri, X est, Y alto, Z sud, matrici per colonne, profondità clip OpenGL. La proiezione si moltiplica per `QRhi::clipSpaceCorrMatrix()`, che assorbe le differenze di Direct3D, Metal e Vulkan.

| Elemento | Tecnica di oggi | Port su QRhi |
| --- | --- | --- |
| Quote del terreno | Texture array R16I, uno strato per chunk, `texelFetch` nel vertex shader | Texture array `R16` normalizzata con offset 32768, decodifica esatta nello shader. QRhi 6.12 non ha un formato intero a 16 bit con segno |
| Geometria del terreno | Una griglia di triangoli condivisa per passo di LOD; chunk senza mappa in istanze (circa 4 draw call) | Uguale: un buffer indice per griglia, disegno istanziato con dati per istanza |
| Mappa satellitare dei chunk | Texture BC1 con catena di mip, cache LRU da 1500 | `QRhiTexture::BC1`; RGBA8 dove BC1 manca |
| Dettaglio satellitare | 4 texture 2048² toroidali, zoom 17–20 | Uguale, con upload parziali delle sole tile entranti |
| Vista schematica | Isolinee 10 / 50 / 250 m, griglia da 1 km, acqua, batimetria nel fragment shader | Stesso shader, portato |
| Contorni | Scena → target con profondità → passata di contorno → sovrapposizioni | Tre passate QRhi con gli stessi target |
| Geozone sul terreno | Texture di copertura 1024² e texture dei pavimenti, rasterizzate in CPU | Uguale; la rasterizzazione va sul pool terreno |
| Linee spesse | Quad istanziati allargati in pixel, tratteggio, passata fantasma, stencil | Port diretto di `ThickLine.js` |
| Simboli e particelle d'acqua | Point sprite con `gl_PointSize` | Quad istanziati: la dimensione dei punti non è garantita su tutti i backend (in particolare Direct3D) |
| Nuvola LiDAR | `THREE.Points` in blocchi da 262.144 punti, upload parziali | Quad istanziati o punti da 1 pixel più dimensione in shader; stessi blocchi e stessi upload parziali |
| Etichette | Sprite da canvas | Testo disegnato con `QPainter` in un atlante, quad a dimensione fissa in pixel |
| Modelli dei veicoli | 14 file GLB con GLTFLoader, luce direzionale più ambiente | cgltf e uno shader Lambert semplice |
| Superfici ROS | Blocchi RG32F, vertici da `gl_VertexID`; mesh per chunk nel modo 3D | Port diretto di `RosMesh3D.js` e `RosVolume3D.js` |
| Sole e ombreggiatura | Normali e hillshade nello shader; il sole è un uniform | Uguale |

**Camera.** I modi di inseguimento, orbita, FPV e AR diventano una classe `CameraRig` in C++ che produce direttamente `CameraData`. Sparisce l'adattatore che oggi la ricava dalla camera Three.

**Caricamento.** Il limite storico era `CHUNKS_PER_FRAME` = 5. In C++ si tiene un tetto analogo sugli upload per frame, espresso in byte, per non allungare il frame durante uno spostamento.

**Risoluzione e frame rate.** I profili Native, Balanced ed Eco diventano la scala del target dello `SceneItem`. Il limite a 30 fps del modo eco si ottiene saltando l'aggiornamento dell'item un frame su due.

**Misure.** Il renderer espone le stesse statistiche di `getRenderPerformanceStats()`: passate, draw call, triangoli, tempo CPU di invio, più i tempi GPU dove il backend li dà. Servono al confronto di parità.

## Mappa 2D

La mappa è un componente proprio, `MapItem`, e non Qt Location. Il motivo è la cache: mini-mappa, pagina FLIGHT PLAN, texture dei chunk 3D e download offline devono leggere le stesse tile dallo stesso archivio, cosa che Qt Location non offre con API pubbliche (da riverificare sulla 6.12 prima di chiudere la decisione).

**Archivio delle tile.** `TileStore` è un file SQLite per fornitore in `<cartella dati>/tiles/`, con lo schema MBTiles (zoom, colonna, riga, immagine). Sostituisce la cache IndexedDB e resta leggibile da strumenti esterni. La politica è quella di oggi: prima la cache, poi la rete, 6 download in parallelo per lo scaricamento in blocco.

**Livelli di zoom.** Restano allineati tra i tre consumatori: chunk 3D fino a zoom 16 vicino al velivolo, dettaglio satellitare da 17 a 20, downloader offline sugli stessi livelli.

**Disegno.** `MapItem` è un `QQuickItem` in proiezione Web Mercator con tre strati:

1. Tile: nodi dello scene graph, una texture per tile, quindi disegnate dalla GPU.
2. Vettori: un `QQuickPaintedItem` per traccia, rotta, geozone e copertura radio. `QPainter` copre i motivi a bande che oggi sono pattern SVG.
3. Oggetti interattivi: elementi QML posizionati dalla proiezione, per velivolo, waypoint, maniglie di trascinamento, schede al passaggio del mouse.

| Funzione di oggi (Leaflet) | In `MapItem` |
| --- | --- |
| Mini-mappa: velivolo, traccia fino a 3000 punti, home, traffico ADS-B, clic per il punto guidato | Strati 2 e 3 |
| FLIGHT PLAN: strumenti di disegno, maniglie su vertici, punti medi, centro e raggio, menu contestuale | Elementi QML su un modello C++ dei segmenti |
| Geozone: bande no-fly e condizionali, soffitti dal più alto, solo zone entro tre volte la vista | Strato 2, stessa tavolozza di `GeoZoneStyle` |
| Copertura radio: raster polare in Web Mercator, alone lungo la rotta, linea di vista | Immagine generata in C++ e disegnata nello strato 2 |
| Profilo altimetrico con cursore sincronizzato | `QQuickPaintedItem` separato |

Il fornitore delle immagini satellitari resta configurabile come oggi. I termini d'uso del fornitore non cambiano con il port e vanno rispettati allo stesso modo.

## HUD, strumenti e grafici

Tutto ciò che oggi disegna su Canvas 2D si porta su `QPainter`, le cui chiamate corrispondono quasi una a una. Sono 12 file, di cui `HUDRenderer.js` (1.300 righe) è il più grande e l'unico che ridisegna a ogni frame.

**Il punto delicato è il costo del HUD.** In Chromium il canvas è accelerato dalla GPU. In Qt Quick un `QQuickPaintedItem` disegna in software e poi carica una texture. Un HUD a tutta finestra ridisegnato a 60 fps può costare più di oggi.

La regola di progetto è quindi:

1. Dividere il HUD in strumenti separati, ciascuno un item della sua dimensione: orizzonte, nastri di velocità e quota, bussola, variometro, grafico del carico G, messaggi.
2. Ridisegnare uno strumento solo quando il suo valore cambia di almeno un pixel.
3. Misurare alla fase 4. Il tetto è 2 ms di CPU per frame per tutto il HUD sul PC di riferimento.
4. Se uno strumento sfora, lo si riscrive come nodi dello scene graph (geometria e testo sulla GPU). I candidati sono orizzonte e nastri.

Qt Canvas Painter sarebbe il sostituto diretto del canvas accelerato, ma è GPLv3 o commerciale e resta escluso.

| Oggi | Aggiornamento | In Qt |
| --- | --- | --- |
| `HUDRenderer.js`: orizzonte, nastri, bussola, variometro, carico G, messaggi | Ogni frame | Item separati come sopra |
| `RotorLoadPanel`, `EkfStatusPanel` | Alcune volte al secondo | `QQuickPaintedItem` |
| `SpectrumViews`: spettro FFT e barra FFT | A ogni spettro | `QQuickPaintedItem` |
| Grafico delle tracce di telemetria | 10 Hz circa | `QQuickPaintedItem` sul RingBuffer |
| `RadioLinkView`: banda e riquadro LINK PROFILE | A ogni compilazione della rotta | `QQuickPaintedItem` |
| `MagCal3D`, `MountPreview` (piccole scene Three) | Durante l'uso | Un `QQuickRhiItem` piccolo che riusa le primitive di `gpu` |

Nessun modulo di grafici Qt entra nel progetto: Graphs è GPL e Charts è obsoleto. I grafici attuali sono già disegnati a mano, quindi non si perde nulla.

## Interfaccia QML

L'interfaccia si riscrive in QML sopra view model C++; è la parte più lunga del lavoro. Oggi sono 3.500 righe di `index.html` con circa 760 elementi identificati e 500 controlli, 7.000 righe di CSS e 13.500 righe di controller.

**Regola di divisione.** QML contiene disposizione, stile e binding. Ogni decisione sta in un view model C++ registrato con `QML_ELEMENT` e provabile senza finestra. Niente logica in JavaScript dentro QML oltre alle espressioni di binding.

| Controller di oggi | View model C++ | Vista QML |
| --- | --- | --- |
| `TabController` (2.300 righe) | `Navigation`, più un view model per pagina di SETUP e SYS CONFIG | `MainWindow`, `SetupPage`, `SysConfigPage` |
| `FlightPlanController` (2.500 righe) | `FlightPlanEditor` su `RouteModel`, con annulla e ripeti | `FlightPlanPage`, schede dei segmenti, ispettore |
| `CommandBarController` | `CommandBar` | Barra in basso: ARM, modo, batteria, GPS, link, timer |
| `GCSSidebarController` | `Connections`, `SitlLauncher`, `RtkPanel`, `TelemetryForward` | Barra laterale destra |
| `ParametersPageController`, `ParamCatalog` | `ParameterModel` (`QAbstractListModel` con filtro) | Lista virtualizzata con modifica in riga |
| `CalibrationController` | `Calibration` (macchina a stati per accelerometro, bussola, giroscopio) | Pagine guidate |
| `UIController` | `TelemetryCells` | Griglia 2×3 configurabile, pannelli |
| `AnnunciatorPanel`, `EkfStatusPanel` | `Annunciators`, `EkfStatus` | Pannelli |
| `LidarController`, `RosController`, `FPVController`, `CellularLinkController`, `JoystickUI` | Un view model ciascuno | Sezioni di SETUP e strisce sulla schermata di volo |

**Finestra.** Senza cornice come oggi: `Qt.FramelessWindowHint`, con spostamento e ridimensionamento affidati al sistema (`startSystemMove`, `startSystemResize`) e i tre pulsanti disegnati in QML.

**Tema.** I token di `variables.css`, scuro e chiaro, diventano un singleton QML `Theme`. I controlli partono dallo stile Basic di Qt Quick Controls e si personalizzano una volta in una cartella `controls/`. L'effetto vetro dei pannelli diventa un fondo traslucido senza sfocatura: sfocare il 3D dietro costa una passata in più a ogni frame.

**Lingua.** Il dizionario cinese è già indicizzato per stringa inglese, che è il modello di `qsTr()`. Uno script genera il file `.ts` da `lang-zh.js`; il cambio a caldo usa `QTranslator` e `retranslate()`. Sparisce il `MutationObserver`.

**Capacità dello stack di volo.** La tabella di `Platforms.js` resta l'unica fonte. Ogni controllo che lo stack non può usare resta visibile, disabilitato, con il motivo nel tooltip: una proprietà `Capability` collegata al controllo lo fa in una riga.

**Scorciatoie e carattere.** Le scorciatoie di `docs/shortcuts.md` diventano elementi `Shortcut`. I font di `vendor/fonts` si caricano dalle risorse con `QFontDatabase`.

**Compilazione.** I file QML si compilano in anticipo con gli strumenti di Qt (`qmlcachegen`, `qmlsc`), così l'avvio non interpreta nulla e gli errori di tipo escono in build.

## Video FPV

Il client RTSP attuale si porta in C++ e la decodifica passa a libavcodec con accelerazione hardware. VLC non serve più: il decoder software di FFmpeg è il ripiego dentro il programma.

**Ricezione.** `RtspClient` è il port di `rtsp-client.js` su `QTcpSocket` e `QUdpSocket`, sul thread video. Si conserva tutto il comportamento provato con la SIYI HM30:

- RTSP 1.0 senza autenticazione, keep-alive con GET\_PARAMETER.
- RTP su TCP interlacciato; UDP quando il server risponde 461; finestra di riordino di 32 pacchetti.
- Depacketizzazione RFC 6184 e RFC 7798 in access unit Annex-B, con i parameter set anteposti ai key frame.
- Dopo una perdita si scarta fino al key frame successivo.

**Decodifica.** Un `VideoDecoder` avvolge libavcodec con il flag di basso ritardo.

| Sistema | Decoder hardware | Ripiego |
| --- | --- | --- |
| Windows | D3D11VA | Software |
| macOS | VideoToolbox | Software |
| Linux | VAAPI | Software |

Se il decoder resta indietro si salta al key frame successivo, come fa oggi `FPVController`.

**Presentazione.** Il frame decodificato si copia in memoria come NV12 e si carica in due texture; la conversione in RGB è nello shader. A 1080p sono circa 3 MB per frame: trascurabile. Il percorso senza copia si valuta solo se le misure lo chiedono.

Il video è la prima passata dello `SceneItem`. Nel modo AR il 3D si disegna sopra con fondo trasparente; nel modo CAMERA il 3D non si disegna affatto, come oggi.

**Perché non Qt Multimedia.** `QMediaPlayer` apre un indirizzo RTSP, ma non lascia controllare buffer e ritardo, e sostituirebbe un client già messo a punto. Qui la latenza conta più della comodità.

La build di FFmpeg contiene solo i decoder H.264 e H.265 e le accelerazioni elencate, senza componenti GPL, e resta collegata dinamicamente.

## Altri sottosistemi

I sottosistemi restanti sono port diretti: la logica non cambia, cambia solo dove gira. Due di essi correggono nel passaggio un blocco noto del main process.

| Sottosistema | Port di | Cosa cambia in C++ |
| --- | --- | --- |
| Pianificazione | `RouteModel`, `RouteCompiler`, `CameraFootprint`, `RadioLink`, `InavMission`, `MissionHistory` | Nulla nella logica: sono funzioni pure. Il formato `corv-gcs-mission` versione 2 resta identico |
| LiDAR Livox | `lidar-core.js` | Thread proprio con `QUdpSocket` (ricerca su 56000, configurazione su 56100, punti su 56301). Scrive direttamente in `PointCloudData`: sparisce la copia attraverso l'IPC |
| ROS | `RosWorker`, `RosPoints`, `SurfaceTiles`, `SurfaceRaster`, `SurfaceVolume` | `QWebSocket` verso rosbridge, CBOR con le classi di Qt Core al posto di roslib |
| ADS-B | `ADSBManager` | `QNetworkAccessManager`; stessi limiti: 50 km, una richiesta ogni 10 s, voci scadute a 60 s |
| Joystick | `JoystickManager` | SDL3 letto a 25 Hz. Gli indici degli assi di SDL non coincidono con quelli del browser: serve una conversione delle mappature salvate |
| Geozone | `geozones.js`, `GeoZoneRules`, `GeoZones` | Stesse fonti e stessa cache JSON in `<cartella dati>/geozones`, quindi condivisa con Electron |
| Celle LTE | `cell-db.js` | Lettore PMTiles v3 portato, richieste HTTP a intervalli di byte, stessa cartella `celldb/` |
| SITL | `sitl-manager.js` | `QProcess`; su Windows via WSL. La ricerca dell'indirizzo dell'host WSL diventa asincrona: oggi blocca 5,7 s |
| Replay dei log | `log-replay-manager.js`, `log-replay-bin-parser.js` | Indice costruito sul thread log, con il file mappato in memoria e soli offset nell'indice. Oggi: 1,25 s di blocco e 884 MB di heap su un tlog da 202 MB |
| Spettro FFT | `log-spectrum.js` | FFT e stima di Welch portate con i loro vettori di prova |
| Download offline | `OfflineDownloader`, `TileCache` | SRTM1 compressi da decomprimere con zlib, salvati in `<cartella dati>/terrain` |
| Log di debug | `app-log.js`, `DebugLog.js` | Gestore dei messaggi di Qt; stesso file, stessa finestra di 5 minuti, stessa intestazione |
| Volo demo e LiDAR demo | Parte di `js/main.js`, `LidarDemo.js` | Una sorgente di messaggi come le altre; serve anche alle prove senza veicolo |

Lo script `scripts/rosbridge-sim.js` e il simulatore Livox restano strumenti Node di sviluppo. Non entrano nel pacchetto e servono a provare la versione nativa contro gli stessi dati.

## Dati su disco e compatibilità

La versione nativa usa la stessa cartella dati e gli stessi formati, così le due versioni convivono sulla stessa macchina. Fanno eccezione le impostazioni e la cache delle tile, che oggi stanno nel profilo di Chromium e vanno esportate.

**Dove sta la cartella.** La regola resta quella di `mission-store.js`: `data/` accanto all'eseguibile se la cartella è scrivibile, altrimenti la cartella per utente. Quest'ultima va indicata con il percorso esatto usato da Electron (su Windows `%APPDATA%\CORV GCS\data`), non con quello che Qt sceglierebbe da solo.

| Cartella o file | Stato | Nota |
| --- | --- | --- |
| `index.json`, `missions/` | Identico | Formato `corv-gcs-mission` versione 2; i file versione 1 si ricostruiscono al caricamento |
| `logs/` | Identico | `.tlog` e `.crv` |
| `debug/` | Identico | `corv-gcs-debug.log` e `.prev.log` |
| `terrain/` | Identico | Tile SRTM `.hgt`; restano validi `topo/` e `topography/` accanto all'installazione, senza distinzione tra maiuscole e minuscole |
| `celldb/`, `geozones/`, `lidar/` | Identico | Cache e file PLY |
| `models/` accanto all'installazione | Identico | GLB |
| `tiles/` | Nuovo | SQLite con schema MBTiles; sostituisce IndexedDB |
| `settings.json` | Nuovo | Sostituisce `localStorage` |

**Scritture.** Ogni file si scrive con `QSaveFile`, che fa file temporaneo e rinomina: è la stessa garanzia che oggi protegge missioni e tile.

**Migrazione di ciò che sta in Chromium.** Qt non può leggere il profilo di Electron. Serve un'ultima versione Electron con un comando di esportazione:

1. Scrive `settings.json` con tutte le chiavi di `localStorage`: celle del HUD, mappatura del joystick, nomi e preferiti dei parametri, impostazioni FPV, LiDAR e ROS, ultimo punto di casa, profilo radio, stack di volo, lingua.
2. Copia la cache IndexedDB delle tile in `tiles/`.
3. La versione nativa importa `settings.json` al primo avvio e converte la mappatura del joystick agli indici di SDL.

La chiave AES del link cellulare non si migra. È cifrata dal portachiavi attraverso Electron e l'operatore la reinserisce una volta; poi la conserva QtKeychain.

## Build, pacchetti e CI

La build è CMake con preset per i tre sistemi, e tutti e tre i pacchetti escono da GitHub Actions. Oggi Windows si compila in locale e solo macOS e Linux passano dalla CI.

| Sistema | Compilatore | Backend grafico predefinito | Pacchetto |
| --- | --- | --- | --- |
| Windows | MSVC 2022 | Direct3D 11 | `windeployqt`, poi installer NSIS con scelta della cartella, come oggi |
| macOS | Apple Clang | Metal | `macdeployqt`, un DMG per arm64 e uno per x64, come oggi |
| Linux | GCC sulla Ubuntu LTS più vecchia da supportare | OpenGL | AppImage e deb, come oggi |

**Dipendenze.** Qt si installa in CI dall'archivio ufficiale. OpenSSL, FFmpeg, SDL3 e zlib arrivano da vcpkg con versioni bloccate. Le librerie a singolo header (MAVLink generato, cgltf, stb\_dxt) stanno in `third_party/`.

**Identità.** Restano l'identificativo `com.corvsystems.gcs` e il nome "CORV GCS". La versione sta in un punto solo, `project(VERSION)`, e da lì arriva a finestra, log e installer.

**Dettagli che oggi risolve Electron con un flag:**

- GPU dedicata sui portatili: in Windows si esportano i simboli `NvOptimusEnablement` e `AmdPowerXpressRequestHighPerformance`. Sostituisce `force_high_performance_gpu`.
- Backend grafico forzabile per diagnosi con la variabile `QSG_RHI_BACKEND`; il backend scelto va scritto nell'intestazione del log di debug.
- Finestra coperta da un'altra: Chromium smette di disegnare, Qt Quick si ferma solo quando la finestra non è esposta, per esempio ridotta a icona. Da verificare sui tre sistemi e da decidere se limitare il frame rate in quel caso.

**CI.** A ogni push: build e test senza finestra sui tre sistemi. A ogni tag: pacchetti e pubblicazione della release. Le prove che aprono una finestra girano su un runner con GPU, o in locale.

**Obblighi LGPL nel pacchetto.** Le librerie Qt e FFmpeg restano file separati e sostituibili. L'installer include i testi delle licenze e una pagina "Licenze" nell'app indica dove trovare i sorgenti.

## Verifica di parità

La versione nativa si considera pronta quando dà gli stessi risultati di Electron sugli stessi dati, a sei livelli. La versione Electron fa da oracolo per tutto il porting.

| Livello | Come si verifica | Criterio |
| --- | --- | --- |
| Funzioni pure | I 17 script `scripts/test-*.js` si riscrivono in Qt Test con gli stessi vettori: HGT, demux, FTP, PoseSmoother, EKF, geozone, celle, spettro, joystick, matematica LiDAR, link radio, superfici ROS, architettura del rendering | Tutti verdi sui tre sistemi |
| Uscite di riferimento | Uno script Node esegue i moduli JS (compilatore della rotta, link radio, StateMapper, parser bin) su casi campione e salva il risultato in JSON; i test C++ lo confrontano | Uguali, con tolleranza numerica dichiarata per campo |
| Stato dal replay | Lo stesso `.tlog` entra nelle due versioni; ciascuna scrive lo stato a 10 Hz | Campo per campo entro la tolleranza |
| Byte sul filo | Contro SITL si catturano i frame inviati da Electron per armamento, cambio modo, caricamento missione, scrittura parametri, FTP | Stessi frame, a parte sequenza e tempo |
| Immagine | Scene fisse: stato congelato, sole a un'ora fissa, Innsbruck e pianura padana (45,05 N, 9,70 E), vista satellitare e schematica | Differenza di pixel sotto soglia; si escludono etichette ADS-B, traccia e lampeggi |
| Prestazioni | Scenari di `PIANO-MIGRAZIONE-GRAFICA.md`: 400–550 chunk, 1 e 3 milioni di punti, missione lunga, FPV e AR; finestre visibili, almeno 5 esecuzioni | Non peggio della base qui sotto su frame e latenza; meno RAM e avvio più breve |

Base misurata il 5 ottobre 2026 su Ryzen AI 9 365 con Radeon 880M, a batteria, 1800×993: 60 fps in tutte le viste; tempo GPU mediano 2,2 ms in satellite e 3,4 ms in schematico; invio CPU mediano 1,1 ms; memoria di lavoro tra 1,45 e 1,9 GB.

**Strumento che sostituisce CDP.** Oggi le prove guidano l'app attraverso il protocollo di debug di Chromium. La versione nativa offre, solo con l'opzione `--debug-port`, un piccolo server locale JSON-RPC con quattro comandi: imposta lo stato, esegui un'azione di un view model, leggi le statistiche di rendering, salva uno screenshot (`QQuickWindow::grabWindow`). Va scritto alla fase 3, perché tutte le verifiche successive lo usano. Fatto: `app/DebugServer` ascolta solo su 127.0.0.1, una richiesta JSON-RPC 2.0 per riga, con i metodi `set`, `get`, `call` (con il valore di ritorno), `stats`, `screenshot` (su file o in base64) e `script` per i comandi di `--grab-script`; `tools/debug-client.py` manda una richiesta dalla riga di comando.

**SITL.** Si ripetono gli scenari già in uso: multirotore, aereo jet600, ROV con e senza GPS, barca a vela sul Garda.

**Ciò che il simulatore non copre** si prova sull'hardware: DTR della seriale, camera SIYI, Livox Mid-360, modulo LTE, joystick, base RTK.

## Fasi di lavoro

Le fasi vanno dal fondo verso l'interfaccia, e ognuna si chiude con una prova contro la versione Electron. Dopo la fase 3 c'è un punto di decisione: è il primo momento in cui si può misurare se il nativo dà un vantaggio reale.

Non ci sono stime di durata: dipendono da chi lavora al porting e non ho dati per darle. Come ordine di grandezza, la versione Electron ha richiesto 8 mesi e 174 commit.

1. **Fondazioni.** Repository, CMake, CI sui tre sistemi, licenze dei moduli confermate, finestra QML vuota impacchettata.
   - Uscita: i tre installer prodotti dalla CI si avviano; dimensione dell'installer misurata.
2. **Link e stato, senza interfaccia.** Librerie `core`, `link`, `vehicle`; uno strumento a riga di comando che si collega a SITL e stampa lo stato.
   - Uscita: confronto dello stato dal replay e dei byte sul filo superati.
3. **Terreno 3D.** Librerie `terrain`, `scene`, `gpu`: vista schematica e satellitare, camera, modello del veicolo, traccia; server di debug.
   - Uscita: parità d'immagine sulle scene fisse e prestazioni misurate sugli scenari di terreno.
   - **Decisione: continuare o fermarsi.** La soglia proposta in `PIANO-MIGRAZIONE-GRAFICA.md` è un miglioramento ripetibile del 25% sul p95 dei frame o sul costo CPU, senza peggiorare il resto. Se non c'è, il porting si giustifica solo con i tablet o con la dimensione del pacchetto.
4. **Schermata di volo.** HUD, barra dei comandi, barra laterale delle connessioni, mini-mappa, annunciatori, stato EKF.
   - Uscita: un volo SITL completo (connessione, armamento, decollo, punto guidato, rientro) condotto dalla versione nativa; HUD sotto 2 ms per frame.
5. **Missione.** Pagina FLIGHT PLAN, compilatore, trasferimento, libreria, missione in 3D, geozone, link radio.
   - Uscita: le missioni salvate da Electron si aprono e producono gli stessi item; caricamento e lettura contro SITL.
6. **Setup.** Parametri e FTP, calibrazioni, joystick, RTK, inoltro telemetria, MSP e missioni INAV, CORV binario, link cellulare.
   - Uscita: ogni pagina provata contro SITL o sull'hardware.
7. **Sensori e video.** FPV, LiDAR, ROS, ADS-B, celle LTE.
   - Uscita: simulatori di sviluppo e una prova sull'hardware per ciascuno.
8. **Log e servizi.** Replay, spettro FFT, log di debug, download offline, lancio di SITL.
   - Uscita: gli stessi file di log danno le stesse tracce e gli stessi spettri.
9. **Rilascio.** Esportazione delle impostazioni da Electron, pacchetti firmati, sito `docs/` aggiornato, beta affiancata a Electron.
   - Uscita: lista di parità completa e almeno un ciclo di rilascio con le due versioni in parallelo.

Le fasi da 5 a 8 sono indipendenti tra loro e possono procedere in parallelo se lavora più di una persona.

## Stato dell'implementazione

Al 14 novembre 2026 la versione nativa compila su Windows e su Linux, si installa su Windows con un installer NSIS da 35 MB e vola su SITL con link, stato del veicolo, terreno 3D, HUD e pagina FLIGHT PLAN. Nessuna fase ha ancora superato la sua prova di uscita. Le fasi non sono andate in ordine: la 5 è più avanti della 3 e della 4. Il codice è in `Desktop\Corv gcs 2.0.0` e nel repository `gcs3`, circa 56.000 righe tra C++, QML, shader, script e prove.

| Fase | Stato | Fatto | Manca |
| --- | --- | --- | --- |
| 1 Fondazioni | In corso | CMake con preset per i tre sistemi; finestra QML senza cornice; CI con build e test su Windows, macOS e Linux (Qt per Windows da `tools/ci/install-qt-windows.py`, runner macOS 15); versione in `project(VERSION)`; simboli per la GPU dedicata; backend grafico nell'intestazione del log; AppImage Linux (`tools/build-appimage.sh`, 47 MB, provata su Ubuntu con Wayland); installer Windows NSIS (`tools/package-windows.ps1`): per utente senza diritti di amministratore, scelta della cartella, runtime Visual C++ accanto all'eseguibile, libcrypto, SDL3, le celle LTE incluse e gli avvisi delle licenze, disinstallazione che lascia la cartella `data`; 35 MB contro i 140 MB di Electron, installato accanto a CORV GCS 1.x; la CI lo produce sui tag `v*` insieme allo zip portatile ; la CI produce sui tag anche l'AppImage (Ubuntu 22.04) e il DMG macOS (bundle .app, macdeployqt, universale, firma ad hoc) e li allega alla release insieme ai pacchetti Windows; la versione completa viene dal tag (`CORV_VERSION_LABEL`, `-dev` nelle build locali); controllo degli aggiornamenti all'avvio sulle release pubbliche di `Xarin94/Corv-GCS` con installazione a un clic (setup Windows `/UPDATE` silenzioso nella stessa cartella, AppImage sostituita sul posto e riavviata, DMG aperto), SHA-256 verificato | Pacchetti firmati e notarizzati; release nel repository pubblico |
| 2 Link e stato | In corso | Seriale, UDP, TCP; MAVLink (ardupilotmega, development, CORV_CELL_INFO); CORV-2 binario; LinkDemux, su ogni seriale oltre che sul cellulare; tlog; LinkTrace, con la diagnosi della ricezione (nessun byte, byte senza frame validi) nel log e sul HUD; StateMapper e modi di volo; PoseSmoother; SceneOrigin; RelativeNav; VehicleHub; CommandSender con comandi, parametri, calibrazioni e missione; MAVLink FTP e @PARAM/param.pck (READ ALL di 1433 parametri in 0,08 s su SITL); ConnectionManager; volo demo; `corv-link-cli`, anche `lte`; link cellulare verso un modulo LTEtelem tramite relay (TLS con certificato fissabile, accesso CRV3 Ed25519, AES-256-GCM end to end, anti-replay, riconnessione), provato da capo a capo con il relay vero in MAVLink e in CORV-2; RTK / GPS INJECT (NTRIP v1/v2, base seriale, GPS_RTCM_DATA); inoltro della telemetria (specchio MAVLink, LTM); MSP v1 e v2 su seriale e TCP (MspSession: stato del veicolo da INAV e Betaflight, missioni INAV in lettura e scrittura), provato contro una scheda simulata; 20 gruppi di test Qt Test, tutti verdi. Provato su SITL | MSP su una scheda vera; prova sulla scheda della seriale USB corretta (DTR); confronto dello stato dal replay e dei byte sul filo |
| 3 Terreno 3D | In corso | HgtStore e download SRTM1, anche automatico per i tile mancanti attorno al veicolo; chunk da 1/30° nel raggio 35 / 50 / 70 km; renderer QRhi (texture array R16, disegno istanziato, LOD sull'errore in pixel, frustum culling); vista schematica e palette altimetrica; nebbia; immagini satellitari dei chunk (BC1 con mip, RGBA8 su OpenGL); linee spesse in pixel con passata fantasma per traccia, corridoio della traiettoria prevista, home, rotta della missione e traffico ADS-B; modelli GLB del tipo di veicolo; prima persona con sguardo sulle frecce e blocco dell'orizzonte, terza persona in orbita; clipping sul terreno selezionabile; superficie dell'acqua e particelle sott'acqua; sole per ora e luogo; etichette di HOME, missione e traffico da un atlante di testo; SatelliteDetail (quattro texture a clipmap degli zoom 17–20 attorno al veicolo, SYS CONFIG → SATELLITE DETAIL); passata di contorno della vista schematica (creste e skyline trovati nella profondità, tre passate); geozone UAS dipinte sul terreno; pali delle celle LTE e collegamento tratteggiato al ripetitore servente; server di debug (`--debug-port`) | Parità d'immagine e misure, quindi la decisione di continuare; il modello del veicolo non ha contorno |
| 4 Schermata di volo | In corso | HUD diviso in strumenti (0,1–0,3 ms medi, meno di 1,5 ms di picco per frame), con il solo nastro VS della velocità verticale (il variometro a energia totale è stato tolto), che segue lo sguardo e sparisce in terza persona; HUD blu sul tema chiaro; celle dati 2×3; banner DISARMED / DEMO; barra dei comandi; GCS OPTIONS; mini-mappa con traccia e home, scambiata al clic con la vista 3D; scorciatoie T, P, M, L, C; annunciatori; barra EKF STATUS (EKF3); barra FFT (spettro, waterfall di 60 s); ROTOR LOAD e SAIL · RUDDER, con i motori ricostruiti dai SERVOn_FUNCTION (MotorN sull'anello, spinta del quadplane o motori del bimotore al centro; servi di coda del tri, alette del coassiale e servi dell'elicottero esclusi) e l'indicatore FLAPS sopra quando un'uscita porta flap o flaperoni (stile EIS Airbus, da −10° a +30°, 100 % = 30°); i parametri dei servi sono chiesti solo con ROTOR LOAD attivo; banner del cambio modo; tabella ADS-B; righe della cella LTE. Provati su SITL (aereo, barca a vela) | Volo SITL completo |
| 5 Missione | In corso | RouteModel, RouteCompiler (anche INAV), MissionTransfer, MissionStore v2, Platforms, RadioLink; FLIGHT PLAN con annulla e ripeti, mappa a strati, profilo con banda radio; libreria missioni; caricamento e lettura della missione; rotta compilata nella vista 3D con simboli, etichette, linee al suolo e cerchi di loiter; missioni INAV su MSP; geozone UAS (file ED-269 / ED-318, FAA, DFS DIPUL, IGN, ENAIRE, BAZL) sulla mappa con legenda, suggerimenti e annunciatori | Prova con le missioni salvate da Electron e del trasferimento su SITL |
| 6 Setup | In corso | SETUP → CONNECTION, PARAMETERS, ACCELEROMETER, COMPASS, GYRO / BARO, CELLULAR LINK, TELEMETRY FORWARD, RTK / GPS INJECT, FLIGHT MODES, FAILSAFE, PID TUNING, EXT TUNING, RADIO CAL, VIBRATIONS, SERVO / RELAY, CORV SETUP, SIMULATION; SYS CONFIG con tutte le sezioni (chiavi di Electron in `settings.json`), compreso TERRAIN CLIPPING. Provate su SITL fin dove lo permette un veicolo fermo ; JOYSTICK / RC (SDL3 in `libs/input`: gamepad nella disposizione standard di Chromium, altri joystick grezzi; 8 o più ingressi sui canali RC con deadzone ed expo; RC_CHANNELS_OVERRIDE a 10, 25 o 50 Hz con SYSID_MYGCS, RC_OVERRIDE_TIME, RC_OPTIONS e FS_THR_ENABLE salvati e ripristinati; failsafe a 500 ms; pulsanti per i servo e per lo sguardo) | Calibrazione bussola fino a SUCCESS; LIDAR; ROS; joystick provato solo senza dispositivo |
| 7 Sensori e video | In corso | ADS-B da OpenSky e da ADSB_VEHICLE (tabella, mini-mappa, vista 3D); cella LTE del modulo (CORV_CELL_INFO) nella schermata di volo e in CELLULAR LINK; database delle celle (CellDb: file CSV dell'utente, le celle di Italia, San Marino e Vaticano in `celldb/` accanto all'eseguibile, la copia PMTiles 2024 di OpenCelliD letta a blocchi con richieste HTTP range, ricerca online su beaconDB e sull'API OpenCelliD) e mappa delle antenne: siti entro 10 km e ripetitore servente sulla mini-mappa e nella vista 3D, posizione e distanza del ripetitore in CELLULAR LINK e nella schermata di volo. 581.117 celle del pacchetto caricate in 0,5 s; un blocco dell'archivio in 5 s; joystick | FPV; LiDAR; ROS |
| 8 Log e servizi | In corso | Log di debug a 5 minuti; download offline di SRTM e tile satellitari; archivio delle tile in SQLite; LOG REPLAY di .tlog e .bin sul thread log (un .bin da 140 MB indicizzato in 0,29 s); spettri dai campioni IMU dei log (Welch) | Nulla per ora |
| 9 Rilascio | Non iniziata | Nulla | Esportazione da Electron, pacchetti firmati, docs, beta affiancata |

Scostamenti dal piano e difetti corretti nel port:

- Oltre al server di debug restano le opzioni di prova `--grab`, `--grab-script` (comandi `tab`, `emit`, `call`, `set`, `get`, `key` / `keyup`, `size`, `stats`: gli stessi del server, in `app/TestDriver`) e `--quit-after`; `CORV_DEMO_ORIGIN=lat,lon` sposta il volo demo, per esempio su un lago.
- Le catture ravvicinate (una ogni pochi decimi di secondo) tengono occupato il thread GUI e fermano la sincronizzazione della vista 3D finché durano: nelle prove si cattura di rado. Direct3D 11 non disegna fotogrammi con la finestra coperta; le verifiche visive a finestra coperta usano `QSG_RHI_BACKEND=opengl`.
- Nella vista schematica il modello del veicolo non ha contorno: in Electron faceva parte del livello del mondo. Qui è disegnato con le sovrapposizioni, per non dover tenere una seconda pipeline del modello per il target senza multicampionamento.
- Il joystick legge i dispositivi anche con la finestra in secondo piano (SDL), mentre Chromium li nascondeva e l'override si sospendeva: i pulsanti (servo, sguardo) contano comunque solo a finestra attiva. La libreria `libs/input` non era nel piano.
- Il database delle celle legge anche i file OpenCelliD senza intestazione (l'ordine delle colonne di OpenCelliD), che in Electron davano errore. La cartella `celldb/` della cartella dati ha la stessa struttura, quindi blocchi scaricati, file importati e ricerche online valgono per entrambe le versioni.
- Il grafico del carico G di HUDRenderer.js non si porta: in Electron nessun canvas lo usa. L'asse delle frequenze della barra FFT è logaritmico, su richiesta.
- Indice dei .tlog: corretta la compressione dei buchi oltre il minuto, che in Electron rendeva negativi i tempi.
- Corretto un crash intermittente presente dall'inizio (0xC0000409 in Qt6Gui.dll): una risorsa QRhi sostituita durante il frame veniva distrutta con upload ancora in sospeso.
- Inoltro LTM: rollio, beccheggio e prua in gradi; Electron mandava radianti arrotondati, quasi sempre 0.
- Seriale: `QSerialPort::close()` su Windows rimetteva DTR alto subito prima di chiudere, e l'USB dell'esp32s31corv si ammutoliva. Ora DTR e RTS restano bassi e nulla viene ripristinato, come node-serialport con `hupcl: false`.
- Terreno 3D: i tile SRTM mancanti non venivano mai scaricati, e senza `topo/` la scena restava nera. Ora si scaricano da AWS attorno al veicolo.
- Volo demo: guarda il terreno 150, 300 e 600 m avanti e non scende mai sotto 15 m dal suolo.
- Lo sguardo con le frecce ha un timer proprio a 60 Hz: non dipende dai fotogrammi, che Direct3D sospende quando la finestra è coperta.
- REMEMBER KEY del link cellulare nel portachiavi di sistema (DPAPI, Secret Service); su macOS la chiave va scritta ogni volta e non è condivisa con Electron.
- La build macOS universale per ora è senza link cellulare: la libcrypto di Homebrew ha una sola architettura. In locale su Windows OpenSSL 3.5 per MSVC sta in `build/tools`.
- MagCal3D, ROTOR LOAD e FLAPS sono `QQuickPaintedItem` disegnati solo quando cambiano; il banner del cambio modo non ha la sfocatura dello sfondo.

## Rischi e punti da decidere

Quattro decisioni vanno prese prima di scrivere codice, perché cambiano il progetto e non solo i dettagli.

| Decisione | Scelta assunta in questo documento | Cosa cambia con l'altra scelta |
| --- | --- | --- |
| Licenza di Qt | Open source LGPLv3, progetto Apache-2.0 | Con la licenza commerciale tornano disponibili Quick 3D, Canvas Painter e Graphs, il collegamento statico e iOS senza vincoli; il renderer QRhi e il HUD in `QPainter` si potrebbero evitare |
| Tablet Android e iOS | Solo strada aperta, non nel perimetro | Interfaccia da ripensare per il tocco, texture ASTC o ETC2 al posto di BC1, niente SITL, permessi USB e rete per piattaforma |
| Convivenza con Electron | Electron resta il prodotto fino alla parità | Fermare le funzioni nuove su Electron accorcia il porting ma blocca gli utenti per tutta la durata |
| Sistemi e GPU minimi | Gli stessi di oggi | Abbassare il minimo è uno dei pochi vantaggi concreti del nativo e va dichiarato come obiettivo per poterlo misurare |

Rischi tecnici, dal più pesante:

- **Volume dell'interfaccia.** È metà del codice e non si porta: si riscrive. È anche la parte che l'operatore vede, quindi ogni differenza si nota.
- **Regressioni sui protocolli.** MAVLink, MSP, FTP, RTK e link cellulare sono provati in volo. Le prove sui byte e sul replay riducono il rischio, non lo azzerano.
- **Tre backend grafici invece di uno.** Oggi Chromium traduce tutto attraverso un solo strato e assorbe i difetti dei driver. Con QRhi ogni shader va provato su Direct3D, Metal e OpenGL.
- **Costo del HUD in software.** Coperto dalla regola dei 2 ms e dal ripiego sullo scene graph, ma è lavoro in più se scatta.
- **API QRhi instabili tra versioni minori.** Coperto restando su 6.12 e isolando QRhi nella libreria `gpu`.
- **Decodifica video su Linux.** VAAPI varia con driver e distribuzione; il ripiego software deve reggere 1080p.
- **Perdita degli strumenti di Chromium.** DevTools, profilatore e CDP spariscono. Il server di debug e gli strumenti di Qt li sostituiscono solo in parte.

Verifiche ancora aperte, da chiudere alla fase 1:

- [ ] Licenza LGPLv3 di ogni modulo Qt usato, letta sulla pagina del modulo per la 6.12.
- [ ] Disponibilità delle correzioni della 6.12 LTS per chi usa la versione open source.
- [ ] Texture array in formato `R16` e texture BC1 su tutti i backend, in particolare Metal su Apple Silicon. Su Linux con Mesa radeonsi le texture array BC1 escono nere sul backend OpenGL di QRhi (i caricamenti riescono, il campionamento dà nero) e si vedono su Vulkan: su OpenGL le immagini satellitari vanno in RGBA8, `CORV_SAT_FORMAT=bc1` o `rgba8` forza un formato per le prove.
- [ ] Dimensione dei punti nel vertex shader: su quali backend manca davvero. Per ora non serve: particelle sott'acqua e simboli sono quad istanziati, provati su Direct3D 11.
- [ ] Se Qt Location 6.12 permette una cache di tile condivisa con API pubbliche.
- [ ] Che nessun messaggio sintetico di CORV, MSP o replay porti campi fuori dallo schema MAVLink; se sì, vanno aggiunti a `corv.xml`.
- [ ] Obblighi LGPL dentro un pacchetto macOS firmato e notarizzato.
