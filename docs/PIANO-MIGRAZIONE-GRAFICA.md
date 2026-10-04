# Migrazione grafica CORV GCS — 4 ottobre 2026

**Aggiornamento architettura:** la fase 3 è implementata per terreno, camera e
LiDAR Livox. La GCS usa i nuovi adapter Three e il client Qt legge uno snapshot
binario dei medesimi dati. Dettagli, contratto e limiti in
[ARCHITETTURA-RENDERING.md](ARCHITETTURA-RENDERING.md). La baseline comparativa e
il porting dei materiali/UI della fase 4 rimangono da completare.

La prima fase è implementata: conversione HGT sul worker, avvio dei chunk dopo
conferma di registrazione, profili di risoluzione 3D e separazione della gestione
del renderer. Qt/QML è un candidato concreto per un client nativo, ma non c'è
ancora una misura che dimostri che l'intera GCS sarebbe più veloce.

## Decisione attuale

Mantenere operativo Three.js/WebGL2 durante la migrazione e confrontare due
percorsi: Three.js/WebGPU e Qt Quick/QML + Qt Quick 3D. Evitare un motore completo
da zero: gestione dei dispositivi, shader, asset, picking, font, video e fallback
assorbirebbero lavoro senza risolvere automaticamente i colli di bottiglia CPU.
Il renderer specializzato per terreno e sensori può crescere sopra un backend
esistente, con risultati misurabili a ogni fase.

Gli artefatti dell'audit esplorativo sono stati rimossi durante la pulizia del repository.
Il terreno usa già shader GPU per altezza/normali e geometrie condivise. La
conversione big-endian HGT era invece ripetuta in tre percorsi del thread UI;
il main process Node leggeva i file attraverso `fs.promises.readFile`.

## Modifiche già applicate

### HGT fuori dall'interfaccia

`TerrainWorker.js` riceve `prepareHgt` con un File/Blob, legge il buffer nel
worker, converte gli Int16 big-endian nello stesso buffer e conserva la griglia
nativa per campionare i chunk. La copia necessaria alle richieste sincrone di
quota viene creata nel worker e trasferita all'interfaccia. `hgtReady` conferma
che la griglia è registrata prima dell'invio di `buildChunk`.

`TerrainManager.js` condivide una promessa per tile tra importazione, disco,
download e richieste asincrone di quota. Non usa più FileReader né un ciclo di
conversione completa nel percorso UI normale. Dimensioni non valide vengono
rifiutate. Un errore del worker libera le richieste pendenti e attiva un fallback
che cede il controllo ogni 65.536 campioni; una richiesta senza risposta ha un
timeout di 60 secondi. L'importazione annuncia il completamento dopo la conversione.

Questa fase aggiunge parallelismo tra UI e conversione, utilizzando il worker
del terreno già esistente. Le conversioni sono seriali all'interno di quel
worker: non è un pool che occupa tutti i core. Non introduciamo un worker per
ogni tile, che aumenterebbe picchi di memoria e competizione con i chunk.

Il modello delle quote resta Int16 e preserva valori negativi e void. Rimangono
una griglia nel renderer e una nel worker, oltre al File: un SRTM1 3601² costa
24,73 MiB per griglia. La cache resta per l'intera sessione; un limite alla RAM
richiede in seguito una LRU coordinata con i job di quota, chunk e missione.

### Confine iniziale del backend e risoluzione

`RenderBackend.js` possiede creazione, dimensionamento, passaggi e statistiche
di Three/WebGL2. `Scene3D.js` passa da questo componente per disegnare il frame.
Il renderer Three rimane esposto per compatibilità: scene, materiali, terreno
e sensori dipendono ancora da Three. Questo è il primo confine, non un'interfaccia
già capace di sostituire Three con Qt.

La sezione SYS offre **3D RESOLUTION** persistente:

| Profilo | DPR del canvas 3D | Destinazione |
| --- | --- | --- |
| Native | DPR del display | Massima risoluzione |
| Balanced, predefinito | massimo 1,5 | Uso normale |
| Eco | massimo 1 | Ridurre pixel e memoria dei render target |

Il DPR non viene aumentato sui display con scala inferiore al limite. Su un
display DPR 2, Balanced disegna il 56,25% dei pixel di Native ed Eco il 25%:
riduzione del numero di pixel, non percentuali garantite di aumento degli FPS.
HUD e DOM mantengono la risoluzione del display. Il LOD segue la nuova altezza
del drawing buffer; il diametro dei punti LiDAR segue il DPR del renderer.

Le statistiche vengono azzerate una volta per frame. Il disegno schematico
conta insieme terreno, outline e overlay, anziché riportare solo l'ultimo
passaggio. `getRenderPerformanceStats()` e DebugLog includono passaggi, draw
call, triangoli, risoluzione e p95 del tempo CPU di invio degli ultimi 120 frame.
Questo tempo non comprende tutta l'UI e non misura l'esecuzione GPU.

## Valutazione Qt/QML

Qt Quick mantiene un scene graph, effettua batching e può renderizzare su un
thread distinto dalla GUI; il backend passa attraverso le API grafiche native.
La scelta del render loop dipende da piattaforma e driver. Questi meccanismi
rendono plausibile un vantaggio quando il limite è la preparazione della scena
o il costo dell'interfaccia. [Documentazione Qt Quick](https://doc.qt.io/qt-6/qtquick-visualcanvas-scenegraph.html).

Per questa GCS, il candidato è una UI QML con Qt Quick 3D oppure un renderer
del terreno integrato nel backend nativo. Qt Quick 3D permette geometrie
personalizzate e la loro integrazione nella scena. [Qt Quick 3D](https://doc.qt.io/qt-6/qtquick3d-index.html),
[QQuick3DGeometry](https://doc.qt.io/qt-6/qquick3dgeometry.html).

Il JavaScript QML e la logica applicativa non diventano automaticamente multicore.
Conversione, mesh, parsing e calcoli missione richiedono worker C++ o servizi
separati. Una GPU limitata da pixel, texture o banda continuerà a esserlo anche
con un frontend nativo. L'eventuale vantaggio dipende quindi dal profilo reale.

Qt WebEngine integra Chromium: caricare l'HTML attuale al suo interno manterrebbe
gran parte dell'architettura browser. Perseguire il percorso nativo richiede la
riscrittura della UI, non soltanto cambiare il contenitore della finestra.
[Qt WebEngine](https://doc.qt.io/qt-6/qtwebengine-overview.html).

### Prototipo implementato e verificato

`prototypes/qt-terrain` contiene un client separato PySide6/Qt 6.9.3 con HGT
reale, normali, geometria indicizzata, camera animata e overlay QML. La prova
trasparente sul PC Ryzen AI 9 365/Radeon 880M ha usato **Direct3D 11**, disegnato
**64.800 triangoli** e presentato frame da un thread distinto dalla GUI.
JSON e screenshot temporanei della prova sono stati rimossi durante la pulizia.

La prima prova offscreen aveva selezionato software e non disegnava il terreno;
è stata sostituita dalla prova GPU con finestra trasparente. Python 3.9.1 locale
non caricava Shiboken: la prova riuscita usa Python 3.12.10 temporaneo isolato,
senza cambiare installazioni globali o dipendenze Electron.

Questo dimostra la fattibilità tecnica, non un guadagno prestazionale: la scena
è una singola mesh decimata, senza il carico completo della GCS. I tempi di
preparazione Python sono esclusi e non sono rappresentativi di un worker C++.
Il confronto con l'app richiede ancora satellite, chunk/LOD, outline, missione,
video e LiDAR equivalenti. Il prototipo è escluso dal pacchetto distribuito.

## Valutazione Three/WebGPU

WebGPURenderer offre WebGPU e compute, con fallback WebGL2. Gli ShaderMaterial
e gli hook `onBeforeCompile` attuali richiedono un porting a materiali node/TSL;
anche il post-processing cambia. Per questo non basta cambiare il costruttore
del renderer. La documentazione segnala che la convenienza dipende dalla scena
e che WebGLRenderer può ancora essere più veloce. [Guida ufficiale](https://threejs.org/manual/pages/webgpurenderer).

Il percorso WebGPU riutilizza più UI e logica applicativa di Qt. I primi porting
sono height texture, normali, colormap, terreno instanziato e outline da depth;
seguono linee spesse, simboli, acqua e sensori. Compute può essere utile per
grandi nuvole di punti o analisi massicce quando i dati restano sulla GPU.
Trasferire 550 controlli di visibilità sulla GPU con readback a ogni frame non
è il primo obiettivo.

## Fasi successive e criteri di scelta

| Fase | Lavoro concreto | Condizione di uscita |
| --- | --- | --- |
| 1, completata | HGT nel worker, risoluzione, misure dei passaggi, backend iniziale, prototipo Qt | Test funzionali superati; prove salvate |
| 2 | Dataset e replay identici; misurare CPU UI, submit CPU, tempo GPU, RAM, stalli e latenza input in finestre visibili | Baseline ripetibile, almeno 5 esecuzioni per scenario |
| 3, implementata per terreno/camera/Livox | Modelli neutri in uso, adapter GPU, pacchetto CRVG e consumer Qt | Contratto senza oggetti Three; test dei due consumer superati |
| 4 | Porting dimostrativo WebGPU e Qt del medesimo terreno con texture, outline, overlay e sensori | Stessa resa, camera, DPR, geometrie e funzionalità |
| 5 | Selezionare backend, poi portare le restanti funzioni e il packaging | Vantaggio stabile, regressioni funzionali assenti |

Scenari: schema e satellite con 400–550 chunk; replay con movimenti e cambio
tile; un milione e tre milioni di punti; missione lunga; FPV/AR; cambio scheda;
connessione telemetria durante la compilazione della missione. Distinguere
caricamento a freddo, regime e picchi. Disabilitare rete esterna usando gli stessi
asset locali, ma ripetere anche la prova di streaming controllato.

A parità di carico, adottare come soglia iniziale un miglioramento ripetibile
di almeno **25% del p95 dei frame oppure del costo CPU della UI**, senza peggiorare
latency input, fedeltà, RAM e stabilità. È un criterio progettuale proposto, non
un risultato misurato. Non confrontare una mesh Qt da 64.800 triangoli con la
scena GCS completa né gli FPS di una finestra nascosta con quelli di una visibile.

Se vince Qt: sostituire progressivamente HUD/pannelli/mappa/video con QML e
spostare i servizi attuali dietro un protocollo esplicito. Nel prototipo si può
riutilizzare un servizio Node separato; il rilascio finale deve definire ownership
di seriale, MAVLink, ROS, replay e IPC. Evitare uno scambio massivo di oggetti
JSON per le nuvole: usare messaggi binari con buffer e timestamp.

Se vince WebGPU: integrare i materiali TSL e il backend nella GCS, mantenendo
fallback e verifica su Windows/Linux/macOS. Finché non emergono questi risultati,
la versione corrente mantiene Three/WebGL2 con le ottimizzazioni già applicate.

## Verifiche della prima fase

`npm run test:hgt`: decoder completo SRTM3/SRTM1, quote negative, void, input
invalido, fallback con yield, chunk sul bordo e LOD subito dopo l'ack, thread
chiamante capace di eseguire timer durante la conversione.

`npm run test:render`: conteggi cumulativi dei tre passaggi, reset tra frame,
profili DPR e ridimensionamento. La prova Electron isolata ha inoltre verificato
deduplicazione import/quote, dimensioni reali dei canvas, rendering schematico,
fallback per worker assente e per errore durante una richiesta. Durante l'avvio:
quattro HGT pronti, 160 chunk prodotti, nessun `chunkFailed`, nessun fallback HGT.

Gli output temporanei della verifica sono stati rimossi. La finestra Electron
era nascosta: queste sono verifiche funzionali, non un benchmark FPS.

Passano anche `node scripts/test-lidar-math.js` e `node scripts/test-radio-link.js`.
Quest'ultimo usava percorsi assoluti negli import ESM e falliva su Windows prima
di eseguire i controlli: gli import ora usano `pathToFileURL`.
