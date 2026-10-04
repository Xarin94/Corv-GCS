# Prototipo Qt/QML del terreno

Client separato che carica un HGT reale, crea una geometria indicizzata con
normali, la disegna in Qt Quick 3D e sovrappone elementi QML. Serve a verificare
il backend nativo e il thread di rendering prima di portare la GCS.

Non sostituisce l'app Electron. Manca ancora la parità per chunk/LOD,
satellite, isolinee, missione, LiDAR, video e servizi del veicolo. Il preprocessing
Python avviene prima dell'event loop ed è escluso dalle misure del rendering.
In produzione dovrebbe essere in C++ su un worker, con buffer persistenti.

## Avvio

Usare Python 3.12 e un ambiente dedicato; il Python 3.9.1 presente sul PC della
prova non riusciva a caricare le DLL Shiboken. Nessuna dipendenza è aggiunta
all'app Electron.

```powershell
py -3.12 -m venv prototypes/qt-terrain/.venv
prototypes/qt-terrain/.venv/Scripts/python.exe -m pip install -r prototypes/qt-terrain/requirements.txt
prototypes/qt-terrain/.venv/Scripts/python.exe prototypes/qt-terrain/run.py --hgt topo/N45E010.hgt --step 20 --seconds 15 --output qt-result.json
```

`--step` deve dividere 1200 o 3600, in funzione della dimensione HGT. La prova
di quindici secondi chiude automaticamente la finestra e salva JSON e PNG.
Il JSON identifica API grafica, thread GUI e thread di `frameSwapped`, DPR,
buffer e intervalli dei frame dopo due secondi di riscaldamento.

Per una verifica senza finestra visibile, aggiungere `--smoke`: la finestra
GPU è trasparente, senza focus e senza voce nella barra delle applicazioni.
Gli intervalli non sono pubblicati come benchmark. L'esito richiede un'immagine
del terreno e un backend grafico diverso da software.

`--offscreen` serve soltanto a controllare il caricamento QML su macchine senza
display; può usare il renderer software e non disegnare la scena 3D.

## Esito sul PC della prova

Python 3.12.10 isolato + PySide6/Qt 6.9.3: HGT 3601², passo 20, 64.800 triangoli,
Direct3D 11, frame presentati da un thread diverso dalla GUI e screenshot del
terreno verificato. Gli artefatti temporanei della prova sono stati rimossi.
Il test dimostra la fattibilità del rendering nativo; per confrontare le
prestazioni servono scene equivalenti e finestre visibili su entrambi i client,
come definito in `docs/PIANO-MIGRAZIONE-GRAFICA.md`.

## Dati esportati dalla GCS

Il client supporta ora `--packet` in alternativa a `--hgt`:

```powershell
prototypes/qt-terrain/.venv/Scripts/python.exe prototypes/qt-terrain/run.py --packet "$env:USERPROFILE/Downloads/render-world.crvg" --seconds 3 --smoke --output qt-packet-result.json
```

Generare uno snapshot dalla GCS come descritto in `docs/ARCHITETTURA-RENDERING.md`.
CRVG v1 contiene heightfield dei chunk con il loro LOD, camera con proiezione
e quaternion, viewport e punti LiDAR con transform. Il prototipo usa direttamente
questi dati; non ricalcola un LOD indipendente dal client Electron. Il file reader
`render_packet.py` si può verificare senza Qt. Il rendering rimane uno snapshot
con materiali semplici: satellite, isolinee, UI e streaming non sono ancora portati.
Il formato e le responsabilità sono documentati in `docs/ARCHITETTURA-RENDERING.md`.

Verifica del consumer con Python e Node, senza dipendenze Qt. Lo snapshot di
prova viene generato dal produttore JavaScript in una cartella temporanea:

```powershell
python prototypes/qt-terrain/test_render_packet.py
```
