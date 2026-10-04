# Joystick, override RC e vista FPV

La configurazione si trova in **SETUP → Joystick / RC**. Collegare il controller,
premere un pulsante per renderlo visibile alla Gamepad API, quindi selezionarlo
nel menu GAMEPAD (SCAN aggiorna la lista).

## Otto ingressi RC

Sono disponibili almeno otto righe IN 1–8, anche sui gamepad con quattro assi
fisici. Per ogni riga scegliere la sorgente **Axis N** oppure **Button N** e il
canale di destinazione CH1–18. I pulsanti analogici, inclusi i grilletti esposti
come pulsanti, conservano i valori intermedi. Un pulsante digitale passa da
1000 a 2000 PWM; INV inverte il verso. Deadzone ed expo esistenti restano applicati.

Le prime quattro assegnazioni salvate vengono conservate. I nuovi ingressi
partono senza canale assegnato. Se una sorgente non esiste sul controller
selezionato, il relativo canale viene rilasciato. I controller che espongono più
di otto assi continuano a mostrarli.

Le barre mostrano l'anteprima anche a override spento. **ENABLE RC OVERRIDE**
attiva l'invio alla frequenza scelta; **SENDING** distingue l'invio dall'anteprima.
Le mappature restano salvate al riavvio; l'override deve essere riattivato.

## Due comandi servo

Per CMD 1 e CMD 2 scegliere un pulsante, un'uscita Servo 1–16 e una posizione
da 0 a 100%. La conversione è la stessa del test servo già presente nella GCS:
0% = 1000 µs, 50% = 1500 µs, 100% = 2000 µs.

**ENABLE SERVO BUTTONS** abilita questi comandi indipendentemente dall'RC
override. Ogni nuova pressione invia una sola richiesta `MAV_CMD_DO_SET_SERVO`;
tenere premuto o rilasciare non invia altre richieste. Ad esempio, assegnare
CMD 1 a Servo 9 / 0% e CMD 2 a Servo 9 / 100% permette di usare due pulsanti
per le due posizioni dello stesso servo. In alternativa, scegliere due uscite
diverse. L'uscita deve essere configurata sul veicolo per accettare comandi servo.

Le assegnazioni vengono salvate; l'abilitazione è limitata alla sessione.
Servono una connessione e la finestra in primo piano. I pulsanti già premuti
all'abilitazione, al cambio mappatura o al ritorno della connessione/focus devono
essere rilasciati e premuti nuovamente. Lo stato **Sent** indica l'invio della
richiesta, non la conferma di movimento del servo; gli errori di trasporto
compaiono nello stesso campo.

## Pulsanti per guardarsi attorno

Assegnare UP, DOWN, LEFT e RIGHT in **FPV LOOK BUTTONS**. La riga **Pressed
buttons** aiuta a identificare i numeri del controller.

Nella vista in prima persona di Flight Data, tenere premuto sposta lo sguardo
con gli stessi angoli e la stessa animazione delle frecce: 90° verticali e 110°
orizzontali. Sono supportate le diagonali; al rilascio la vista torna al centro.
Tastiera e gamepad mantengono stati indipendenti: rilasciare un pulsante non
annulla una freccia ancora premuta. Le assegnazioni non muovono la vista durante
la scrittura nei campi, nelle altre schede o con la finestra senza focus.
Funzionano anche in demo, senza connessione e con RC override spento.

## Verifiche

- `npm run test:joystick`: migrazione delle configurazioni, otto assi fisici e
  virtuali, PWM, rilascio dei 18 canali, perdita dati, riconnessione, due comandi
  servo e interazione FPV/tastiera. Controller e trasporto sono simulati.
- `npm run test:joystick-ui`: pannello reale in Electron con gamepad e IPC
  simulati; controlli, anteprima, invio servo, persistenza al ricaricamento,
  deselezione del controller e layout stretto. Non apre connessioni a veicoli.

La revisione corregge anche il bit di `RC_OPTIONS` usato per accettare gli
override (bit 1, valore 2) e il rilascio MAVLink dei canali 9–18 (65534 sul
protocollo). Riferimenti: [ArduPilot RC Options](https://ardupilot.org/copter/docs/common-rc-options.html)
e [MAVLink RC_CHANNELS_OVERRIDE](https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE).
