# Arkitektur, första integrationssteget

```text
OAK camera ─┬─> vision ─┬─> FieldStateMachine ─> vald controller ─> leased motor boundary
OAK IMU ───┤            │                            │                    │
odometry ──┘            └─> diagnostics/latest-frame └─> verified stop <──┘
web UI ──> control lease ─> MANUAL/AUTO/Start/STOP
```

`FieldStateMachine` avgör endast state och tillåten controller. Den äger inte
CAN. I varje aktiv AUTO-state ska kamera, IMU, odometri och CAN övervakas med
monotona timeouter. Saknas en kritisk sensor går systemet till `FAULT`, och den
gemensamma motorgränsen skickar ett kvitterat stopp till båda motorerna.

`AUTO_SEARCH` är enbart tillåten när kameran levererar färska frames men
visuella mål saknas, en tillförlitlig `row_heading_reference` finns och den
begränsade söksträckan inte passerats. Kameraavbrott är alltså aldrig SEARCH.

`AUTO_START_DELAY` är ett extra explicit säkerhetsstate för den specificerade
startfördröjningen. STOP och sensorövervakning fortsätter att köras i detta
state.

Den nuvarande `DisabledMotorBoundary` är den enda motorgräns som kan skapas.
Den tar emot STOP för diagnostik men vägrar varje körkommando. Den kommande
V3.8-adaptern måste återanvända `get_heading/motor_transport.py` och en
verifierad `remote_control`-lease före varje enskild motoröverföring.
