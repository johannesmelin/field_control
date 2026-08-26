# Arkitektur, första integrationssteget

```text
OAK camera ─┬─> vision ─┬─> FieldStateMachine ─> vald controller ─> leased motor boundary
OAK IMU ───┤            │                            │                    │
odometry ──┘            └─> diagnostics/latest-frame └─> verified stop <──┘
web UI ──> control lease ─> MANUAL/AUTO/Start/STOP
```

`FieldStateMachine` avgör endast state och tillåten controller. Den äger inte
CAN. I varje aktiv AUTO-state övervakas kamera, IMU, odometri och CAN med
monotona timeouter. Saknas en kritisk sensor går systemet till `FAULT`, och den
gemensamma motorgränsen begär ett verifierat stopp av båda motorerna.

`AUTO_SEARCH` är enbart tillåten när kameran levererar färska frames men
visuella mål saknas, en tillförlitlig `row_heading_reference` finns och den
begränsade söksträckan inte passerats. Kameraavbrott är alltså aldrig SEARCH.

`AUTO_START_DELAY` är ett extra explicit säkerhetsstate för den specificerade
startfördröjningen. STOP och sensorövervakning fortsätter att köras i detta
state.

Standardapplikationen använder fortfarande `DisabledMotorBoundary` och öppnar
aldrig CAN. Det explicit konfigurerade raised-wheel-läget öppnar däremot
`VerifiedMotorBoundary`, som använder `remote_control.PhysicalCanMotors` som
ensam ägare av SocketCAN-arbetaren. Varje körkommando är lease-gated, STOP och
stängning preempterar köade kommandon, och stängning gör den verifierade
STOP+0x9C-settle-sekvensen.

Fysisk odometri delar samma CAN-worker genom en icke-ägande
encoderadapter. Den öppnar aldrig ett andra CAN-socket och kan inte stänga
motorgränsen. En fysisk armering väntar begränsat på ett färskt, typat
encoderprov; saknat, ogiltigt eller senare föråldrat odometrivärde ger
fail-closed STOP/fel även i MANUAL. De separata HIL-entrypointsen är den enda
vägen som får aktivera fysisk utgång.
