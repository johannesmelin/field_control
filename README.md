# field_control

Fail-closed integrationsmjukvara för autonom radnavigation av en
saffransskörderobot. Projektet kombinerar OAK-D SR-kamera, BNO086-IMU,
HSV-baserad vision, headingfiltrering, odometri, state machine, diagnostics
och ett framtida säkert motorlager.

## Status

Följande är implementerat och verifierat utan anslutna motorer eller CAN:

- OAK-D SR CAM_B och BNO086 körs i samma DepthAI-pipeline.
- Kamera och IMU exponeras som oberoende bounded latest-value-källor.
- Alla sensoråldrar använder monotonic tid.
- Kamera, IMU och odometri kan ersättas med fakes i tester.
- HSV-vision bevarar buds, leaves, turn-marker, zoner, target och maskdata.
- Heading filtreras cirkulärt och använder OAK-D:s fabrikskalibrering.
- En sammanhängande observation matas till den explicita state machine:n.
- Diagnostics-API, dashboard och senaste-bildströmmar finns.
- Encoderbaserad odometri och testbar 180-graders turn-geometri finns.
- Control lease/watchdog finns som hårdvaruoberoende komponent.

Fysisk motoroutput är avsiktligt avstängd. Standardgränsen är
`DisabledMotorBoundary`, och applikationen öppnar inte CAN.

## Arkitektur

```text
OAK-D SR
  +-- CAM_B frame queue -- latest camera value -- vision --+
  +-- BNO086 IMU queue --- latest IMU value ---- heading -+--> observation
  +-- encoder backend ---- latest distance ------ odometry-+       |
																  v
															state machine
																  |
												   disabled motor boundary
																  |
															  diagnostics
```

Viktiga principer:

- Sensorer producerar senaste värdet, aldrig en växande backlog.
- Kontroll- och timeoutlogik använder `time.monotonic()`.
- Stale eller ogiltiga kritiska sensorer leder till `FAULT` och stopp.
- Webbläsare och livestreamar får inte blockera sensor- eller kontrollloopar.
- Diagnostics kan inte kringgå motorgränsen.

## Kodstruktur

| Fil | Ansvar |
| --- | --- |
| `field_control/app.py` | Top-level lifecycle för runtime och webserver |
| `field_control/sources.py` | OAK-D-, IMU-, encoder- och latest-value-källor |
| `field_control/observation.py` | Headingprocessor och samlad observation |
| `field_control/vision.py` | HSV-detektion, zoner, target och maskbilder |
| `field_control/heading.py` | Cirkulär filtrering och row-heading-reference |
| `field_control/odometry.py` | 8:1 motor-/hjulgeometri och dödräkning |
| `field_control/turn.py` | Ren turn-geometri utan hårdvaruåtkomst |
| `field_control/state_machine.py` | Explicit navigation och fail-closed states |
| `field_control/control.py` | Begränsade vision- och headingkommandon |
| `field_control/motor_boundary.py` | Disabled/lease-gated fysisk outputgräns |
| `field_control/lease.py` | Monoton control lease och watchdog |
| `field_control/diagnostics.py` | JSON-safe statusmodell |
| `field_control/web.py` | Dashboard, status-API och MJPEG streams |

## Installation

Python 3.11 eller senare krävs. Använd en virtuell miljö:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e .
.venv/bin/python -m pip install 'depthai==3.8.0'
```

`depthai` är separat från projektets grundberoenden så att tester och
diagnostics kan köras utan OAK-D-hårdvara. OpenCV och NumPy installeras från
`pyproject.toml`.

## Starta diagnostics

Det finns ännu ingen färdig CLI-entrypoint. Starta applikationen från Python:

```python
from field_control.app import FieldControlApplication
from field_control.config import RuntimeConfig

application = FieldControlApplication(RuntimeConfig(), web_port=8080)
try:
	application.start()
	# Håll processen levande i den omgivande applikationen.
finally:
	application.close()
```

Med standardkonfigurationen används `320x240` för både processering och
livestream. Utan encoderbackend rapporterar runtime saknad odometri och
förblir fail-closed vid aktiv AUTO-körning.

## Webgränssnitt

När webservern körs finns dashboarden på `http://127.0.0.1:8080/`.

API och streams:

- `GET /api/status` - runtime, state, sensoråldrar, heading, vision,
  odometri, lease och motorstatus.
- `POST /api/manual` - välj MANUAL och stoppa.
- `POST /api/auto` - välj AUTO utan att starta körning.
- `POST /api/start-auto` - begär AUTO-start om färska data och target finns.
- `POST /api/stop` - omedelbart STOP.
- `GET /stream/raw` - originalbild.
- `GET /stream/overlay` - bild med zoner och target-overlay.
- `GET /stream/buds` - buds-mask.
- `GET /stream/leaves` - leaves-mask.
- `GET /stream/marker` - turn-marker-mask.

Dashboarden är ett diagnostics- och operatörsgränssnitt. Den armerar inte
motorer och kan inte kringgå lease eller motorboundary.

## State machine

Viktiga states är:

- `MANUAL`
- `AUTO_START_DELAY`
- `AUTO_ROW_FOLLOW`
- `AUTO_PICK`
- `AUTO_POST_PICK`
- `AUTO_SEARCH`
- `AUTO_IN_ROW_TURN`
- `AUTO_NEW_ROW_TURN`
- `AUTO_COMPLETE`
- `FAULT`

Kameraavbrott är sensorfel, inte `AUTO_SEARCH`. SEARCH är endast tillåten när
kamera, IMU, odometri och headingreferens är giltiga. Blind körning är
begränsad av `search_length_m`.

## Sensorer och kalibrering

`DepthAICombinedBackend` använder en pipeline med två bounded output queues.
CAM_B ger BGR-frames och BNO086 ger `ROTATION_VECTOR`-data. IMU-heading använder
CAM_B:s fabrikskalibrerade `getImuToCameraExtrinsics`.

Den fysiska OAK-D/BNO086-kommunikationen har verifierats med:

- OAK-D device ID `1944301061E6065B00`;
- IMU `BNO086`;
- IMU-firmware `3.9.9`;
- faktisk kamera- och IMU-data samtidigt;
- färska diagnostics-timestamps.

## Konfiguration

Konfigurationstyperna finns i `field_control/config.py`. Viktiga värden är:

- `processing_width` / `processing_height`: standard `320x240`;
- `stream_width` / `stream_height`: standard `320x240`;
- `camera_timeout_s`, `imu_timeout_s`, `odometry_timeout_s`;
- `heading_filter_alpha`;
- `row_heading_window_m` och `heading_reference_min_distance_m`;
- `row_spacing_m` och `odometry_geometry`;
- visionens HSV-filter och normaliserade zoner;
- navigationens P-regulatorer och hastighetsgränser.

Alla zoner använder normaliserade koordinater mellan `0.0` och `1.0`.

## Testning

Kör hela sviten:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Testerna täcker state transitions, freshness, monotonic time, latest-value,
vision, heading, odometri, turn-geometri, control lease, web actions och
simulerad end-to-end-runtime. Testerna kräver inte CAN eller anslutna motorer.

## Hårdvarusäkerhet och återstående arbete

CAN och motorer ska inte anslutas för normal testkörning. Följande kräver
separat säkerhetsgranskning och explicit hardware-testläge:

- fysisk encoderläsning över CAN;
- integration av control lease/watchdog med fysisk motorboundary;
- verifierad MyActuator-output under kontrollerade förhållanden;
- fysisk validering av motorernas riktning, RPM och turn-manövrer;
- fälttest av odometri, radreferens och sensorbortfall.

Ingen fysisk output får aktiveras som bieffekt av att starta kamera, IMU,
dashboard eller diagnostics.

## Relaterad dokumentation

- [Kravspecifikation](docs/requirements.md)
- [Arkitektur](docs/architecture.md)
- [Återanvändningsinventering](docs/reuse_inventory.md)
- [Agentinstruktioner](agent.md)
