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
| `field_control/odometry.py` | 8:1 motor-/hjulgeometri, motor↔hjul-RPM och dödräkning |
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

Fysisk CAN-output är ett separat, explicit raised-wheel-testläge och använder
den verifierade implementationen från syskonprojektet `remote_control`; den
dupliceras inte i detta projekt. Installera den i samma miljö innan sådan
deployment ens kan konstrueras:

```bash
.venv/bin/python -m pip install -e ../remote_control
```

Den fysiska konfigurationen måste uttryckligen aktivera output, ange `can0`,
den installerade same-ID-profilen och en stabil `/dev/serial/by-id/...`-sökväg,
samt bekräfta genomfört STOP-test och att hjulen är upphissade. Standardstart
öppnar aldrig CAN.

### Separat första rörelsetest, upphissade hjul

Den avgränsade HIL-köraren provar exakt **ett** hjul per process och kan aldrig
starta AUTO. Den skickar ett enda positivt kommando på `+2 RPM` och förnyar
inte leasen; den oberoende watchdoggen ska då registrera lease-expiry och
avaktivera runtime-outputen inom 300 ms, följt av verifierad STOP+0x9C-settle
vid stängning. Kör aldrig båda kommandona samtidigt eller utan operatör vid
det oberoende nödstoppet.

Ersätt `<CANABLE-BY-ID>` med den enda exakta sökvägen under
`/dev/serial/by-id/`. Vänsterhjul testas först:

```bash
.venv/bin/python -m field_control.first_motion_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --side left \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

Högerhjul testas i en ny, separat process först efter att vänsterresultatet
har kontrollerats:

```bash
.venv/bin/python -m field_control.first_motion_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --side right \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

Ett godkänt resultat anger `fault: "CONTROL_LEASE_EXPIRED"` samt
`command_motor_rpm: [2.0, 0.0]` eller `[0.0, 2.0]`; vid standard 8:1 är
`expected_wheel_rpm` en åttondel. Raw A2-kommandot delas aldrig med 8 —
protokollet tar motor-side RPM. Annat resultat är underkänt och
ska utredas innan nästa motorprov.

### Observerad 10-RPM-körning, upphissade hjul

Efter att first-motion-provet har godkänts kan den separata observeringsköraren
användas för ett valt hjul. Den är fast låst till `+10 motor-RPM` på den valda
motorsidan,
`0 RPM` på det andra, nominellt 10,0 s och 100 ms lease-refresh. Den saknar
AUTO- och hastighets-/tidsinställningar. Kör den först efter uttrycklig
operatörsbekräftelse och med det oberoende nödstoppet tillgängligt.

```bash
.venv/bin/python -m field_control.observed_motion_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --side left \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

Ange `--side right` för en separat högerkörning. Resultatets tid avser endast
programmets observationsfönster; den är inte en uppmätt fysisk motor- eller
stopptid. CAN-bortfall och utebliven Pi-schemaläggning täcks fortfarande av
det oberoende nödstoppet, inte av denna HIL-rutin.

### Manuellt webb-kommando framåt, upphissade hjul

Den separata HIL-köraren startar en isolerad MANUAL-runtime med fejkade tomma
källor (ingen OAK, AUTO eller navigering), armerar den verifierade gränsen och
skickar exakt **en** riktig HTTP-POST till befintliga `/api/manual/forward`.
Kommandot är fast låst till `+10 motor-RPM` på båda motorerna, dvs. `1,25
wheel-RPM` per sida med standardutväxlingen 8:1. Den förnyar sedan inte
leasen: godkänt resultat kräver `lease_fault: "CONTROL_LEASE_EXPIRED"`,
disarmering och den verifierade begränsade STOP+0x9C-stängningen. Det finns
inga riktning-, hastighets- eller tidsflaggor.

Kör först efter uttryckligt operatörsbeslut, med hjulen upphissade och det
oberoende nödstoppet tillgängligt:

```bash
.venv/bin/python -m field_control.manual_web_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

Ett annat resultat är underkänt. Kör då inga andra manuella riktningar innan
orsaken har utretts.

### Webb-STOP under aktiv manuell körning, upphissade hjul

Denna separata fasta HIL-rutin håller först det befintliga
`/api/manual/forward`-kommandot aktivt i tre sekunder (`+10 motor-RPM` per
sida, `+1,25 wheel-RPM` vid 8:1) och skickar sedan exakt en riktig
HTTP-POST till befintliga `/api/stop`. Godkänt resultat kräver att det aktiva
kommandot var accepterat, att STOP-routen svarar utan fel samt att utgången är
disarmerad utan runtime-fel innan den verifierade STOP+0x9C-stängningen.
Hastighet, riktning och tid kan inte ändras via CLI.

Kör endast efter ett nytt uttryckligt operatörsbeslut, med hjulen upphissade
och oberoende nödstopp tillgängligt:

```bash
.venv/bin/python -m field_control.manual_web_stop_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

### Phase A: automatisk in-row-turn, upphissade hjul

Phase A testar den normala produktionsvägen `AUTO_IN_ROW_TURN` med riktig
OAK-D SR/BNO086, delad fysisk CAN-odometri och `FieldControlApplication`.
Den kör den verifierade A4-modellen: färsk `0x92`-baslinje, ett begränsat A4-
positionmål per motor och worker-bekräftelse av båda målen med `0x92`.
Hjulen är upphissade, så detta är fortfarande inte en 180°-kalibrering eller
headingkontroll. Ett godkänt resultat är däremot ett normalt
`turn_completed` efter målbekräftelse, korrekt motriktad per-hjulsodometri
inom konfigurerad turn-tolerans, följt av HIL-rutinens explicita MANUAL/STOP
och disarmerad utgång.

Profilen är tillfälligt fast för just denna Phase-A-HIL: en gul markör med HSV
`26..36, 20..255, 150..255`, normal
tre-frames markerdebounce, fast turn-hastighet `40 motor-RPM`, monotont
begränsad A4-måldeadline på 34 sekunder. Den härleds från det största
720°-hjulmålet: `720 / 360 * 8 / 40 * 60 = 24 s`, plus en fast 10-sekunders
marginal för bounded CAN/positionsettle, samt noll
basfart och noll visuell korrigering före vändningen. Operatören placerar
markören i kamerans bild innan testet; under den begränsade väntan förblir
motorutgången disarmerad och inga icke-noll motorkommandon ges. Det finns inga CLI-flaggor för
hastighet, riktning eller tider.

De fasta gränserna är en snäv, empirisk OAK-kalibrering av den aktuella gula
testytan i nedre mittområdet: `H=26..36`, `S >= 20`, `V >= 150`. Den största
sammanhängande kandidaten var `587 px` och den näst största `7 px`; därför
behålls minsta area `100 px`. Detta är en tillfällig HIL-profil, inte en
produktionsprofil eller turn-kalibrering. Slutlig vändmarkör ska anges i
konfigurationsfilen (`marker_hsv_low`, `marker_hsv_high`, `marker_min_area`)
med värden uppmätta i `hsv_filter`.
Markören söks endast i den fasta nedre mittzonen `x=0,2..0,8`, `y=0,3..1,0`,
i linje med navigationsmålets framåtriktade sökområde; en i övrigt giltig gul
blob utanför zonen kan inte arma Phase A.

Kör endast efter ett nytt uttryckligt operatörsbeslut, med hjulen upphissade
och nödstoppet tillgängligt:

```bash
.venv/bin/python -m field_control.turn_phase_a_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised \
  --confirm-turn-not-calibrated
```

Resultatet registrerar den oförändrade turn-planen, per-hjuls
encoderförändringar och normala runtime-event. Fel, felaktigt encodertecken,
målavvikelse utanför `turn_distance_tolerance_m`, uteblivet
`turn_completed` eller en fortfarande armerad utgång efter slutlig STOP är
underkänt. Gör ingen kalibrering av `in_row_turn_wheel_degrees` från Phase A.

De äldre entrypointsen `turn_phase_a_long_hil` och
`turn_phase_a_visible_hil` finns kvar endast för kompatibilitet och delegerar
exakt till denna A4-rutin. De kan inte längre utföra en separat A2-
hastighets-/timeoutkörning.

### Manuellt webb-kommando back, upphissade hjul

Backtestet är en separat fast HIL-rutin. Den använder endast den befintliga
`/api/manual/reverse`-routen: båda motorerna kommenderas `-10 motor-RPM` (vid
8:1 alltså `-1,25 wheel-RPM`) och samma route förnyas var 100 ms under ett
nominalt femsekunders observationsfönster. En oberoende deadline äger sedan
explicit `STOP` även om en sista HTTP-begäran blockerar; en sen route kan då
inte återkommandera den disarmerade utgången. Rutinen kräver disarmerad utgång
utan runtime-fel och stänger den verifierade gränsen. Hastighet, riktning och
tid kan inte ändras via CLI.

Den dedikerade fasta entrypointen använder samma tre fysiska säkerhetsflaggor
som framåttestet. Kör den endast efter ett nytt uttryckligt operatörsbeslut,
med hjulen upphissade och oberoende nödstopp tillgängligt:

```bash
.venv/bin/python -m field_control.manual_web_reverse_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

### Manuellt webb-kommando vänster, upphissade hjul

Vänstertestet är en separat fast HIL-rutin som endast anropar befintliga
`/api/manual/left`. Den skickar det logiska fordonskommandot `(-10, +10)`
motor-RPM: vänster motor bakåt och höger motor framåt före de redan
verifierade, konfigurerade fysiska motorriktningarna. Vid 8:1 är detta
`(-1,25, +1,25)` wheel-RPM. Routen förnyas var 100 ms i ett nominellt
femsekundersfönster; en oberoende deadline skickar sedan explicit STOP även
om en sista HTTP-begäran blockerar. Hastighet, riktning och tid är inte
CLI-parametrar.

Kör den endast efter ett nytt uttryckligt operatörsbeslut, med hjulen
upphissade och oberoende nödstopp tillgängligt:

```bash
.venv/bin/python -m field_control.manual_web_left_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

### Manuellt webb-kommando höger, upphissade hjul

Högertestet är en separat fast HIL-rutin som endast anropar befintliga
`/api/manual/right`. Den skickar det logiska fordonskommandot `(+10, -10)`
motor-RPM: vänster motor framåt och höger motor bakåt före de redan
verifierade, konfigurerade fysiska motorriktningarna. Vid 8:1 är detta
`(+1,25, -1,25)` wheel-RPM. Routen förnyas var 100 ms i ett nominellt
femsekundersfönster; en oberoende deadline skickar sedan explicit STOP även
om en sista HTTP-begäran blockerar. Hastighet, riktning och tid är inte
CLI-parametrar.

Kör den endast efter ett nytt uttryckligt operatörsbeslut, med hjulen
upphissade och oberoende nödstopp tillgängligt:

```bash
.venv/bin/python -m field_control.manual_web_right_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

### Stoppad encoderförkontroll, upphissade hjul

Den observationsbaserade HIL-rutinen öppnar den redan verifierade CAN-workern,
vars befintliga preflight gör STOP och `0x9C`-settle. Den armerar aldrig
motorutgången, skapar aldrig en körlease och skickar aldrig `A2`. Därefter läser
den exakt fem atomära `0x92`-par från samma worker med 10 Hz planering. Den
avvisar ogiltiga värden, icke-monotona tidsstämplar och en vinkeländring större
än den fasta stillaståendegränsen `0,10` motorgrader (tio gånger den
dokumenterade rapporteringsgranulariteten `0,01` grader; detta är inte ett
precisionspåstående). Stängningen använder återigen verifierad STOP +
`0x9C`-settle.

Kör endast med upphissade hjul, bekräftat oberoende STOP och en explicit
operatörsbekräftelse:

```bash
.venv/bin/python -m field_control.encoder_preflight_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --enable-can \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

Resultatet är begränsat JSON med råa motorvinklar, differenser från första
provet och uppmätta provintervall. Ett felresultat är underkänt; kör då inga
sväng- eller odometritester innan CAN/encoderfelet har utretts.

### Encoder-/odometrirörelse, upphissade hjul

Efter godkänd stillastående encoderförkontroll kan varje sida provas i en ny,
separat process. Rutinen är låst till den valda sidan på `+2` motor-RPM under
ett nominellt ensekunds-fönster med 100 ms lease-förnyelse. Den kräver en
färsk typad encoder-/odometriuppdatering före armering, skickar explicit STOP
före slutavläsningen och stänger sedan via den verifierade STOP+0x9C-settlen.
Den har inga flaggor för hastighet, riktning eller tid.

Godkänt resultat kräver minst 1 mm absolut odometriförändring på den
kommenderade sidan och högst 1 mm på den andra sidan. Detta är en isolerings-
och kopplingskontroll, inte en kalibrering eller ett påstående om riktning.
Kör vänster och höger först efter varsin ny uttrycklig operatörsbekräftelse,
med hjulen upphissade och oberoende STOP tillgängligt:

```bash
.venv/bin/python -m field_control.encoder_motion_hil \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --side left \
  --enable-motors \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised
```

Byt endast `--side left` till `--side right` för den separata högerrutinen.
Ett felresultat är underkänt; fortsätt då inte till sväng-HIL innan orsaken är
utredd.

### Automatisk 180°-vändning: säker förkontroll och kvarvarande blockerare

En automatisk vändning får inte verifieras genom att mata in en påhittad
heading, simulera encoderdata eller mutera state machine direkt i en fysisk
HIL-process. Den verkliga `AUTO_IN_ROW_TURN`-vägen kräver samtidigt färsk
OAK-D/BNO086-heading och färsk delad per-hjulsodometri under hela manövern.
Denna ingång gör därför endast en icke-aktuerande förkontroll av den fasta
första profilen: ärvd `in_row_turn_wheel_degrees = 720`, konfigurerad riktning
och turn-geometri. Den öppnar inte CAN, startar inte OAK, armerar inte motorer
och gör inget påstående om 180°-kalibrering.

```bash
.venv/bin/python -m field_control.turn_hil_preflight \
  --slcan-device /dev/serial/by-id/<CANABLE-BY-ID> \
  --confirm-physical-stop-tested \
  --confirm-wheels-raised \
  --confirm-turn-not-calibrated
```

Den integrerade Phase-A-runtime-HIL:en ovan verifierar redan den normala
`AUTO_IN_ROW_TURN`-triggern, A4-målbekräftelse, hjultecken/encoder och
explicit STOP/disarm efter ett lyckat mål. En verklig 180°-kalibreringskontroll
kräver däremot fortsatt en rigg eller markkontakt som fysiskt vrider
chassit/OAK-enheten, samt verifiering av heading-, encoder- och
CAN-felbeteenden. Värdet `720` är endast en ärvd startpunkt och får inte kallas
kalibrerat förrän den uppmätta 180°-manövern har godkänts.

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

- aktuell OAK-D device ID `194430107168615A00`;
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

`field_control.config_io` läser och skriver strikt JSON med enbart
standardbiblioteket. Okända nycklar, `NaN`/`Infinity`, booleska värden där ett
tal krävs och ogiltiga nested-värden avvisas innan den vanliga
`RuntimeConfig.validate()` körs. Skriv en full standardfil med
`dump_runtime_config(RuntimeConfig(), "field_control.json")` och läs den med
`load_runtime_config("field_control.json")`. Alla RPM-fält, även
`manual_rpm`, är motor-side RPM före växellådan; `manual_rpm` aktiverar aldrig
motorer på egen hand. `imu_sample_hz` är ett positivt heltal (standard `100`)
och `log_level` är en av `DEBUG`, `INFO`, `WARNING`, `ERROR`, `CRITICAL`.

Ett minimalt säkert JSON-dokument kan lämna fysisk output avstängd:

```json
{
  "imu_sample_hz": 100,
  "manual_rpm": 0.0,
  "log_level": "INFO",
  "physical_can": {"enabled": false}
}
```

Övriga värden får då sina säkra dataclass-standardvärden. Att sätta
`physical_can.enabled` till `true` kräver fortfarande alla separata
raised-wheel-bekräftelser, `can0`, same-ID-profil och stabil by-id-sökväg.

### Säker config-, diagnostics- och fysisk webb-CLI

Installationen ger `field-control`. Den skriver och validerar enbart strikt
JSON-konfiguration och startar vid normal körning diagnostics/runtime från en
sådan fil:

```bash
field-control --write-default-config field_control.json
field-control --validate-config field_control.json
field-control --config field_control.json --host 127.0.0.1 --port 8080
```

Skrivning vägrar ersätta en befintlig fil utan `--force`. Normal CLI-körning
avvisar alltid `physical_can.enabled`; den kan alltså inte öppna eller armera
fysisk CAN. SIGINT/SIGTERM stänger applikationen kontrollerat.

För fälttest via webben finns en separat och uttrycklig fysisk startväg. Den
kräver en redan fullständigt validerad `physical_can`-konfiguration (inklusive
marktest- eller upphissad-hjul-gates), bindning till en numerisk loopback-IP
och två separata CLI-bekräftelser. Den startar alltid disarmerad; webbläsaren
kan aldrig armera motorerna. Om lokal armering avsiktligt önskas görs den först
efter att runtime, watchdog och färsk 0x92-odometri är igång och fortfarande i
`MANUAL`. Den byts sedan omedelbart till en konfigurerbar (högst 60 s), helt
stillastående webb-standby utan aktiv drive-lease. Första giltiga webbaserade
MANUAL-kommando eller Start Auto tar atomärt en ny vanlig 300 ms-lease. AUTO-
valet ensamt kan inte förnya eller ta över den. Timeout, STOP, MANUAL-val,
fel, CAN-/odometrifel, watchdog och stängning disarmerar standby:

```bash
field-control --config field_control_physical.json \
  --physical-web --confirm-physical-web \
  --host 127.0.0.1 --port 8080 \
  --arm-motor-output
```

Utelämna `--arm-motor-output` för en fysisk CAN-diagnostikstart som förblir
disarmerad. `--physical-web` avvisar andra adresser än loopback-IP; publicera
inte dashboarden över nätverket innan en separat autentiserad design finns.
STOP, kontroll-lease, watchdog och runtime-fel disarmerar fortsatt output.
De separata raised-wheel-HIL-körarna används fortfarande för avgränsade
motorprover.

Den lokala testprofilen `field_control_web_test.json` använder `manual_rpm`
`30` och `max_rpm` `40` (motor-RPM före 8:1-utväxlingen). En hållen
riktningsknapp skickar den konfigurerade hastigheten. När knappen släpps
skickar dashboarden därefter ett serverfixerat `0 RPM`-kommando var 100 ms och
behåller den korta control-leasen, så ett nytt riktningskommando kan provas
utan omstart. Den röda STOP-knappen, fokus-/sidförlust, nätfel eller uteblivna
uppdateringar använder däremot hård STOP och disarmerar utgången. I den
fysiska webbstandbyn är MANUAL redan valt; tryck inte MANUAL-knappen, utan
håll direkt inne en riktningsknapp.

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

### Automatisk vändning: A4-positionering redo för HIL

Den tidigare synliga Phase-A-körningen använde kontinuerliga `0xA2`-
hastighetskommandon och är inte längre rätt HIL-väg. Automatisk in-row- och
new-row-turn använder nu den tidigare verifierade modellen från
`oak_d_sr_navigation`: en färsk `0x92`-baslinje följs av `0xA4` absoluta
motorlägen. Hjulomkretsarna är `0,805 m`, spårvidden `1,005 m` och 8:1-
utväxlingen samt motorernas fysiska tecken appliceras exakt en gång i
CAN-workern. New-row skalar varje hjuls A4-hastighet efter dess vinkelmål.

CAN-workern håller minst 1,5 ms mellan motorernas A4-kommandon, bekräftar mål
med begränsade 0x92-avläsningar och STOP kan avbryta en aktiv positionering.
Målbekräftelse uppdaterar därefter rad-heading med +180° utan att vänta på
IMU, så den redan verifierade heading-/visionsregleringen får överta.

Automatiserade mockade tester täcker A4-mål, hastighetsförhållande,
workertrådens 1,5 ms-lucka, STOP-preemption, timeout och båda runtime-
turnvägarna. Det återstår en ny uttryckligt godkänd raised-wheel HIL-körning
för att bekräfta faktisk fysisk riktning, positionering och fail-closed STOP
innan någon 180°-kalibrering eller marktest görs.

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
