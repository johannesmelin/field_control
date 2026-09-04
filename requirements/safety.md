# Säkerhetsram för `field_control`

Detta dokument samlar redan etablerade säkerhetskrav från
`requirements/requirements.md` och `requirements/architecture.md`. Det skapar
inga nya motor-, protokoll- eller kalibreringskrav; den numrerade
kravspecifikationen är fortsatt auktoritativ.

## Fail-closed motorutgång

- Standardapplikationen använder `DisabledMotorBoundary` och öppnar inte CAN.
- Fysisk motorutgång får bara användas genom det explicit konfigurerade
  raised-wheel-läget och dess separata HIL-entrypoints.
- STOP, stängning, kontrollförlust eller fel ska begära ett verifierat stopp av
  båda motorerna.

## Sensorer, tid och styrning

- Kritiska kamera-, IMU-, CAN- eller odometrifel/stale data ska ge `FAULT` och
  stopp; en frisk kamera utan visuella mål är det avgränsade undantaget som kan
  tillåta `AUTO_SEARCH`.
- Sensorfärskhet, leases, watchdogar, fördröjningar och deadlines använder
  monotonic tid. Sensorer levererar senaste värdet, inte en obunden backlog.
- MANUAL/AUTO-byte stoppar båda motorerna först. STOP ska vara omedelbart och
  fungera oavsett state.

## Hårdvarugränser

Motor-ID, riktning, skalning, utväxling, CAN-protokoll och tidsgränser får inte
antas eller ändras utan den verifierade projektdokumentationen. Programvaru-
och mocktester är inte hårdvaruverifiering; fysisk körning kräver dokumenterade
HIL-förutsättningar, upphissade hjul när det anges och ett oberoende nödstopp.
