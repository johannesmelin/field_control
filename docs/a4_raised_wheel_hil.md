# Fasta upphissade A4-tester

De här testerna använder den vanliga produktionskedjan (markör, state-maskin,
workerägd 0x92/A4 och 0x92-målbekräftelse). De har inga flaggor för hastighet,
vinkel, tid eller riktning. Fysisk körning kräver alltid samtliga explicita
säkerhetsgrindar i kommandot.

## AUTO_NEW_ROW_TURN

Den fasta profilen är vänster ny-rad-båge, 40 motor-RPM före 8:1-utväxlingen,
1,20 m radavstånd och aktuell mätt hjulgeometri: 0,805 m vänster/höger
hjulomkrets samt 1,005 m hjulspår.

Det ger logiska hjulmål `+0,306305 m` vänster och `+3,463606 m` höger,
motsvarande cirka `+136,98°` respektive `+1548,94°` hjulvinkel. Största målet
tar 51,63 s nominellt vid 40 motor-RPM; den fasta A4-deadlinen är 61,63 s
(inklusive 10 s marginal). Hjulet måste vara upphissat under testet.

```bash
.venv/bin/python -m field_control.turn_new_row_hil \
  --slcan-device /dev/serial/by-id/USB_DEVICE \
  --enable-motors --confirm-physical-stop-tested \
  --confirm-wheels-raised --confirm-turn-not-calibrated
```

Godkänt resultat kräver `AUTO_NEW_ROW_TURN`, asymmetrisk och teckenkorrekt
encoderförflyttning inom produktionens tolerans, normalt terminalstate samt
publik MANUAL/STOP-disarmering.

## STOP under aktiv A4

Samma fasta profil används. Rutinens STOP skickas först när den verifierat att
den normala runtime-vägen har registrerat A4-positionering **och** att den
workerägda målstatusen är aktiv. Den statusen sätts först efter att båda
sekventiella A4-svaren accepterats. Därefter krävs MANUAL, disarmerad output
och ett fullbordat, begränsat verifierat STOP+0x9C-settle innan appen stängs.

```bash
.venv/bin/python -m field_control.turn_new_row_hil \
  --stop-during-active-a4 \
  --slcan-device /dev/serial/by-id/USB_DEVICE \
  --enable-motors --confirm-physical-stop-tested \
  --confirm-wheels-raised --confirm-turn-not-calibrated
```

Testet är endast en motor-/CAN-verifiering med upphissade hjul. Det kalibrerar
inte den faktiska chassivändningen på marken.
