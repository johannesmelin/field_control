# A4-marktest för `new_row_turn`

`field_control.turn_ground_new_row_hil` kör exakt den ordinarie,
markörtriggade produktionsvägen `AUTO_NEW_ROW_TURN`: vänster ny-radsbåge med
1,20 m radavstånd som grundprofil. Den fasta, explicita profilen
`--row-spacing-profile 1.50` väljer 1,50 m C/C för ett marktest. Det finns
ingen godtycklig radavståndsparameter och produktionsstandardvärdet 1,20 m
ändras inte. Den verifierar asymmetriska, signerade A4-encodermål och
mäter den faktiska IMU-headingförändringen mot startheading +180°. Den
kalibrerar inte geometri eller vändvinkel.

Grundprofilen är fast 20 motor-RPM. `--speed-profile 30` och
`--speed-profile 40` väljer fasta profiler; godtycklig hastighet, vinkel och
varaktighet kan inte anges. `--direction left|right` väljer endast bågens
riktning; standard är `left`. Exempel för den avsedda högervändningen:

```
python -m field_control.turn_ground_new_row_hil ... \
  --row-spacing-profile 1.50 --speed-profile 30 --direction right
```

Resultatrapporten anger alltid valt radavstånd och hastighetsprofil.
Fysisk start kräver motoraktivering, tidigare STOP-test, fri markyta och
tillgängligt nödstopp. Hjul-upphissning accepteras inte. Markör, IMU och
per-hjulsodometri måste vara färska före armering. Efter målankomst krävs tre
nya IMU-prover inom produktionskonfigurationens headingtolerans; alla utfall
går via publik STOP/disarmering före stängning.
