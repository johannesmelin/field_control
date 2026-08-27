# A4-marktest för `in_row_turn`

`field_control.turn_ground_hil` kör exakt en produktionsväg: markörtriggad
`AUTO_IN_ROW_TURN` med A4-vinkelmål. Den är avsedd för fri, plan markyta och
mäter – men kalibrerar inte – `in_row_turn_wheel_degrees`.

Grundprofilen är fast 20 motor-RPM. Den alternativa, också fasta, 40-RPM-
profilen kräver uttryckligen `--speed-profile 40`; godtycklig hastighet,
vinkel och varaktighet stöds inte.

Fysisk start kräver samtliga grindar: `--enable-motors`, tidigare bekräftat
STOP-test, `--confirm-ground-clear` och `--confirm-emergency-stop-ready`.
Den accepterar inte `--confirm-wheels-raised`. Rutinens resultat kräver färsk
markör, IMU och per-hjul-odometri före start, signed A4-encodermål och färska
IMU-bekräftelser inom konfigurerad headingtolerans kring startheading +180°.
Alla utfall går via publik STOP/disarmering före stängning.
