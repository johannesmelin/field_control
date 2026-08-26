# Återanvändningsinventering

| Funktion | Verifierad källa | Integrationsbeslut |
| --- | --- | --- |
| OAK-D SR 2D-kamera | `oak_d_sr_navigation/src/camera/oak.py` | Återanvänd CAM_B, senaste-ruta-kö och icke-blockerande capture. |
| HSV, blobar, `target_x`, zoner | `oak_d_sr_navigation/src/control/vision.py` | Återanvänd som bas; zoner konverteras till normaliserade koordinater i field-konfigurationen. |
| Vändgeometri | `oak_d_sr_navigation/src/navigation/state_machine.py` | Återanvänd `in_row_turn_wheel_degrees=720` som ärvd default (kalibrering pending); field bygger rena signed wheel-distance targets och motor-side command ratios utan CAN-skalning. |
| Heading och tiltkompensation | `get_heading/heading_service.py` | Den cirkulära lågpassfiltreringen återanvänds exakt; BNO086/DepthAI-adaptern är integrerad som latest-value-källa. |
| Heading-P-reglering | `get_heading/heading_navigation.py` | Återanvänd `shortest_angle_deg` och begränsade differentialprinciper; max-rpm begränsas även i motorgränsen. |
| V3.8 CAN, 0xA2/0x81/0x9C/0x92, tidsgränser | `remote_control/remote_control/physical.py` | Återanvänd som ensam SocketCAN-worker med verifierad same-ID-profil, bounded deadlines, STOP+0x9C-settle och atomära 0x92-par. |
| Odometri och 8:1-drivlina | `get_heading/odometry.py` | `motor_turns_per_wheel_turn` är enda 8:1-källan; CAN/`WheelCommand` använder motor-side RPM och wheel RPM härleds utan att ändra raw A2. |
| Manuell styrning, lease, watchdog, STOP | `remote_control` och `field_control.verified_motor_boundary` | Återanvänds via lease-gated adapter; den oberoende watchdogen återkallar output vid kontroll- eller encoderbortfall. |
| Webbsäkerhet/diagnostik | `remote_control/remote_control/server.py` | Återanvänd struktur; fältdiagnostik får en separat latest-frame-ström. |

Normal drift skapar aldrig fysisk output. Raised-wheel-HIL är explicit opt-in
och kräver stabil by-id-enhet, `can0`, fysiskt STOP-test och bekräftat
upphissade hjul. Stillastående 0x92-encoder-preflight är godkänd på den aktuella
hårdvaran; körande odometri- och turn-verifiering är separata, ännu ej
godkända HIL-steg.
