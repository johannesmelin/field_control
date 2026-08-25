# Återanvändningsinventering

| Funktion | Verifierad källa | Integrationsbeslut |
| --- | --- | --- |
| OAK-D SR 2D-kamera | `oak_d_sr_navigation/src/camera/oak.py` | Återanvänd CAM_B, senaste-ruta-kö och icke-blockerande capture. |
| HSV, blobar, `target_x`, zoner | `oak_d_sr_navigation/src/control/vision.py` | Återanvänd som bas; zoner konverteras till normaliserade koordinater i field-konfigurationen. |
| Vändgeometri | `oak_d_sr_navigation/src/navigation/state_machine.py` | Återanvänd `new_row_turn_wheel_degrees`; parameternheter konverteras från cm till m i en adapter. |
| Heading och tiltkompensation | `get_heading/heading_service.py` | Den cirkulära lågpassfiltreringen återanvänds exakt; BNO086/DepthAI-adaptern integreras i nästa hårdvarusteg. |
| Heading-P-reglering | `get_heading/heading_navigation.py` | Återanvänd `shortest_angle_deg` och begränsade differentialprinciper; max-rpm begränsas även i motorgränsen. |
| V3.8 CAN, 0xA2/0x81, tidsgränser | `get_heading/motor_transport.py` | Återanvänd oförändrad med explicita deadline-värden och båda motorernas kvitterade stopp. |
| Odometri och 8:1-drivlina | `get_heading/odometry.py` | Återanvänder explicit 8:1-profil och riktningstecken; field-konfigurationen anger nu hjulomkrets per sida och hjulspår. |
| Manuell styrning, lease, watchdog, STOP | `remote_control/remote_control/controller.py` | Återanvänd den beprövade kontrollsessionen och fail-closed connection-loss-hanteringen. |
| Webbsäkerhet/diagnostik | `remote_control/remote_control/server.py` | Återanvänd struktur; fältdiagnostik får en separat latest-frame-ström. |

Ingen fysisk motorutgång implementeras i första steget. Den aktiveras först när
ovanstående motor- och leasekedja kan testas tillsammans med state-maskinen.
