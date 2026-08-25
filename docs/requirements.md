# Instruktioner för projektet `field_control`

## 1. Syfte

Detta är ett nytt projekt för radnavigation av en saffransskörderobot med differentialstyrning.

Robotens övergripande uppgift är att:

* köra rakt längs raka och parallella saffransrader,
* normalt navigera visuellt efter gröna blad och/eller violetta knoppar,
* stanna när skördbara knoppar befinner sig i ett definierat område framför roboten,
* efter skörd fortsätta längs raden,
* kunna fortsätta en begränsad sträcka med IMU-heading och dödräkning när det tillfälligt saknas blast och knoppar,
* identifiera vändmarkörer vid radslut,
* genomföra vändning och antingen köra tillbaka i samma rad eller gå vidare till nästa rad,
* kunna växla mellan automatisk navigering och manuell styrning från ett webbgränssnitt.

Själva skördefunktionen hanteras av annan mjukvara och ingår inte i `field_control`.

---

# 2. Befintliga verifierade projekt ska återanvändas

`field_control` är i första hand ett **integrationsprojekt**.

Funktioner som redan är implementerade och verifierade i följande projekt ska återanvändas med minsta möjliga förändring:

* `oak_d_sr_navigation`
* `get_heading`
* `remote_control`

Codex ska först undersöka dessa projekt och identifiera vilka befintliga moduler, klasser och funktioner som implementerar:

* kommunikation med OAK-D SR,
* bildinsamling,
* HSV-detektion,
* beräkning av navigationsmålens x-position,
* CAN-kommunikation,
* motorstyrning av MyActuator RMD X6,
* differentialstyrning,
* motorernas riktningar och skalningar,
* acceleration/deceleration,
* watchdog och säkerhetsstopp,
* manuell styrning,
* webbgränssnitt,
* IMU-avläsning,
* headingfiltrering,
* dödräkning,
* `in_row_turn`,
* `new_row_turn`.

Skriv inte nya parallella implementationer av redan verifierad funktionalitet om det inte finns ett tydligt tekniskt skäl.

Om en verifierad funktion behöver flyttas till en gemensam modul eller få ett nytt API för integration är detta tillåtet, men beteendet ska bevaras.

Vid konflikt mellan denna specifikation och äldre projekts **övergripande beteende** gäller denna specifikation.

Verifierade låg-nivåimplementationer såsom:

* CAN-protokoll,
* motor-ID,
* rotationsriktningar,
* skalningsfaktorer,
* IMU-hantering,
* headingfilter,
* dödräkning,
* säkerhetsmekanismer

ska däremot återanvändas och inte ersättas utan tydligt behov.

Dokumentation för MyActuator RMD X6 finns i:

`https://github.com/johannesmelin/Storp-documentation-and-recources`

Motorerna är MyActuator RMD X6 med 8:1 utväxling.

---

# 3. Sensorer och navigationsdata

Navigationen använder:

1. 2D-bild från OAK-D SR.
2. IMU i OAK-D SR.
3. Motor-/hjuldata för dödräkning.

Kameralänk:

`https://shop.luxonis.com/products/oak-d-sr`

Kamerabilder samplas med konfigurerbar frekvens.

Tre typer av strukturer identifieras med HSV-filter:

* saffransknoppar,
* grön blast,
* vändmarkörer.

HSV-parametrarna ska finnas i konfigurationen.

---

# 4. Viktig skillnad mellan förlorade mål och sensorfel

Skilj strikt mellan:

### A. Kameran fungerar men inga navigationsmål identifieras

Det finns giltiga nya frames, men inga giltiga knoppar eller blad identifieras.

Robotens automatiska navigation får då övergå till headingbaserad sökning enligt reglerna nedan.

### B. Kameradata saknas

Exempel:

* kameran slutar leverera frames,
* DepthAI-processen kraschar,
* frame timeout uppstår.

Detta är ett sensorfel.

Robotens motorer ska då stoppas och systemet ska gå till `FAULT`.

Motsvarande princip gäller för kritiskt bortfall av:

* IMU,
* CAN,
* erforderlig odometridata.

Robotens fortsättning med IMU-heading är alltså endast tillåten när kameran fungerar men visuella navigationsmål tillfälligt saknas.

---

# 5. Headingbegrepp

Separera tydligt följande begrepp:

### `filtered_heading`

Aktuell filtrerad heading från IMU:n.

Detta är ett **mätvärde**.

Återanvänd exakt den verifierade headingfiltreringen från `get_heading`.

### `row_heading_reference`

Uppskattad riktning på den aktuella raka saffransraden.

Detta är ett **börvärde** som används när roboten behöver köra utan visuella navigationsmål.

Robotens headingreglering ska då baseras på:

`heading_error = row_heading_reference - filtered_heading`

med korrekt hantering av vinkelomslag.

### `turn_target_heading`

Önskad heading under eller efter en vändmanöver.

---

# 6. Beräkning av `row_heading_reference`

`filtered_heading` ska fortsätta beräknas kontinuerligt så länge IMU:n fungerar.

`row_heading_reference` ska däremot endast uppdateras när roboten kör stabilt längs raden och det finns giltiga visuella navigationsmål.

`row_heading_reference` ska representera radens riktning, inte robotens momentana heading efter en enskild styrkorrigering.

Beräkna därför `row_heading_reference` från headinghistoriken över den senaste konfigurerbara körsträckan:

`row_heading_window_m`

Använd ett lämpligt cirkulärt medelvärde för headingdata.

För att undvika att `row_heading_reference` byggs upp från för få mätningar direkt efter start eller efter en vändning ska en ny headingreferens inte betraktas som fullt tillförlitlig förrän roboten har kört minst:

`heading_reference_min_distance_m`

med giltig och stabil visuell radföljning.

Under:

* `AUTO_IN_ROW_TURN`
* `AUTO_NEW_ROW_TURN`

ska `row_heading_reference` inte uppdateras.

IMU-mätning och `filtered_heading` ska däremot fortsätta normalt.

Efter en genomförd 180-graders vändning ska den nya radreferensen sättas till:

`previous_row_heading_reference + 180°`

normaliserat till korrekt vinkelintervall.

Eftersom saffransraderna är raka och parallella ska detta vara huvudprincipen för radriktningen efter en vändning.

---

# 7. Huvudstates

Systemet ska implementeras som en explicit state machine.

Minst följande states ska finnas:

* `MANUAL`
* `AUTO_ROW_FOLLOW`
* `AUTO_PICK`
* `AUTO_POST_PICK`
* `AUTO_SEARCH`
* `AUTO_IN_ROW_TURN`
* `AUTO_NEW_ROW_TURN`
* `AUTO_COMPLETE`
* `FAULT`

Undvik att implementera huvudlogiken som ett stort antal oberoende booleska flaggor.

State transitions ska vara tydligt definierade och möjliga att logga.

---

# 8. Manuell respektive automatisk styrning

Webbgränssnittet ska ha ett val mellan:

* `MANUAL`
* `AUTO`

Vid varje byte mellan MANUAL och AUTO ska båda motorerna först stoppas.

Byte från MANUAL till AUTO får inte automatiskt starta roboten.

Användaren ska aktivt starta automatisk körning.

När användaren väljer Start Auto ska systemet kunna använda en konfigurerbar fördröjning:

`auto_start_delay_s`

innan motorerna börjar gå.

Under denna fördröjning ska:

* roboten stå still,
* state och nedräkning visas tydligt i webbgränssnittet,
* STOP fortfarande fungera omedelbart.

Det ska dessutom alltid finnas en tydlig `STOP`-funktion i webbgränssnittet som stoppar båda motorerna oberoende av aktuellt state.

Manuell styrning ska i så stor utsträckning som möjligt återanvända den verifierade implementationen från `remote_control`.

---

# 9. Visuell radnavigation

Automatisk radnavigation startas endast där minst ett giltigt navigationsmål finns.

Konfigurationen ska tillåta två navigationslägen:

* `buds_only`
* `buds_and_leaves`

Vid `buds_only` används endast knoppar för radföljningen.

Vid `buds_and_leaves` används både knoppar och blast.

Navigationsdata hämtas från `navigation_zone`.

`navigation_zone` har samma huvudsakliga y-område som `pick_zone`, men ett smalare x-område.

Den visuella styrningen ska återanvända den verifierade principen från `oak_d_sr_navigation`.

Robotens differentialstyrning ska hålla det identifierade radmålet längs:

`x_goal`

i bilden.

Styrfelet är:

`x_error = target_x - x_goal`

och används för P-reglering av motorerna.

Följande ska vara konfigurerbart:

* `x_goal`
* `vision_kp`
* `vision_deadband_px`
* `max_vision_correction_rpm`
* `auto_base_rpm`

---

# 10. Filtrering av navigationsmålets x-position

Återanvänd i första hand den verifierade metoden från `oak_d_sr_navigation` för att beräkna navigationsmålets x-position.

Det resulterande x-värdet ska därefter kunna filtreras tidsmässigt över flera giltiga bildmätningar.

Konfigurationsparameter:

`x_filter_window_frames`

Förbered även arkitekturen för att vid behov kunna ignorera extrema avvikande x-värden.

Konfigurationsparameter:

`x_outlier_threshold_px`

Det är acceptabelt att initialt ha denna funktion avstängd.

---

# 11. Förlust och återfångning av visuella navigationsmål

En enda bild utan detektion ska inte automatiskt innebära att raden betraktas som förlorad.

Använd:

`navigation_lost_timeout_s`

Om inga giltiga navigationsmål har identifierats under denna tid och kameran fortfarande levererar giltiga frames:

`AUTO_ROW_FOLLOW -> AUTO_SEARCH`

När visuella navigationsmål återkommer ska återgång till visionstyrning kräva flera konsekutiva giltiga detektioner:

`navigation_reacquire_frames`

När detta krav uppfylls:

`AUTO_SEARCH -> AUTO_ROW_FOLLOW`

Syftet är att undvika snabb oscillering mellan `AUTO_ROW_FOLLOW` och `AUTO_SEARCH`.

---

# 12. `AUTO_SEARCH`

När visuella navigationsmål saknas men kamera, IMU och odometri fungerar ska roboten fortsätta längs:

`row_heading_reference`

Headingregleringen ska återanvända verifierade principer från `get_heading`.

Separata parametrar ska finnas för headingstyrningen:

* `heading_kp`
* `heading_deadband_deg`
* `max_heading_correction_rpm`
* `search_speed_rpm`

Sträckan mäts med dödräkning såsom i `get_heading`.

Maximal tillåten körsträcka utan visuella navigationsmål:

`search_length_m`

### Om navigationsmål återfinns innan `search_length_m`

Gå tillbaka till:

`AUTO_ROW_FOLLOW`

### Om en giltig vändmarkör hittas

Gå till aktuellt turn-state.

### Om `search_length_m` uppnås utan navigationsmål eller vändmarkör

Stoppa båda motorerna.

Gå till:

`FAULT`

med felorsak exempelvis:

`ROW_LOST`

Robotens sökkörning får aldrig fortsätta obegränsat.

---

# 13. Trigger för skörd

När en saffransknopp detekteras i:

`trigger_zone`

ska båda motorerna stanna.

Systemet går då från exempelvis:

`AUTO_ROW_FOLLOW -> AUTO_PICK`

eller, om relevant:

`AUTO_SEARCH -> AUTO_PICK`

Knoppar i `trigger_zone` får endast trigga stopp när triggerfunktionen är aktiverad.

---

# 14. `AUTO_PICK`

I `AUTO_PICK` ska roboten stå still.

Skörd utförs av annan mjukvara och ingår inte i detta projekt.

Robotens `field_control` ska stanna i `AUTO_PICK` tills något av följande inträffar:

1. `pick_zone` har varit utan knoppar under minst:

   `pick_clear_time_s`

2. maximal väntetid har uppnåtts:

   `max_pick_wait_s`

En enstaka frame utan detekterad knopp ska alltså inte avsluta `AUTO_PICK`.

Om `max_pick_wait_s` uppnås trots att knoppar fortfarande syns:

* lämna `AUTO_PICK`,
* logga händelsen som exempelvis `PICK_TIMEOUT`,
* fortsätt till `AUTO_POST_PICK`.

Timeout ska alltså inte automatiskt orsaka `FAULT`.

---

# 15. `AUTO_POST_PICK`

När skörden avslutas ska roboten gå till:

`AUTO_POST_PICK`

Syftet är att undvika att samma knopp eller samma skördeområde omedelbart triggar ett nytt stopp.

Under `AUTO_POST_PICK`:

* ignorera knoppar i `trigger_zone` för stopp,
* använd normal visuell radföljning om giltiga navigationsmål finns,
* använd `row_heading_reference` om visuella navigationsmål saknas,
* använd dödräkning för att mäta körd sträcka.

När roboten har kört minst:

`post_pick_lockout_distance_m`

ska triggerfunktionen återaktiveras.

Därefter:

* gå till `AUTO_ROW_FOLLOW` om visuella navigationsmål finns,
* annars fortsätt enligt `AUTO_SEARCH`.

---

# 16. Vändmarkörer

Vändmarkörer identifieras med ett eget HSV-filter.

Överväg att endast söka efter dem inom en definierad:

`turn_marker_zone`

så att falska markörer från irrelevanta delar av bilden kan undvikas.

För att en vändmarkör ska räknas som giltig ska den bekräftas under:

`turn_marker_confirm_frames`

konsekutiva frames.

Efter att en vändmarkör triggat en vändning ska nya vändmarkörer ignoreras tills roboten har kört minst:

`turn_marker_rearm_distance_m`

efter vändningen.

Detta förhindrar att samma markör triggar flera vändningar.

---

# 17. Prioritet mellan händelser

Vid samtidiga händelser ska prioriteten vara:

1. säkerhetsfel / `FAULT`
2. giltig vändmarkör
3. knopp i `trigger_zone`
4. normal visuell navigation
5. `AUTO_SEARCH`

En vändmarkör vid radslutet ska alltså inte ignoreras för att det samtidigt finns en knopp i `trigger_zone`.

---

# 18. `in_row_turn` och `new_row_turn`

Återanvänd de verifierade turn-principerna från `oak_d_sr_navigation`.

Konfigurationen ska innehålla:

* `in_row_turn_enabled`
* `new_row_turn_direction`
* `row_spacing_m`
* `number_of_rows`
* `turn_speed_rpm`

`new_row_turn_direction` ska kunna vara:

* `left`
* `right`

`row_spacing_m` anger CC-avståndet mellan två intilliggande saffransrader.

`turn_speed_rpm` anger grundhastigheten som ska användas under turn-manövrer, i den mån den befintliga verifierade implementationen tillåter att denna anges separat.

Robotens geometri ska dessutom innehålla:

`wheel_track_m`

vilket är avståndet mellan vänster och höger drivhjuls centrumlinjer.

`wheel_track_m` ska användas där differentialkinematik eller geometrin för en vändmanöver kräver hjulens inbördes avstånd.

---

# 19. Betydelsen av `in_row_turn_enabled`

Om:

`in_row_turn_enabled = false`

ska en vändmarkör vid radslutet utlösa `AUTO_NEW_ROW_TURN`.

Om:

`in_row_turn_enabled = true`

ska varje fysisk rad köras i båda riktningarna.

Sekvensen ska då vara:

1. kör första riktningen i rad N,
2. första vändmarkören utlöser `AUTO_IN_ROW_TURN`,
3. roboten gör cirka 180° vändning,
4. roboten kör tillbaka i samma fysiska rad,
5. nästa vändmarkör på samma rad utlöser `AUTO_NEW_ROW_TURN`,
6. roboten flyttar till nästa rad,
7. processen upprepas.

`number_of_rows` anger antalet **unika fysiska rader**, inte antalet radpassager.

---

# 20. Heading efter vändning

Under själva turn-manövern fryses uppdateringen av `row_heading_reference`.

Efter en lyckad 180-graders vändning ska:

`row_heading_reference = previous_row_heading_reference + 180°`

normaliserat till korrekt intervall.

Om inga knoppar eller blad omedelbart syns efter:

* `AUTO_IN_ROW_TURN`
* `AUTO_NEW_ROW_TURN`

ska roboten därför kunna fortsätta med headingbaserad navigation längs den nya `row_heading_reference`.

Denna körning följer samma begränsningar som övrig `AUTO_SEARCH`, inklusive maximal `search_length_m`.

---

# 21. Avslutat arbete

När:

`number_of_rows`

fysiska rader har bearbetats ska roboten:

* stoppa båda motorerna,
* gå till `AUTO_COMPLETE`,
* inte starta om automatiskt.

Webbgränssnittet ska tydligt visa att uppdraget är färdigt.

---

# 22. Dödräkning och robotgeometri

Återanvänd dödräkningen från `get_heading`.

Följande geometriparametrar ska vara konfigurerbara:

* `left_wheel_circumference_m`
* `right_wheel_circumference_m`
* `wheel_track_m`

Använd separata hjulomkretsar så att mindre skillnader mellan hjulen kan kalibreras individuellt.

`wheel_track_m` definieras som avståndet mellan centrumlinjerna för vänster respektive höger drivhjul.

Dödräkningen används bland annat för:

* `search_length_m`
* `post_pick_lockout_distance_m`
* `row_heading_window_m`
* `heading_reference_min_distance_m`
* turn-manövrer där den befintliga implementationen använder sträcka,
* `turn_marker_rearm_distance_m`.

---

# 23. Webbgränssnitt

Webbgränssnittet ska återanvända strukturen och säkerhetsprinciperna från `remote_control`.

Det ska minst innehålla:

### Drift

* val MANUAL/AUTO,
* Start Auto,
* STOP,
* manuella styrknappar,
* manuell hastighet.

### Relevanta konfigurationsparametrar

Operatören ska kunna läsa och ändra lämpliga runtime-parametrar.

Ändringar ska hanteras kontrollerat och inte skapa inkonsistenta states mitt under en manöver.

Om vissa parametrar endast bör ändras när roboten står still ska detta implementeras.

---

# 24. Diagnostik i webbgränssnittet

Visa minst:

* aktuellt state,
* aktuell fysisk rad,
* aktuell körriktning/pass,
* `filtered_heading`,
* `row_heading_reference`,
* om headingreferensen ännu är tillförlitlig eller fortfarande byggs upp,
* körd giltig sträcka för uppbyggnad av headingreferens,
* `heading_error`,
* aktuellt filtrerat `target_x`,
* `x_goal`,
* `x_error`,
* kommenderad rpm vänster motor,
* kommenderad rpm höger motor,
* körd sträcka sedan visuellt mål förlorades,
* körd sträcka sedan senaste pick,
* kamera OK/fel,
* IMU OK/fel,
* CAN OK/fel,
* odometri OK/fel,
* senaste felorsak,
* senaste pick timeout om sådan inträffat.

Diagnostiken ska vara avsedd för praktisk felsökning under fälttest.

---

# 25. Kamerazoner

Följande zoner ska finnas:

* `navigation_zone`
* `trigger_zone`
* `pick_zone`
* `turn_marker_zone` om den används

Varje zon definieras av:

* x-min
* x-max
* y-min
* y-max

Använd helst normaliserade koordinater:

`0.0 ... 1.0`

för både x och y.

På så sätt är zonerna oberoende av faktisk kameraprocesseringsupplösning.

---

# 26. HSV-konfiguration

Separata HSV-filter ska finnas för:

### Buds

* `bud_hsv_low`
* `bud_hsv_high`
* `bud_min_area`

### Leaves

* `leaf_hsv_low`
* `leaf_hsv_high`
* `leaf_min_area`

### Turn marker

* `marker_hsv_low`
* `marker_hsv_high`
* `marker_min_area`

Återanvänd befintlig HSV-behandling där sådan redan är verifierad.

---

# 27. Processing och livestream ska separeras

Kamerans processeringsupplösning och webbens livestreamupplösning ska vara oberoende.

Exempel:

`processing_width = 640`

`processing_height = 480`

Processeringsupplösningen ska inte automatiskt ändras när operatören ändrar livestreamens upplösning.

Bildbehandlingsfrekvensen ska konfigureras separat:

`navigation_frame_rate_hz`

---

# 28. Livestream

Webbgränssnittet ska kunna visa två diagnostikströmmar:

### A. Ofiltrerad kamerabild

Den ska visa overlays för:

* `navigation_zone`
* `trigger_zone`
* `pick_zone`
* `turn_marker_zone` om sådan används,
* vertikal linje för `x_goal`,
* eventuellt aktuellt `target_x`.

Det ska finnas en tydlig legend/förklaring.

### B. HSV-diagnostik

Visa tydligt vilka områden som identifierats som:

* knoppar,
* blast,
* vändmarkör.

Det är bättre att visualisera de tre maskerna med separata färger i samma diagnostikbild än att slå ihop allt till en enda vit mask.

Alternativt får gränssnittet erbjuda val av vilken mask som visas.

---

# 29. Livestream får inte påverka navigation

Streaming och webbkommunikation får aldrig blockera:

* kamera-loop,
* bildbehandling,
* IMU-loop,
* navigation,
* motorstyrning,
* watchdog.

Streaming ska använda principen:

`latest_frame`

Om webbläsaren eller nätverket inte hinner konsumera frames ska gamla frames kastas.

Ingen växande kö med gamla frames får byggas upp.

Det är tillåtet att:

* sänka streaming frame rate,
* sänka upplösningen,
* öka JPEG-komprimeringen,
* ersätta streamen med uppdaterade stillbilder,
* stänga av streaming helt.

Konfigurationsparametrar:

* `stream_enabled`
* `stream_fps`
* `stream_width`
* `stream_height`
* `jpeg_quality`

Navigationens funktion och timing har alltid högre prioritet än livestreamen.

---

# 30. Grundläggande motorparametrar

Följande ska vara konfigurerbara där motsvarande parameter inte redan måste vara fixerad av verifierad hårdvarukonfiguration:

* `auto_base_rpm`
* `search_speed_rpm`
* `turn_speed_rpm`
* `manual_rpm`
* `max_rpm`
* `vision_kp`
* `vision_deadband_px`
* `max_vision_correction_rpm`
* `heading_kp`
* `heading_deadband_deg`
* `max_heading_correction_rpm`

Återanvänd befintlig verifierad accelerations-/rampfunktion från tidigare projekt.

Om en max acceleration behöver vara explicit konfigurerbar kan:

`acceleration_limit`

användas.

---

# 31. Sensor- och kommunikationstimeouts

Följande ska finnas eller återanvändas från tidigare verifierad implementation:

* `camera_timeout_s`
* `imu_timeout_s`
* `odometry_timeout_s`
* CAN/watchdog timeout från verifierad motorstyrning

Om kritisk data inte uppdaterats inom tillåten tid:

1. stoppa motorerna,
2. gå till `FAULT`,
3. visa och logga felorsaken.

---

# 32. Konfigurationsparametrar

Konfigurationen ska minst stödja följande grupper.

## Navigation

* `navigation_mode`
* `auto_base_rpm`
* `vision_kp`
* `vision_deadband_px`
* `max_vision_correction_rpm`
* `x_goal`
* `x_filter_window_frames`
* `x_outlier_threshold_px`
* `navigation_lost_timeout_s`
* `navigation_reacquire_frames`

## Heading / IMU

* `heading_filter_alpha`, i samma betydelse och implementation som i `get_heading`
* `imu_sample_hz`
* `row_heading_window_m`
* `heading_reference_min_distance_m`
* `heading_kp`
* `heading_deadband_deg`
* `max_heading_correction_rpm`
* `imu_timeout_s`

## Search

* `search_length_m`
* `search_speed_rpm`

## Picking

* `max_pick_wait_s`
* `pick_clear_time_s`
* `post_pick_lockout_distance_m`

## Turns

* `in_row_turn_enabled`
* `new_row_turn_direction`
* `row_spacing_m`
* `number_of_rows`
* `turn_speed_rpm`
* `turn_marker_confirm_frames`
* `turn_marker_rearm_distance_m`

## Robotgeometri / odometri

* `left_wheel_circumference_m`
* `right_wheel_circumference_m`
* `wheel_track_m`
* `odometry_timeout_s`

## Camera

* `processing_width`
* `processing_height`
* `navigation_frame_rate_hz`
* `camera_timeout_s`

## HSV

* `bud_hsv_low`
* `bud_hsv_high`
* `bud_min_area`
* `leaf_hsv_low`
* `leaf_hsv_high`
* `leaf_min_area`
* `marker_hsv_low`
* `marker_hsv_high`
* `marker_min_area`

## Zones

* `navigation_zone`
* `trigger_zone`
* `pick_zone`
* `turn_marker_zone`

## Manual control

* `manual_rpm`

## Motor limits

* `max_rpm`
* eventuell `acceleration_limit`

## Auto start

* `auto_start_delay_s`

## Streaming

* `stream_enabled`
* `stream_fps`
* `stream_width`
* `stream_height`
* `jpeg_quality`

## Logging

* `log_level`

---

# 33. Automatisk state-logik i sammanfattning

Den principiella state-maskinen ska vara:

```text
AUTO_ROW_FOLLOW
    |
    | bud in trigger_zone
    v
AUTO_PICK
    |
    | pick clear OR timeout
    v
AUTO_POST_PICK
    |
    | lockout distance reached
    v
AUTO_ROW_FOLLOW


AUTO_ROW_FOLLOW / AUTO_POST_PICK
    |
    | visual navigation targets lost
    v
AUTO_SEARCH
    |
    +---- targets reacquired ---> AUTO_ROW_FOLLOW
    |
    +---- turn marker ----------> TURN STATE
    |
    +---- search_length reached -> FAULT / ROW_LOST


AUTO_ROW_FOLLOW / AUTO_SEARCH / AUTO_POST_PICK
    |
    | turn marker
    v
AUTO_IN_ROW_TURN or AUTO_NEW_ROW_TURN
    |
    v
AUTO_ROW_FOLLOW or AUTO_SEARCH


all active states
    |
    | critical sensor/CAN/watchdog failure
    v
FAULT
```

---

# 34. Central styrprincip

När ett giltigt visuellt navigationsmål finns:

```text
styr med x_error och vision_kp
```

När giltiga visuella navigationsmål saknas men headingbaserad körning fortfarande är tillåten:

```text
styr med heading_error =
row_heading_reference - filtered_heading
```

När erforderlig sensorinformation saknas:

```text
STOP
FAULT
```

`row_heading_reference` får användas som primär fallbackreferens först när den har byggts upp från minst:

`heading_reference_min_distance_m`

giltig visuell körsträcka, eller när den uttryckligen har härletts från en tidigare verifierad radriktning efter en 180-graders turn.

---

# 35. Arkitektur

Sträva efter tydlig separation mellan:

* hardware drivers,
* camera acquisition,
* vision processing,
* IMU,
* odometry,
* motor control,
* navigation controllers,
* state machine,
* configuration,
* web UI,
* diagnostics/logging.

State-maskinen ska styra **vilken befintlig controller som är aktiv**, snarare än att duplicera motorlogik i varje state.

Exempelvis bör visuell styrning och headingstyrning kunna vara separata controllers som lämnar önskade vänster/höger motorhastigheter till samma gemensamma motorlager.

Robotgeometrin, inklusive:

* hjulomkretsar,
* `wheel_track_m`

ska ligga i en gemensam konfiguration och inte dupliceras i olika controllers.

---

# 36. Timing och trådar/processer

Navigationens realtidsnära funktioner ska inte vara beroende av webbgränssnittets timing.

Webbserver, streaming och diagnostik får inte blockera motorstyrning eller sensorinsamling.

Undvik onödiga parallella processer om enklare arkitektur räcker, men separera blockerande I/O från navigationsloopen.

Återanvänd tidigare verifierade watchdog-principer.

`auto_start_delay_s` får inte implementeras på ett sätt som blockerar huvudloopen. Under eventuell startfördröjning ska sensorövervakning, STOP och säkerhetsfunktioner fortsätta fungera.

---

# 37. Logging

Logga åtminstone:

* state transitions,
* start/stop,
* AUTO/MANUAL-byte,
* Start Auto,
* eventuell `auto_start_delay_s`,
* radnummer,
* turn start/slut,
* `row_heading_reference` före och efter turn,
* när `row_heading_reference` blir tillförlitlig efter `heading_reference_min_distance_m`,
* förlust av visuella mål,
* återfunna visuella mål,
* start och slut på SEARCH,
* search distance,
* pick start,
* pick clear,
* pick timeout,
* turn marker detection,
* `FAULT`,
* sensor-timeouts,
* CAN/watchdog-fel.

Loggningen får inte blockera navigationen.

---

# 38. Implementationsstrategi

Arbeta i följande ordning.

### Steg 1 – inventera befintlig kod

Undersök:

* `oak_d_sr_navigation`
* `get_heading`
* `remote_control`

Dokumentera kort vilka befintliga komponenter som ska återanvändas för varje funktion.

### Steg 2 – föreslå arkitektur

Definiera:

* moduler,
* state machine,
* interfaces mellan vision/IMU/odometry/motor control,
* konfigurationsstruktur.

Undvik större omskrivning av verifierad låg-nivåkod.

### Steg 3 – implementera integrationen

Integrera de befintliga komponenterna i `field_control`.

### Steg 4 – tester

Lägg till fokuserade tester för framför allt:

* state transitions,
* sensor timeout,
* SEARCH,
* återfångning av rad,
* PICK,
* POST_PICK lockout,
* turn-marker debounce,
* turn-marker rearm,
* radnummer,
* `in_row_turn_enabled`,
* `number_of_rows`,
* headingreference efter 180° turn,
* `heading_reference_min_distance_m`,
* `turn_speed_rpm`,
* `wheel_track_m` där det används i vändnings-/differentialkinematik,
* MANUAL/AUTO-transition,
* `auto_start_delay_s`,
* STOP under startfördröjning,
* motor-watchdog.

---

# 39. Säkerhetsprincip

Om systemet är osäkert på om fortsatt körning är tillåten ska motorerna stoppas.

Ingen av följande funktioner får kunna orsaka obegränsad blind körning:

* förlust av visuella mål,
* kameraavbrott,
* IMU-avbrott,
* odometrifel,
* turn failure,
* webbkommunikationsfel.

Webbklientens bortkoppling får aldrig lämna motorerna i ett osäkert tillstånd.

Startfördröjningen `auto_start_delay_s` får alltid avbrytas med STOP.

Om roboten saknar en tillförlitlig `row_heading_reference` och samtidigt förlorar visuella navigationsmål ska den inte försöka skapa en godtycklig fallback-heading. Den ska stoppa eller gå till ett tydligt fel-/vänteläge beroende på den slutliga state-machine-designen.

---

# 40. Viktiga designmål

Prioritera i denna ordning:

1. säker och deterministisk motorstyrning,
2. korrekt state machine,
3. återanvändning av verifierad funktionalitet,
4. robust navigation,
5. diagnostik,
6. livestream och visuell presentation.

Livestream och webb-UI får aldrig prioriteras framför navigation och motorsäkerhet.

Gör initialt implementationen så enkel som möjligt.

Lägg inte till avancerad prediktion, Kalmanfilter, AI-baserad radmodell eller annan ny navigationsalgoritm om det inte krävs för den specificerade funktionen.

Den första versionen ska i första hand kombinera de redan verifierade byggblocken till ett robust och begripligt system.

---

# 41. Komplett lista över konfigurerbara parametrar

Nedan följer de parametrar som ska kunna ligga i konfigurationsfilen. Parametrar som är lämpliga för operatören ska även kunna visas och vid behov ändras i webbgränssnittet.

Det är inte nödvändigt att alla hårdvaru- och säkerhetsparametrar är ändringsbara medan roboten kör.

## 41.1 Visuell navigation

| Parameter                     | Förklaring                                                                                                                           |
| ----------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `navigation_mode`             | Anger vilka visuella objekt som används för radföljning. Tillåtna värden minst `buds_only` och `buds_and_leaves`.                    |
| `auto_base_rpm`               | Grundhastighet vid normal automatisk radföljning, uttryckt som rpm på hjulmotorernas utgående axel.                                  |
| `vision_kp`                   | P-faktor för bildbaserad styrning. Avgör hur kraftigt motorhastigheterna korrigeras utifrån `x_error`.                               |
| `vision_deadband_px`          | Om absolutvärdet av `x_error` är mindre än detta antal pixlar görs ingen styrkorrigering.                                            |
| `max_vision_correction_rpm`   | Maximalt tillåten differentialkorrektion i rpm från den visuella regulatorn.                                                         |
| `x_goal`                      | Den x-position i den processade bilden där navigationsmålet ska ligga.                                                               |
| `x_filter_window_frames`      | Antal giltiga bildmätningar som används för tidsmässigt glidande medelvärde av `target_x`.                                           |
| `x_outlier_threshold_px`      | Gräns för hur kraftigt en ny x-mätning får avvika innan den eventuellt ignoreras som outlier. Funktionen kan initialt vara avstängd. |
| `navigation_lost_timeout_s`   | Hur länge giltiga visuella navigationsmål får saknas innan systemet går till `AUTO_SEARCH`.                                          |
| `navigation_reacquire_frames` | Antal konsekutiva giltiga detektioner som krävs innan systemet återgår från `AUTO_SEARCH` till visuell radföljning.                  |

## 41.2 IMU och heading

| Parameter                          | Förklaring                                                                                                                                      |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `heading_filter_alpha`             | Alpha-värde för det verifierade lågpassfiltret från `get_heading`. Samma definition och implementation som i den befintliga koden ska användas. |
| `imu_sample_hz`                    | Samplingsfrekvens för IMU-data.                                                                                                                 |
| `row_heading_window_m`             | Hur lång giltig körsträcka bakåt som används för att beräkna `row_heading_reference`.                                                           |
| `heading_reference_min_distance_m` | Minsta sträcka med stabil, giltig visuell radföljning som krävs innan en ny `row_heading_reference` betraktas som tillförlitlig.                |
| `heading_kp`                       | P-faktor för headingbaserad styrning när visuella navigationsmål saknas.                                                                        |
| `heading_deadband_deg`             | Headingfel mindre än denna vinkel ger ingen styrkorrigering.                                                                                    |
| `max_heading_correction_rpm`       | Maximalt tillåten differentialkorrektion i rpm från headingregulatorn.                                                                          |
| `imu_timeout_s`                    | Maximal tillåten tid utan ny giltig IMU-data innan motorerna stoppas och systemet går till `FAULT`.                                             |

## 41.3 Sökning när raden inte syns

| Parameter          | Förklaring                                                                                                 |
| ------------------ | ---------------------------------------------------------------------------------------------------------- |
| `search_length_m`  | Maximal sträcka roboten får köra utan visuella navigationsmål innan den stoppar med exempelvis `ROW_LOST`. |
| `search_speed_rpm` | Grundhastighet under `AUTO_SEARCH`. Kan vara lägre än `auto_base_rpm`.                                     |

## 41.4 Skörd och stopp vid knoppar

| Parameter                      | Förklaring                                                                                                  |
| ------------------------------ | ----------------------------------------------------------------------------------------------------------- |
| `max_pick_wait_s`              | Maximal tid roboten står still i `AUTO_PICK` innan den fortsätter även om knoppar fortfarande detekteras.   |
| `pick_clear_time_s`            | Hur länge `pick_zone` kontinuerligt måste vara utan knoppar innan skörden betraktas som avslutad.           |
| `post_pick_lockout_distance_m` | Minsta sträcka roboten måste köra efter `AUTO_PICK` innan knoppar i `trigger_zone` åter får stoppa roboten. |

## 41.5 Vändningar och rader

| Parameter                      | Förklaring                                                                                                                               |
| ------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `in_row_turn_enabled`          | Anger om roboten ska köra samma fysiska rad i båda riktningarna innan den går vidare till nästa rad.                                     |
| `new_row_turn_direction`       | Anger åt vilket håll roboten ska gå till nästa rad. Tillåtna värden `left` och `right`.                                                  |
| `row_spacing_m`                | CC-avstånd mellan två intilliggande saffransrader.                                                                                       |
| `number_of_rows`               | Antal unika fysiska rader som ska bearbetas.                                                                                             |
| `turn_speed_rpm`               | Grundhastighet under `AUTO_IN_ROW_TURN` och `AUTO_NEW_ROW_TURN`, om den verifierade turn-implementationen stödjer separat vändhastighet. |
| `turn_marker_confirm_frames`   | Antal konsekutiva frames där en vändmarkör måste detekteras innan den godkänns.                                                          |
| `turn_marker_rearm_distance_m` | Minsta sträcka efter en vändning innan en ny vändmarkör får trigga.                                                                      |

## 41.6 Robotgeometri och odometri

| Parameter                     | Förklaring                                                                                                          |
| ----------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| `left_wheel_circumference_m`  | Effektiv omkrets på vänster drivhjul. Används för dödräkning och kan kalibreras separat.                            |
| `right_wheel_circumference_m` | Effektiv omkrets på höger drivhjul.                                                                                 |
| `wheel_track_m`               | Avstånd mellan vänster och höger drivhjuls centrumlinjer. Används i differentialkinematik och vändningsberäkningar. |
| `odometry_timeout_s`          | Maximal tillåten tid utan giltig odometridata innan motorerna stoppas och systemet går till `FAULT`.                |

## 41.7 Kamera och bildbehandling

| Parameter                  | Förklaring                                                              |
| -------------------------- | ----------------------------------------------------------------------- |
| `processing_width`         | Bredd på bilden som används av navigationsalgoritmen.                   |
| `processing_height`        | Höjd på bilden som används av navigationsalgoritmen.                    |
| `navigation_frame_rate_hz` | Frekvens med vilken nya kamerabilder används av navigationen.           |
| `camera_timeout_s`         | Maximal tid utan ny giltig kameraframe innan systemet går till `FAULT`. |

## 41.8 HSV-filter för knoppar

| Parameter      | Förklaring                                                                            |
| -------------- | ------------------------------------------------------------------------------------- |
| `bud_hsv_low`  | Nedre HSV-gräns för detektion av violetta saffransknoppar.                            |
| `bud_hsv_high` | Övre HSV-gräns för detektion av violetta saffransknoppar.                             |
| `bud_min_area` | Minsta accepterade bildarea för en knoppdetektion. Mindre områden ignoreras som brus. |

## 41.9 HSV-filter för blast

| Parameter       | Förklaring                                      |
| --------------- | ----------------------------------------------- |
| `leaf_hsv_low`  | Nedre HSV-gräns för detektion av grön blast.    |
| `leaf_hsv_high` | Övre HSV-gräns för detektion av grön blast.     |
| `leaf_min_area` | Minsta accepterade bildarea för blastdetektion. |

## 41.10 HSV-filter för vändmarkörer

| Parameter         | Förklaring                                     |
| ----------------- | ---------------------------------------------- |
| `marker_hsv_low`  | Nedre HSV-gräns för detektion av vändmarkören. |
| `marker_hsv_high` | Övre HSV-gräns för detektion av vändmarkören.  |
| `marker_min_area` | Minsta accepterade bildarea för en vändmarkör. |

## 41.11 `navigation_zone`

Zonen bör helst anges med normaliserade koordinater mellan `0.0` och `1.0`.

| Parameter               | Förklaring                                                       |
| ----------------------- | ---------------------------------------------------------------- |
| `navigation_zone.x_min` | Vänster gräns för området som används för visuell radnavigation. |
| `navigation_zone.x_max` | Höger gräns för området som används för visuell radnavigation.   |
| `navigation_zone.y_min` | Övre gräns för området som används för visuell radnavigation.    |
| `navigation_zone.y_max` | Nedre gräns för området som används för visuell radnavigation.   |

## 41.12 `trigger_zone`

| Parameter            | Förklaring                                                         |
| -------------------- | ------------------------------------------------------------------ |
| `trigger_zone.x_min` | Vänster gräns för området där en knopp kan trigga stopp för skörd. |
| `trigger_zone.x_max` | Höger gräns för området där en knopp kan trigga stopp för skörd.   |
| `trigger_zone.y_min` | Övre gräns för triggerområdet.                                     |
| `trigger_zone.y_max` | Nedre gräns för triggerområdet.                                    |

## 41.13 `pick_zone`

| Parameter         | Förklaring                                                 |
| ----------------- | ---------------------------------------------------------- |
| `pick_zone.x_min` | Vänster gräns för området som övervakas under `AUTO_PICK`. |
| `pick_zone.x_max` | Höger gräns för området som övervakas under `AUTO_PICK`.   |
| `pick_zone.y_min` | Övre gräns för `pick_zone`.                                |
| `pick_zone.y_max` | Nedre gräns för `pick_zone`.                               |

## 41.14 `turn_marker_zone`

| Parameter                | Förklaring                                       |
| ------------------------ | ------------------------------------------------ |
| `turn_marker_zone.x_min` | Vänster gräns för området där vändmarkörer söks. |
| `turn_marker_zone.x_max` | Höger gräns för området där vändmarkörer söks.   |
| `turn_marker_zone.y_min` | Övre gräns för området där vändmarkörer söks.    |
| `turn_marker_zone.y_max` | Nedre gräns för området där vändmarkörer söks.   |

`turn_marker_zone` kan vara valfri om hela bilden ska användas för markördetektion.

## 41.15 Manuell körning

| Parameter    | Förklaring                                                      |
| ------------ | --------------------------------------------------------------- |
| `manual_rpm` | Normal hjulhastighet vid manuell körning från webbgränssnittet. |

## 41.16 Motorbegränsningar

| Parameter            | Förklaring                                                                                                                                      |
| -------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `max_rpm`            | Absolut maximal tillåten rpm på hjulmotorernas utgående axlar oberoende av state eller regulator.                                               |
| `acceleration_limit` | Maximal tillåten förändring av motorhastigheten per tidsenhet, om detta inte redan hanteras som en fast del av den verifierade motorstyrningen. |

Om befintlig motorstyrning har en verifierad rampfunktion som inte behöver konfigureras ska den återanvändas i stället för att skapa ny rampfunktionalitet.

## 41.17 Start av automatisk körning

| Parameter            | Förklaring                                                                                                                                                                      |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `auto_start_delay_s` | Fördröjning mellan operatörens Start Auto och att roboten börjar köra. STOP och övriga säkerhetsfunktioner ska fungera under hela fördröjningen. `0` innebär ingen fördröjning. |

## 41.18 Livestream och webbvideo

| Parameter        | Förklaring                                                                                                     |
| ---------------- | -------------------------------------------------------------------------------------------------------------- |
| `stream_enabled` | Slår på eller av diagnostisk livestream.                                                                       |
| `stream_fps`     | Uppdateringsfrekvens för webbströmmen. Ska vara oberoende av `navigation_frame_rate_hz`.                       |
| `stream_width`   | Bildbredd för webbströmmen.                                                                                    |
| `stream_height`  | Bildhöjd för webbströmmen.                                                                                     |
| `jpeg_quality`   | JPEG-kvalitet/komprimeringsgrad för webbströmmen. Ska kunna sänkas för att minska CPU- och nätverksbelastning. |

## 41.19 Logging

| Parameter   | Förklaring                                                     |
| ----------- | -------------------------------------------------------------- |
| `log_level` | Loggnivå, exempelvis `DEBUG`, `INFO`, `WARNING` eller `ERROR`. |

## 41.20 Samlad parameterlista

```text
navigation_mode

auto_base_rpm
vision_kp
vision_deadband_px
max_vision_correction_rpm

x_goal
x_filter_window_frames
x_outlier_threshold_px

navigation_lost_timeout_s
navigation_reacquire_frames

heading_filter_alpha
imu_sample_hz
row_heading_window_m
heading_reference_min_distance_m
heading_kp
heading_deadband_deg
max_heading_correction_rpm
imu_timeout_s

search_length_m
search_speed_rpm

max_pick_wait_s
pick_clear_time_s
post_pick_lockout_distance_m

in_row_turn_enabled
new_row_turn_direction
row_spacing_m
number_of_rows
turn_speed_rpm
turn_marker_confirm_frames
turn_marker_rearm_distance_m

left_wheel_circumference_m
right_wheel_circumference_m
wheel_track_m
odometry_timeout_s

processing_width
processing_height
navigation_frame_rate_hz
camera_timeout_s

bud_hsv_low
bud_hsv_high
bud_min_area

leaf_hsv_low
leaf_hsv_high
leaf_min_area

marker_hsv_low
marker_hsv_high
marker_min_area

navigation_zone
trigger_zone
pick_zone
turn_marker_zone

manual_rpm

max_rpm
acceleration_limit

auto_start_delay_s

stream_enabled
stream_fps
stream_width
stream_height
jpeg_quality

log_level
```
