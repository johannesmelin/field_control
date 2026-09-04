# OAK-D-noteringar

Denna sammanfattning är begränsad till befintlig projektinformation i
`README.md` och dokumenten i denna katalog.

- Cam 1 använder OAK-D SR:s `CAM_B` och BNO086-IMU i en gemensam,
  serienummerbunden DepthAI-pipeline för rad 1–2.
- Cam 2 använder en separat, serienummerbunden video-only-källa för rad 3–4.
- Kamera och IMU exponeras som oberoende, bounded latest-value-källor. Åldrar
  mäts monotont och en växande frame-backlog används inte i styrningen.
- En aktiv Cam 2 utan färska frames stoppar AUTO med `CAMERA_2_TIMEOUT`.
  Äldre enkameraprofiler ska därför hålla rad 3–4 avstängda tills operatören
  aktiverar dem.
- OAK-D:s IMU-heading filtreras cirkulärt och använder kamerans
  fabrikskalibrering. Kamerabortfall är ett sensorfel, inte ett skäl att
  fortsätta sökning med gammal bilddata.

Detaljer om installation och HIL-förutsättningar finns i `README.md`; dessa
noteringar ersätter inte hårdvarudokumentation eller kalibreringsunderlag.
