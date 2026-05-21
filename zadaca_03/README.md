# Pick-and-Place Zadaća 3

Repozitorij za interaktivnu Pick-and-Place CLI aplikaciju za branje voća s UR5e robotom i RealSense RGB-D kamerom.

## Što projekt radi

- Snima scenu iz više pozicija kamere.
- Pokreće YOLO detekciju i lokalizaciju voća.
- Rekonstruira point cloud i računa 3D centroide.
- Planira pick i place trajektoriju.
- Šalje izvršenje prema UR robotu.

## Struktura

| Putanja | Svrha |
|---|---|
| `src/run_pipeline.py` | Glavni ulaz u pipeline |
| `src/01_camera_calibration/` | Skripte za kalibraciju i pomoćne provjere |
| `src/02_fruit_pick_and_place/` | Glavna logika pipelinea |
| `data/camera_calibration/` | Kalibracijske matrice i transformacije |
| `data/models/` | YOLO modeli |
| `data/raw/` | Snimljeni i generirani podaci |

## Pokretanje

Aktiviraj virtualno okruženje i pokreni glavni pipeline:

```powershell
.venv\Scripts\activate
python -X utf8 src\run_pipeline.py
```

Za offline rad bez robota i kamere:

```powershell
python -X utf8 src\run_pipeline.py --offline
```

Za rad s već snimljenim podacima:

```powershell
python -X utf8 src\run_pipeline.py --existing-captures data\raw\outputs\run_XXXXX\captures
```

## Ovisnosti

Osnovne Python ovisnosti su navedene u [requirements.txt](requirements.txt).

## Dokumentacija

- [STATE.md](STATE.md) - trenutno stanje projekta
- [tasks.md](tasks.md) - sažetak zadataka i što je završeno
- [howtouse.md](howtouse.md) - detaljnije upute za korištenje

## Napomena

Projekt je reorganiziran tako da su podaci, modeli i kalibracije odvojeni od izvornog koda i smješteni u odgovarajuće direktorije unutar repozitorija.
