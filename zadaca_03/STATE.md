# Stanje projekta

## Projekt
**Zadaća 3:** interaktivna Pick-and-Place CLI aplikacija za branje voća s UR5e robotom.

## Što je završeno

| Područje | Status | Napomena |
|---|---:|---|
| Vizijski pipeline | Optimizirano | Uveden Label-Constrained DBSCAN i optimizirani parametri. |
| Reorganizacija repozitorija | Završeno | Uvedena je jasna podjela na `src/` i `data/`. |
| Glavni pipeline | Završeno | Radna logika je konsolidirana u `src/02_fruit_pick_and_place`. |
| Kalibracija | Završeno | Kalibracijske datoteke su centralizirane u `data/camera_calibration`. |
| Modeli | Završeno | YOLO težine su smještene u `data/models`. |
| Outputi i privremeni podaci | Završeno | Debug i output artefakti idu u `data/raw/outputs`. |
| Dokumentacija stanja | Završeno | Ovaj dokument odražava trenutno stanje projekta. |

## Trenutna struktura

```text
src/
├── run_pipeline.py
├── 01_camera_calibration/
└── 02_fruit_pick_and_place/

data/
├── camera_calibration/
├── models/
└── raw/
	├── captures/
	└── outputs/
```

## Pokretanje

```bash
.venv\Scripts\activate
python -X utf8 src/run_pipeline.py
python -X utf8 src/run_pipeline.py --offline
python -X utf8 src/run_pipeline.py --existing-captures data/raw/outputs/run_XXXXX/captures
```

## Tok rada aplikacije
1. Snimanje scene iz više pozicija.
2. YOLO detekcija i lokalizacija voća.
3. Rekonstrukcija point clouda i izbor ciljnog objekta.
4. Planiranje pick i place trajektorije.
5. Izvršenje na robotu uz prikaz i zapis rezultata.
