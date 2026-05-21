# Tasks and Implementation Steps

Ovo je sažeti pregled izvorno planiranih zadataka i onoga što je u ovom repoju stvarno napravljeno.

## Zadaci koji su završeni

| Zadatak | Status | Napomena |
|---|---:|---|
| Kalibracija TCP-a i eye-in-hand transformacije | Završeno | Kalibracijski fajlovi su premješteni u `data/camera_calibration`. |
| Snimanje scene i organizacija ulaza | Završeno | Ulazni i izlazni podaci su razdvojeni u `data/raw`. |
| YOLO + point cloud pipeline | Završeno | Logika je konsolidirana u `src/02_fruit_pick_and_place`. |
| Planiranje pick-and-place trajektorije | Završeno | Trajektorijski modul je dio glavnog pipelinea. |
| Izvršenje na UR robotu | Završeno | URScript izvršavanje je uključeno kroz pipeline skripte. |
| Reorganizacija repozitorija | Završeno | Struktura je pojednostavljena i očišćena od duplikata. |

## Trenutna mapa projekta

| Zona | Svrha |
|---|---|
| `src/01_camera_calibration` | Skripte za kalibraciju i pomoćne provjere |
| `src/02_fruit_pick_and_place` | Glavni pick-and-place pipeline |
| `data/camera_calibration` | Matrice i transformacije |
| `data/models` | YOLO modeli |
| `data/raw` | Snimljeni i generirani podaci |

## Sažetak pipelinea

| Faza | Opis | Ishod |
|---|---|---|
| 1 | Kalibracija | Transformacijske matrice |
| 2 | Snimanje scene | RGB-D ulazi i point cloudovi |
| 3 | Detekcija i lokalizacija | Identificirani objekti i 3D centri |
| 4 | Rekonstrukcija i pick točke | Kandidati za pickup |
| 5 | Trajektorija | Izračun pokreta |
| 6 | Izvršenje | Slanje komandi robotu |

## Napomena

Originalni dugi plan je zadržan samo kao povijesni opis zadatka; za aktualno stanje pogledaj `STATE.md`.
