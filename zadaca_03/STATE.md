# Stanje projekta

## Projekt
**Zadaća 3:** interaktivna Pick-and-Place CLI aplikacija za branje voća s UR5e robotom.

## Što je završeno

| Područje | Status | Napomena |
|---|---:|---|
| Arhitektura sustava | Završeno | Sustav podijeljen na adaptere (`VisionAdapter`, `RobotAdapter`) i orkestratora. |
| Vizijski pipeline | Završeno | YOLO detekcija + filtriranje po dubini (Z-os) + 3D DBSCAN klasteriranje. Point cloudovi se uspješno sijeku s 2D YOLO maskama. |
| Spajanje point cloudova | Završeno | Implementirana ICP (Iterative Closest Point) registracija za spajanje 3 pogleda. Uspješno pronalazi točne centroidne 3D točke objekata u prostoru. |
| Kalibracija i koordinatni prostori | Završeno | Riješena problematika Hand-Eye kalibracije (matrica V3) te dodan sub-centimetarski offset (-4.3, 5.5, 7.5 mm) u TCP prostor. |
| Kretanje i trajektorije | Završeno | RTDE pomaci implementirani (J i L linearna kretanja). Kvintički polinomi interpoliraju točke putanje s ubrzanjima. Pneumatika radi (DO 4, DO 5) za otvaranje i zatvaranje mekog grippera. |
| Generiranje izvješća | Završeno | Kompajliran PDF (LaTeX) dokument od 10 stranica s ugrađenim slikama iz robota i obrade (*run_1* i *run_2*). |
| Čišćenje repozitorija | Završeno | Pobrisane stare probne skripte, sačuvane korisne. |

## Trenutna struktura repozitorija

```text
src/
├── core/                   # Osnovne klase (FruitType, FruitObject) i konfig
├── adapters/               # VisionAdapter, RobotAdapter
├── use_cases/              # PlanningUseCase
└── main.py                 # (ako postoji) ili run_pipeline.py

scripts/
├── auto_capture_assets.py  # Alat za generiranje point-cloud vizualizacija za izvješće
├── testing/                # Skripte za kalibracijsko testiranje (ex src_testing)
└── yolo_training/          # Skripte za učenje YOLO mreže (ex src_yolo)

data/
├── camera_calibration/     # Matrice kamere
├── models/                 # YOLO težine (yolov8n-seg.pt)
└── raw/
	├── captures/           # Ulazne RGB-D snimke
	└── outputs/            # Razni run_XXXX izlazi i slike
```

## Završni Zaključak
Cijela funkcionalnost aplikacije ("perception, planning and action") u kontekstu branja voća radi. Svi koraci iz zadatka 3 su pokriveni i kompletirani u LaTeX obliku.
