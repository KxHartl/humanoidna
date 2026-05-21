# Zadatak: Humanoidna – Zadatak 3

Kratki opis:
Ovaj dokument sadrži korake (1–7) koje studenti trebaju provesti za dovršetak zadatka. Svaki korak ima obavezne isporuke i nazive datoteka koje treba predati.

1. Kalibracije:
- Kalibracija TCP-a: definirati i kalibrirati TCP soft grippera.
- Kalibracija kamere (eye-in-hand): odrediti odnos TCP-a i kamere.
- Obavezno: `data/camera_calibration/T_cam_from_tcp.npy`, `camera_matrix.npy`, `dist_coeffs.npy`.
- Izlaz: log i backup matrica.

2. Snimanje scene:
- Snimiti najmanje 3 point clouda iste scene iz različitih pozicija (3 pozicije robota/kamere).
- Scena: 6 različitih objekata (po jedan primjerak svake klase voća).
- Obavezno: `data/raw/scene_<run>_pos1.pcd`, `pos2.pcd`, `pos3.pcd`.

3. Obrada point cloud podataka:
- Obavezni koraci: pass-through filter, StatisticalOutlierRemoval, VoxelGrid downsample, registracija (ICP), stitching/fuzija cloudova.
- Izlaz: `data/processed/final_merged_point_cloud.pcd` i filtrirani cloudovi.

4. Detekcija i lokalizacija objekta:
- Detekcija: YOLO (segmentacija) na RGB slikama; mapiranje maski u 3D pomoću depth podataka.
- Segmentacija u point cloudu i pronalaženje centroida svakog klastera.
- Transformacija centroida u robot koordinatni sustav pomoću kalibracijskih matrica.
- Izlaz: `data/processed/objects_with_robot_coords.json`.

5. Pick and place:
- Definirati HOME, APPROACH (npr. +100 mm), PICK (centroid ± offset), APPROACH_PLACE, PLACE.
- Izlaz: `data/processed/pickup_coordinates.json`.

6. Generiranje trajektorija:
- Generirati izvedivu i glatku trajektoriju u task-space (kvintička interpolacija ili ekvivalent).
- Poštovati ograničenja robota (vmax, amax) i diskretizirati dovoljno gusto za izvršenje.
- Izlaz: `data/processed/last_trajectory.json` i grafovi.
- SIGURNOST: tijekom puštanja programa paziti na sudare s stolom i objektima; testirati u simulaciji prije stvarne izvedbe.

7. Predaja i dokumentacija:
- Pripremiti izvještaj (`report.pdf` ili `report.md`) koji sadrži: opis metode, ključne parametre, rezultate, probleme i upute za reprodukciju (komande + verzije paketa).
- Spakirati sve obavezne izlazne datoteke u `submission.zip`.
- Rok predaje: 22.5.2026. u 23:59.
