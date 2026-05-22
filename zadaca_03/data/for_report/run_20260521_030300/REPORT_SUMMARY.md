# Podaci za izvješće - run_20260521_030300

Ova mapa sadrži sve materijale potrebne za poglavlje "Rezultati" u tvom seminaru.

## 1. Kalibracija (`/calibration`)
- Sadrži `camera_matrix.npy`, `dist_coeffs.npy` i `T_cam_from_tcp.npy`.
- Ove matrice definiraju unutrašnje parametre kamere i ruku-oko (hand-eye) transformaciju.

## 2. Point Cloud Obrada (`/pointclouds`)
- `view_X_original.pcd`: Sirov oblak točaka iz pojedinog pogleda.
- `view_X_filtered.pcd`: Oblak nakon voxel downsamplinga i uklanjanja outlier-a.
- `object_ID_KLASA.pcd`: Izdvojeni segmentirani plodovi voća (korišten YOLOv8 + DBSCAN).
- `final_merged_result.pcd`: Konačni spojeni oblak točaka cijele scene nakon ICP registracije (Strategija 5: Smart Fitness Rejection).

## 3. Detekcija Objekata (`/planning/objects_detected.json`)
- Sadrži listu svih prepoznatih plodova s njihovim pouzdanostima (confidence) i koordinatama centra (centroid) u koordinatnom sustavu baze robota.

## 4. Analiza Trajektorije (`/planning`)
- `trajectory_xyz_kinematics.png`: Grafovi pozicije, brzine i ubrzanja za X, Y, Z osi.
- `trajectory_3d_path.png`: 3D vizualizacija putanje TCP-a od početne do krajnje točke.
- `last_trajectory.json`: Svi podaci trajektorije za numeričku analizu.

## Kako generirati za drugi capture?
Ako želiš generirati iste ove materijale za neki drugi folder sa snimkama, pokreni:
```powershell
python src/generate_report_data.py putanja/do/captures
```
Nakon toga pokreni generiranje grafova (ako imaš `last_trajectory.json`):
```powershell
python src/generate_report_images.py putanja/do/last_trajectory.json data/for_report/IME_CAPTURA/planning
```
