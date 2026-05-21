# Tasks and Implementation Steps — Perceptivno-manipulacijski sustav

Kratak opis: implementirati autonomni pick-and-place pipeline za unaprijed definiran tip objekta koristeći RealSense RGB-D kameru i UR robot sa soft gripperom.

## 📋 Redosljed Zadataka (Pipeline)

```
1. KALIBRACIJA
   ├─ Kalibracija TCP-a
   ├─ Eye-in-hand kalibracija
   └─ Output: T_camera2robot, T_tcp, matrice transformacija

2. SNIMANJE SCENE — RGB-D iz više kutova
   ├─ 3 pozicije: iznad, 45°, 45° drugačiji kut
   ├─ RGB frame + Depth mapa za svaku
   └─ Output: 3× .pcd/.ply datoteke

3. DETEKCIJA I LOKALIZACIJA VOĆA — YOLO + PCL Pipeline
   ├─ 3a) YOLO detekcija na RGB slici
   ├─ 3b) Pass-through filter + Statistical outlier removal
   ├─ 3c) Voxel downsampling (5 mm)
   ├─ 3d) ICP registration između 3 cloud-a
   ├─ 3e) Euclidean clustering (segmentacija objekata)
   ├─ 3f) Pronalaženje centroida u 3D
   ├─ 3g) Transformacija iz kamerom u robot koordinate
   └─ Output: objects_with_robot_coords.json (voće + 3D centroidi)

4. OBRADA TRAJEKTORIJA — Pickup pozicije
   ├─ HOME pozicija
   ├─ APPROACH (100 mm iznad objekta)
   ├─ PICK (na centroidu)
   ├─ APPROACH_PLACE (100 mm iznad pick lokacije)
   └─ Output: pickup_coordinates.json

5. GENERIRANJE TRAJEKTORIJA
   ├─ Kvintička interpolacija između točaka
   ├─ Diskretizacija sa ograničenjima brzine/akceleracije
   └─ Output: trajectory.npy, grafovi x(t), ẋ(t), ẍ(t)

6. IZVRŠENJE PICK-AND-PLACE NA ROBOTU
   ├─ TCP/IP komunikacija (port 30003)
   ├─ Stream referentnih točaka
   ├─ Gripper kontrola (open/close)
   └─ Output: demo video

7. PREDAJA I DOKUMENTACIJA
   ├─ Izvješće (10 str)
   ├─ Video demo
   └─ Paket: kod + video + izvješće
```

---

## 🔑 Ključni Fajlovi i Biblioteke

### Biblioteke
- **OpenCV** - RGB obrada, detekcija markera
- **Open3D** - Point cloud obrada i vizualizacija
- **Ultralytics YOLO** - Detekcija voća (`yolo26n_trained.pt`)
- **NumPy/SciPy** - Matematika, transformacije
- **Matplotlib** - Grafici i analiza
- **pyrealsense2** - RealSense kamera (ako dostupna)
- **urx ili RTDE** - UR robot komunikacija

### Struktura `src/` modula (iz STATE.md)
```
src/
├── _01_percepcija/
│   ├── realsense_capture.py    # Snimanje RGB-D
│   └── pcl_processing.py       # Filtriranje, segmentacija
├── _02_detekcija/
│   └── object_detector.py      # YOLO inference
├── _03_planiranje/
│   └── trajectory_planner.py   # Generiranje trajektorija
├── _04_izvrsenje/
│   └── robot_controller.py     # UR komunikacija
├── utils/
│   ├── transforms.py           # Transformacijske matrice
│   └── visualization.py        # Vizualizacija
├── config.py                   # Globalne varijable
├── main.py                     # Glavna CLI aplikacija
└── test_hardware.py            # Testiranje veza
```

### Očekivani Izlazni Fajlovi
```
data/
├── raw/
│   ├── scene_01_pos1.pcd
│   ├── scene_01_pos2.pcd
│   └── scene_01_pos3.pcd
├── processed/
│   ├── scene_01_pos1_filtered.pcd
│   ├── scene_01_pos2_registered.pcd
│   ├── final_merged_point_cloud.pcd
│   └── cluster_labels.npy
├── detections/
│   ├── frame_pos1_detections.json
│   ├── frame_pos2_detections.json
│   └── frame_pos3_detections.json
└── output/
    ├── objects_with_robot_coords.json  # ← KLJUČNI FILE
    ├── pickup_coordinates.json         # ← KLJUČNI FILE
    └── trajectory.json

models/
├── yolo26n_trained.pt          # YOLO model iz Zadace 2
└── calibration/
    └── T_camera2robot.npy      # Output iz Zadace 1
```

---

## 📊 Analiza Klasifikacije iz Zadace 2 → Primjena u Zadaci 3

### Kako je Klasifikacija Odradena u Zadaci 2?

**Model**: YOLOv8-nano s detektovanjem i segmentacijom
- Dataset: `Humanoidna_svo_voce_02.yolov8-obb` (7 klasa voća)
- Obučeni model: `yolo26n_trained.pt` ili `yolo26n-seg_trained.pt`
- Output inference-a:
  - **Bounding box**: `[x, y, w, h]` na 2D slici (u pikselima)
  - **Confidence score**: Koliko je model siguran (0-1)
  - **Class ID**: Koji tip voća (Banana=0, Red Apple=1, Lemon=2, itd.)
  - **Segmentation mask** (ako koristi `-seg` model): Piksel-level maska objekta

### Kako se Primjenjuje u Zadaci 3?

**Ulazne RGB slike** → **YOLO inference** → **Detektovana voća sa bounding boxovima**

```
RGB Frame (1280×720)
    ↓
  YOLO  (yolo26n_trained.pt)
    ↓
Output:
  - Red Apple at [x=100, y=150, w=50, h=60] (confidence=0.92)
  - Banana at [x=300, y=200, w=40, h=100] (confidence=0.88)
  - Lemon at [x=500, y=100, w=45, h=45] (confidence=0.85)
  ... (više objekata)
```

**Uloga u Pick-and-Place**:
1. YOLO nam **klasificira što je koju vrstu voća**
2. **Bounding box** nam daje **2D lokaciju na slici** (u pikselima)
3. **Depth mapa** (iz RealSense) → konvertuje 2D pixel u **3D koordinate** u kamerom sustavu
4. **Point cloud clustering** → pronalazi **centroid voća u 3D**
5. **Transformacija** → pretvara 3D centroid iz kamerom u **robot koordinate** za pickup

**Napomena**: Ovdje se **NE trenira novi model** — koristimo **pretreniranu YOLO mrežu** iz zadace 2!

---

## ⏱️ Detaljan Redosljed Koraka (Sekvencijalno)

### Nakon što je Kalibracija (Zadaća 1) završena:

1. **Priprema scene** (5 min)
   - Postavi 7 tipova voća na plohu
   - Očisti pozadinu i osiguraj dobru osvjetljenja

2. **Snimanje RGB-D sekvence iz 3 pozicije** (10 min)
   - Pozicija 1: Kamera direktno iznad scene
   - Pozicija 2: Kamera pod kutom ~45° (X smjer)
   - Pozicija 3: Kamera pod kutom ~45° (Y smjer)
   - Za svaku poziciju: Snimiti RGB frame + Depth frame → Konvertovati u .pcd

3. **YOLO Detekcija na RGB slici** (5 min)
   - Učitaj model: `yolo26n_trained.pt`
   - Pokreni inference na svakom RGB frame-u
   - Dobiti: `[class, confidence, bbox]` za svaki detektovani objekt
   - Spremi rezultate u JSON

4. **Point Cloud Obrada** (15 min):
   - **4a.** Pass-through filter (Z: 0-2m, X/Y prema radnoj plohi)
   - **4b.** Statistical Outlier Removal (20 susjeda, std_ratio=2)
   - **4c.** Voxel Grid Downsampling (voxel_size=5mm)
   - **4d.** ICP Registration (pos1 je referenca, registriraj pos2 i pos3)
   - **4e.** Merge/Stitch tri registrirane cloud-a
   - **4f.** Euclidean Clustering (tolerancija=2cm) → Dobij cluster labels
   - **4g.** Pronađi centroid svakog klastera
   - **4h.** Transformiraj centroide iz kamerom u robot sustav

5. **Pronalaženje Pickup Pozicija** (5 min)
   - Za svaki centroid: generiši APPROACH, PICK, PLACE točke
   - Validiraj dostižnost robota

6. **Generiranje Trajektorija** (10 min)
   - Interpoliraj između HOME → APPROACH → PICK → APPROACH_PLACE → PLACE
   - Diskretizacija i vizualizacija

7. **Testiranje na Robotu** (Variable)
   - Slanje trajektorije robotu
   - Gripper kontrola
   - Demo video snimanje

**Ukupno vrijeme izvršavanja pipeline-a (bez snimanja videa): ~50 minuta**

---


- Cilj: definicija TCP soft grippera i transformacija kamera ↔ TCP ↔ robot baza.
- Koraci:
  - Kalibrirati TCP: izmjeriti i definirati TCP točku za soft gripper.
  - Eye-in-hand kalibracija: snimiti par pozicija, izračunati transformaciju kamere prema TCP koristeći metode iz laboratorija.
  - Validacija: projicirati poznati 3D marker i provjeriti usklađenost.
- Output: matrice transformacija, skripta `calibration.py` i kratki log s greškama.

2) Snimanje scene — RGB-D kamenom iz više kutova
- Cilj: prikupiti najmanje 3 RGB-D snimke (u pravilu: RGB slika + depth mapa) iste scene iz RAZLIČITIH KUTOVA.
- Koraci:
  - Postaviti scenu s **7 tipova 3D-printanog voća** (Banana, Crvena jabuka, Limun, Naranca, Orah, Zelena jabuka, Pomelo ili slično).
  - Snimiti scenu s **3 različite pozicije kamere** (ili robota s kamerom):
    - **Pozicija 1**: Direktno iznad (Z+ smjer)
    - **Pozicija 2**: Iz kuta ~45° (Z+ i X+ smjer)
    - **Pozicija 3**: Iz drugog kuta ~45° (Z+ i Y+ smjer)
  - Za svaku poziciju: spremiti RGB frame, depth frame i konvertovati u point cloud (.pcd ili .ply format)
  - Organizirati podatke: `data/raw/scene_01_pos1.pcd`, `scene_01_pos2.pcd`, `scene_01_pos3.pcd`
  - Zabilježiti pose robota (position + orientation) pri svakom snimanju ako je robot mobilan
- Output: **3× .pcd/.ply datoteke**, RGB slike, depth mape, i log s informacijama o pose-u pri snimanju.

3) Detekcija i lokalizacija voća — YOLO + PCL Pipeline
- Cilj: **Za svaku od 3 scene detektovati voće koristeći YOLO, segmentirati ga u point cloudu, i pronaći centroide u 3D.**
- Koraci:

  **3a) YOLO Detekcija na RGB slici**
  - Koristiti pretreniranu YOLO neuronsku mrežu: `yolo26n_trained.pt` (iz zadace 2)
  - Za svaki RGB frame iz 3 pozicije:
    - Pokrenuti inference (YOLOv8 detection ili segmentation)
    - Dobiti: `[class_id, confidence, bounding_box_2D, mask_2D]` za svaki detektovani objekt
    - Klasifikacija: Banana, Red Apple, Lemon, Orange, Walnut, Green Apple, itd.
    - Spremi rezultate u JSON ili CSV: `frame_pos1_detections.json`
  - Filtriranje: odbaciti detektovane objekte s confidence < 0.5

  **3b) Point Cloud Filtriranje i Downsampling**
  - Pass-Through Filter:
    - Ograničiti Z-os: `0.0 m < Z < 2.0 m` (ukloniti točke van dosega)
    - Ograničiti X-os i Y-os prema veličini radne plohe
  - Statistical Outlier Removal:
    - Parametri: `nb_neighbors=20`, `std_ratio=2.0`
    - Svrha: ukloniti šumne točke iz depth senzora
  - Voxel Grid Downsampling:
    - Voxel size: `0.005 m` (5 mm) ili veće ovisno o rezoluciji
    - Svrha: smanjiti broj točaka s ~1M na ~100K za brže procesiranje
  - Spremi obrađene cloud-e: `data/processed/scene_01_pos1_filtered.pcd`, itd.

  **3c) Point Cloud Registration (ICP) i Stitching**
  - Registracija: usklađivanje 3 cloud-a iz različitih kutova
  - Algoritam: Iterative Closest Point (ICP)
    - Iteracije: 50
    - Threshold: 0.02 m
    - Koristi `pos1` kao referencu, registracija `pos2` i `pos3` prema `pos1`
  - Spajanje (Stitching): fuzija registriranih cloud-a u jedan
  - Rezultat: `final_merged_point_cloud.pcd`
  - **Visualizacija**: Open3D - prikaži početne 3 cloud-a i konačni merged cloud

  **3d) Euclidean Clustering (Segmentacija objekata)**
  - Algoritam: Euclidean Cluster Extraction
    - Tolerancija: `0.02 m` (2 cm)
    - Min/Max point count: `100 ~ 10000` (ovisno o veličini voća)
  - Output: labele svake točke → znaš kojem objektu pripada
  - Spremi labels: `cluster_labels.npy`

  **3e) Pronalaženje Centroida za svaki klaster**
  - Za svaki pronađeni klaster:
    - Izračunaj centroid (srednji položaj svih točaka): `centroid = mean(cluster_points)`
    - Spremi 3D koordinatu u **kamerom koordinatnom sustavu**: `[x_cam, y_cam, z_cam]` (metri)
  - Output: JSON datoteka s listom voća i njihovih centroida
    ```json
    {
      "objects": [
        {"id": 1, "type": "Red Apple", "centroid_camera": [0.1, 0.05, 0.8]},
        {"id": 2, "type": "Banana", "centroid_camera": [0.3, -0.1, 0.75]},
        ...
      ]
    }
    ```

  **3f) Transformacija centroida iz kamerom u robot koordinatni sustav**
  - Koristiti transformacijsku matricu iz kalibracije (Zadaća 1): `T_camera2robot`
  - Za svaki centroid:
    - Konvertovati iz 2D piksela u 3D (koristeći depth vrijednost)
    - Aplikuj transformaciju: `p_robot = T_camera2robot @ p_camera`
    - Output: `centroid_robot = [x_robot, y_robot, z_robot]`
  - Validacija: provjerite da li su koordinate fizički razumne (npr. unutar workspace robota)
  - Spremi konačne koordinate: `objects_with_robot_coords.json`

- Output:
  - Obrađeni point cloud-ovi (filtrirani, registrirani, fuziji)
  - Vizualizacije registracije i klasteriranja
  - JSON datoteka s voćem i njihovim 3D centroidama **u robot koordinatnom sustavu**
  - Log s broj detektovanih objekata i greškeama registracije
  - Grafički prikazi: before/after filtriranje, ICP fitness, clustering rezultati

**Pseudokod za Zadaću 3:**
```python
# 1. Snimanje / učitavanje
clouds = [load_pcd(f"scene_01_pos{i}.pcd") for i in 1:3]

# 2. YOLO detekcija
yolo_model = load_yolo("yolo26n_trained.pt")
detections = [run_inference(rgb_frame) for rgb_frame in rgb_frames]

# 3. Obrada cloud-a
clouds_filtered = [preprocess_cloud(cloud) for cloud in clouds]
clouds_downsampled = [downsample(cloud) for cloud in clouds_filtered]

# 4. ICP registracija
clouds_registered = [icp_register(cloud, clouds_downsampled[0]) for cloud in clouds_downsampled[1:]]
merged_cloud = merge_clouds([clouds_downsampled[0]] + clouds_registered)

# 5. Klasteriranje
labels = euclidean_clustering(merged_cloud)

# 6. Pronalaženje centroida
centroids_camera = get_centroids_per_cluster(merged_cloud, labels)

# 7. Transformacija
transform = load_calibration_matrix("T_camera2robot.npy")
centroids_robot = [transform @ centroid_cam for centroid_cam in centroids_camera]

# 8. Output
save_results({"objects": objects, "centroids_robot": centroids_robot})
```

5) Obrada trajektorija — Pickup pozicije
- Cilj: Iz centroida voća dobiti **pickup pozicije** koje robot može dosegnuti.
- Koraci:
  - Za svaki detektovani centroid (iz Zadatka 3):
    - PICK točka = centroid na Z-osi + offset vertikalno gore (npr. +20 mm)
    - APPROACH točka = PICK točka + offset iznad (npr. +100 mm po Z)
    - PLACE točka = fiksna lokacija (npr. dio stola gdje se skladišti)
    - APPROACH_PLACE = PLACE + offset iznad
  - Validiraj dostižnost: provjeri da li su točke unutar workspace-a robota
  - Spremi trajectory reference: `pickup_coordinates.json`
- Output: Koordinate za HOME, APPROACH, PICK, APPROACH_PLACE, PLACE

6) Generiranje trajektorija
- Cilj: iz zadatih točaka generirati izvedive, glatke trajektorije u task-space.
- Obavezne točke: HOME, APPROACH (iznad objekta), PICK, APPROACH_PLACE, PLACE.
- Koraci:
  - Definirati offset za APPROACH (npr. +100 mm po Z od PICK/PLACE).
  - Diskretizirati kretanje i osigurati ograničenja brzine/akceleracije.
  - Generirati međutrajektorije i eksportske reference `x_ref(t)` za slanje robotu.
  - Validirati trajektoriju u simulatoru (ako postoji) i vizualizirati x(t), ẋ(t), ẍ(t) za jedan segment.
- Output: `trajectory.npy` ili `trajectory.json`, grafovi i vizualizacije.

7) Izvršenje pick-and-place na robotu
- Cilj: robot autonomno izvršava pick-and-place bez ručne intervencije.
- Koraci:
  - Implementirati TCP/IP komunikaciju s UR robotom (koristeći UR API ili RTDE/urx).
  - Slati referentne točke/trajectory stream i kontrolirati gripper (open/close).
  - Dodati provjere sudara (stop) i sigurnosne limite.
  - Testirati najprije u simulatoru, zatim na stvarnom robotu.
- Output: demo video, logovi robota, safety checks.

8) Predaja i dokumentacija
- Pisano izvješće (<=10 stranica): arhitektura, kalibracija, obrada PC-a (prikazi za svaki PC), detekcija, analiza trajektorije (grafovi), zaključak.
- Video: snimiti pipeline radeći bez ručne intervencije; pokazati dvostruko izvođenje nakon ponovnog pozicioniranja objekta.
- Paket za predaju: .zip s kodom, videozapisom, izvješćem, i kratkim uputama za pokretanje.
- Rok: 22.5.2026. 23:59 (provjeriti upute predmeta)

Checkliste i preporuke
- Verzije i ovisnosti: navedite u `requirements.txt` ili `environment.yml`.
- Struktura repoa: `src/` (moduli), `data/` (raw i processed), `notebooks/` (analize), `docs/` (kratke upute), `scripts/` (kalibracija, pokretanje pipeline).
- Testiranje: razvijati i testirati svaki modul izolirano; koristiti vizualizacije za PC i trajektorije.
- Sigurnost: obavezno imati `EMERGENCY_STOP` i provjere sudara prije rada s robotom.

Kontakt i doprinos
- Svaki student radi samostalno na svom objektu (tablica objekata u zadatku). Zabilježiti ime i objekt u `README.md`.
