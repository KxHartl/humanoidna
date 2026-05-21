# TO_SHOW — što predati za seminar

Ovaj dokument sažima sve obavezne stavke koje trebate predati i demonstrirati za završnu provjeru seminara.

## 1) Video (obavezno)
- Predati video koji prikazuje kompletan pipeline bez ručne intervencije tijekom demonstracije.
- Video mora sadržavati, redom:
  1. snimanje point-cloudova scene (skeniranje/rekonstrukcija),
  2. odlazak robota prema objektu (približavanje/pose),
  3. podizanje objekta,
  4. ostavljanje objekta na zadanu lokaciju.
- Nakon što robot jednom podigne i ostavi predmet, bez zaustavljanja videa treba predmet premjestiti na novo mjesto i ponovno pokrenuti pipeline (tj. još jedno pick-and-place u istom zapisu).
- Video ne mora biti profesionalno montiran, ali svi ključni koraci moraju biti jasno vidljivi.

## 2) Pisano izvješće (obavezno)
- Maksimalna duljina: 10 stranica, kratko i tehnički strukturirano.
- Nije potreban teorijski uvod — fokus na implementaciji i rezultatima.
- Izvješće mora sadržavati:
  - arhitekturu sustava (pipeline, kratak opis svakog modula),
  - kalibraciju: sve matrice transformacija (homogene transformacije između senzora i robota),
  - obradu point-cloudova: za svaki point cloud prikažite originalni PC, filtrirani PC, segmentirani PC te konačni rezultat registracije (sva tri PC-a), uz navođenje korištenih metoda i parametara,
  - detekciju objekata: opis načina segmentacije/detekcije i način određivanja centra objekta,
  - generiranje i analiza trajektorije: napraviti analizu jedne cjelovite generirane trajektorije — prikaz svih generiranih točaka, vremena međutrajektorija, vizualizaciju međutrajektorija s istaknutih pet glavnih TCP poza; za jedan segment priložiti grafove x(t), ẋ(t), ẍ(t) (pozicija, brzina, ubrzanje),
  - korišteni parametri i kratko objašnjenje kako su odabrani.

## 3) Datoteke koje treba predati
- Video datoteka (mp4, mkv ili sličan format). Naziv: `studentID_video.mp4` ili jasno označen.
- Pisano izvješće u PDF-u: `studentID_report.pdf` (<=10 stranica).
- Izvorni kod / repozitorij ili arhiva: uključiti sve skripte potrebne za reprodukciju pipeline-a (najmanje: moduli za rekonstrukciju, segmentaciju, lokalizaciju objekta, planiranje trajektorije, izvršenje UR naredbi).
- Modeli i težine korišteni u projektu (npr. YOLO/segmentation `.pt`) ili jasne upute gdje ih preuzeti.
- Konfiguracijske datoteke: `requirements.txt`, `config.py` (ili ekvivalentno), parametri segmentacije/filtra/trajektorije.
- Kalibracijske matrice i transformacije: npr. `camera_matrix.npy`, `dist_coeffs.npy`, `T_cam_from_tcp.npy`.
- Primjeri podataka / test scena koje ste koristili i `last_trajectory.json` (ako postoji) za brzu reprodukciju.

## 4) Upute za reproduciranje demonstracije
- Navedite minimalne korake koje ocjenjivač treba napraviti da reproducira pipeline (npr. setup virtual env, instalacija, kako pokrenuti glavnu skriptu i kojim redoslijedom):

```
python -m venv .venv
.venv\Scripts\activate    # Windows
pip install -r requirements.txt
python src/02_fruit_pick_and_place/main.py --run-demo --config config.yaml
```

- Ako je potrebna robotska oprema (UR robot), navedite koja je naredba za switch u simulaciju ili kako reproducirati bez fizičkog robota (npr. playback trajektorije iz `last_trajectory.json`).

## 5) Kriteriji ocjenjivanja / provjere
- Video jasno pokazuje sve tražene faze i nema ručne intervencije.
- Izvješće zadovoljava sve tražene sekcije i ima tražene grafove za analizu trajektorije.
- Kôd i konfiguracije omogućuju reproduciranje pipeline-a (ili s fizičkim robotom ili putem replaya/simulacije).

## 6) Dodatne napomene
- Ako nešto nije izvedivo na fizičkom robotu, jasno objasnite u izvješću zašto i priložite reproducirajući replay (log/trajektorija) i/ili simulaciju.
- Jasno označite sve datoteke koje su autorstva drugih (npr. pretrained modeli) i navedite licence/izvore.

---
Ako želite, mogu:
- dodati predložak `README_SUBMISSION.md` s koracima za slanje,
- generirati kontrolnu listu (checklist) u obliku `submission_checklist.txt` za datoteke koje trebate uploadati.
