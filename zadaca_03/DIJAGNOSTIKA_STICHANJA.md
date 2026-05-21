# Dijagnostika i Rješenje Problema sa Stichanjem (3-5 cm odmak)

Ako vidiš tri odvojena limuna (jedan za svaki pogled) koji su međusobno pomaknuti za nekoliko centimetara, to znači da je **matematički model transformacije ispravan, ali su podaci koji ulaze u njega (kalibracija ili konvencije) pogrešni.**

Evo popisa mogućih uzroka i kako ih fiksirati:

## 1. Pogrešna Hand-Eye kalibracija (`T_cam_from_tcp`)
Ovo je najčešći uzrok pomaka od 3-5 cm. Ako matrica koja povezuje kameru i vrh robota nije savršena, svaki pogled će "vidjeti" limun na drugom mjestu u prostoru baze.

**Kako provjeriti:**
*   Pogledaj datoteku `voce_zadaca/calibration/T_cam_from_tcp.npy`.
*   Jeste li radili kalibraciju s kvačicom (checkerboard)? Ako niste, ili ako je rađena površno, tu nastaje taj odmak.

**Rješenje:** 
*   Ponoviti Hand-Eye kalibraciju.
*   Privremeno rješenje: Ručno korigiraj translaciju u matrici ako znaš točan odmak.

## 2. Zamjena osi (Konvencije koordinatnih sustava)
Postoji velika šansa da su osi kamere i osi koje robot očekuje zamijenjene.
*   **Standard kamere (OpenCV/RealSense):** Z gleda naprijed, X desno, Y dolje.
*   **Standard robota (UR):** Često se koristi drugačija orijentacija.

Ako ti je npr. Y os u kodu usmjerena "dolje", a robot je vidi kao "gore", dobit ćeš duplicirane objekte.

**Test u `reconstruct_scene.py`:**
Pokušaj invertirati matricu kalibracije. U `reconstruct_scene.py` promijeni:
```python
# Umjesto invert_cam_tcp=False, probaj:
reconstruct_scene_from_pairs(..., invert_cam_tcp=True)
```

## 3. Redoslijed transformacija (TCP vs Base)
Matematika mora biti: `P_base = T_base_tcp * T_tcp_cam * P_cam`.
Ako su `T_base_tcp` i `T_tcp_cam` pomnoženi u krivom redoslijedu ili ako je jedna od njih transponirana, dobit ćeš pomak koji prati rotaciju robota.

**Provjera:**
U `reconstruct_scene.py`, linija:
```python
T_base_cam = T_base_tcp @ T_tcp_cam
```
Provjeri je li `T_tcp_cam` zapravo `T_cam_from_tcp` (pazi na "from" i "to"). Ako imaš matricu koja je "Camera u odnosu na TCP", a tvoj kod je tretira kao "TCP u odnosu na Cameru", dobit ćeš upravo tih par centimetara greške (dužina nosača kamere).

## 4. Problem s dubinom (RealSense Offset)
RealSense kamere imaju mali offset između RGB i Depth leće. Ako u `capture_scene.py` nije savršeno odrađen `align`, 3D točka neće odgovarati pixelu na slici.

**Provjera:**
Pogledaj `voce_zadaca/output/run_.../segment/view_00_overlay.png`. Ako maska (plava boja) ne prekriva limun savršeno, nego bježi par milimetara, to će se u 3D-u pretvoriti u centimetre.

---

### BRZI TEST ZA POPRAVAK:
U `/src_testing/reconstruct_scene.py` isprobaj ove tri varijante:

1.  **Varijanta A (Inverzija):** `T_base_cam = T_base_tcp @ np.linalg.inv(T_tcp_cam)`
2.  **Varijanta B (Zamjena):** `T_base_cam = T_tcp_cam @ T_base_tcp` (manje vjerojatno, ali testiraj)
3.  **Varijanta C (Ručni Offset):** Ako vidiš da je limun uvijek npr. 3cm "previsoko", dodaj `T_base_cam[2, 3] += 0.03`.

**Preporuka:** Najvjerojatnije je problem u **Hand-Eye matrici**. Ako možete, ponovite kalibraciju ili provjerite je li matrica u `.npy` datoteci ispravno spremljena (Row-major vs Column-major).
