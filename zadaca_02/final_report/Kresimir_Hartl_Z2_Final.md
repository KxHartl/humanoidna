# SVEUČILIŠTE U ZAGREBU
# FAKULTET STROJARSTVA I BRODOGRADNJE

<br><br><br><br>

# Fruit detection and pose estimation
### Instancna segmentacija 3D printanih modela voća primjenom YOLOv8 nano arhitekture

<br><br><br><br>

**Student:** Krešimir Hartl  
**Kolegij:** Humanoidna robotika  

<br><br><br><br>
**Zagreb, travanj 2026.**

---

## 1. UVOD
U okviru kolegija Humanoidna robotika, zadatak ovog rada bio je razviti sustav za detekciju i instancnu segmentaciju 3D printanih modela voća. U robotici se često susrećemo s potrebom za preciznim lociranjem objekata kako bi robotski manipulatori mogli sigurno interagirati s okolinom. Za ovaj zadatak odabrana je arhitektura **YOLOv8 nano (n-seg)** zbog svog izvrsnog omjera brzine obrade i preciznosti, što ju čini pogodnom za izvođenje u stvarnom vremenu na ugrađenim (embedded) računalnim sustavima humanoidnih robota.

## 2. OPIS SKUPA PODATAKA
Skup podataka nastao je kolaborativnim naporom studenata i merganjem individualnih setova. Proces prikupljanja podataka bio je podijeljen u nekoliko ključnih faza:
- **Snimanje i anotacija**: Svaki student je osigurao slike jedne specifične klase. Autor ovog rada prikupio je podatke za klasu **crvena jabuka**, snimajući objekte u uvjetima radnog stola ("desk") te u kontroliranim uvjetima unutar kutije ("box") radi veće raznolikosti osvjetljenja.
- **Standardizacija i Merganje**: Budući da su podaci stizali s različitih izvora, izvršen je proces re-mapiranja klasa kako bi se osigurala konzistentnost (npr. ujednačavanje naziva klasa u jedinstveni CSV format).
- **Podjela podataka (Data Splitting)**: Skup je podijeljen na **trening (70%)**, **validaciju (20%)** i **testiranje (10%)**. Ukupan broj slika nakon procesa ujednačavanja iznosi 771, od čega je 539 korišteno za treniranje.
- **Augmentacija**: Primijenjene su tehnike poput mozaika (spajanje 4 slike u jednu), nasumične rotacije i promjene zasićenosti boja, čime se umjetno povećala raznolikost skupa podataka i spriječila prenaučenost.

## 3. METODOLOGIJA I TRENIRANJE
Treniranje je provedeno pomoću **PyTorch** frameworka i **Ultralytics** biblioteke.
- **Hiperparametri**: Model je treniran kroz 100 epoha uz početni learning rate od 0.01. Korišten je optimizator sa SGD-om i momentumom od 0.937.
- **Finetuning**: Zadnjih 10 epoha treniranja provedeno je bez mozaik augmentacije. To omogućava modelu da se u završnoj fazi fokusira na precizne geometrijske karakteristike objekata, što rezultira oštrijim rubovima maski kod segmentacije.
- **Hardver**: Proces je ubrzan korištenjem NVIDIA GeForce RTX 4060 GPU procesora, čime je vrijeme treniranja svedeno na manje od 20 minuta.

## 4. REZULTATI I ANALIZA
Model je postigao visoku razinu točnosti na testnom skupu, s **mAP50 od 0.9428** za detekciju i **0.9332** za segmentaciju.

![Krivulja mAP](file:///d:/truenas_kresimir_share_cp/FSB/semestar_10/humanoidna/zadaca_02/final_report/assets/premium_map_curve.png)
*Slika 1. Konvergencija mAP50 metrike kroz 100 epoha (70/20/10 split).*

![Analiza gubitaka](file:///d:/truenas_kresimir_share_cp/FSB/semestar_10/humanoidna/zadaca_02/final_report/assets/premium_loss_curves.png)
*Slika 2. Trendovi gubitaka (Loss) tijekom procesa učenja.*

| Klasa | Slučajeva (Test) | mAP50 (Box) | mAP50 (Mask) |
| :--- | :--- | :--- | :--- |
| Crvena jabuka | 120 | 0.930 | 0.908 |
| Zelena jabuka | 108 | 0.912 | 0.893 |
| Limun | 48 | 0.915 | 0.887 |
| Banana | 52 | 0.876 | 0.835 |
| Orah | 44 | 0.957 | 0.952 |
| Naranča | 69 | 0.967 | 0.967 |

## 5. VIZUALIZACIJA REZULTATA
Model pokazuje robusnost u grupnim scenama i precizno ocrtava granice plodova čak i pri međusobnom preklapanju.

![Primjer 1 - Grupno](file:///d:/truenas_kresimir_share_cp/FSB/semestar_10/humanoidna/zadaca_02/runs/segment/output/predictions/exp/grupno_image_20260410_0009_png_png.rf.mX6PV07J4w1yCDpBv9Zd.jpg)
*Slika 3. Identifikacija i segmentacija plodova u kompleksnoj grupnoj sceni.*

![Primjer 2 - Crvena jabuka](file:///d:/truenas_kresimir_share_cp/FSB\semestar_10\humanoidna\zadaca_02\runs\segment\output\predictions\exp\jabuka_image_20260410_0015_png_png.rf.alC3Jw6RBh8F6qvVgl9X.jpg)
*Slika 4. Rezultat segmentacije na osobno prikupljenom setu (crvena jabuka).*

## 6. ZAKLJUČAK
Primjena YOLOv8 nano arhitekture pokazala se iznimno uspješnom za ovaj zadatak. Unatoč smanjenom broju parametara, nano model postiže vrhunske rezultate na specijaliziranim skupovima podataka kao što je ovaj, omogućujući implementaciju na sustave s ograničenim resursima bez gubitka funkcionalne točnosti.

## 7. LITERATURA
[1] J. Redmon, et al., "You Only Look Once: Unified, Real-Time Object Detection," CVPR, 2016.  
[2] Ultralytics, "YOLOv8 Segmentation Guide," [Online]. Available: https://docs.ultralytics.com.  
[3] G. Jocher, et al., "YOLO by Ultralytics," 2023.
