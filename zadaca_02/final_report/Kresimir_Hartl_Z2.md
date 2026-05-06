# SVEUČILIŠTE U ZAGREBU
# FAKULTET STROJARSTVA I BRODOGRADNJE

<br><br><br><br>

# ZAVRŠNI RAD
### Instancna segmentacija 3D printanih modela voća primjenom YOLOv8 nano arhitekture

<br><br><br><br>

**Student:** Krešimir Hartl  
**Mentor:** Prof. dr. sc. Ime prezime  
**Kolegij:** Humanoidna robotika  

<br><br><br><br>
**Zagreb, travanj 2026.**

---

## 1. UVOD
U okviru kolegija Humanoidna robotika, zadatak ovog rada bio je razviti sustav za detekciju i instancnu segmentaciju 3D printanih modela voća. Sustav se temelji na konvolucijskoj neuronskoj mreži YOLO (You Only Look Once) u verziji 8, specifično nano varijanti optimiziranoj za rad u stvarnom vremenu. Instancna segmentacija predstavlja napredni zadatak računalnog vida koji kombinira detekciju objekata (pronalaženje okvira) i semantičku segmentaciju (precizno ocrtavanje granica svake pojedine instance).

## 2. OPIS SKUPA PODATAKA
Skup podataka nastao je kolaborativnim naporom studenata. Svaki student bio je zadužen za prikupljanje i anotaciju specifične klase voća.
- **Osobni doprinos**: Za potrebe ovog rada, osobno sam izvršio skeniranje klase **crvena jabuka**, što je uključivalo fotografiranje objekata na radnom stolu ("desk images") te unutar kontroliranih uvjeta ("box images").
- **Anotacija**: Označavanje je izvršeno pomoću alata Roboflow, koristeći poligonalne maske za instancnu segmentaciju.
- **Spajanje i augmentacija**: Individualni skupovi podataka su spojeni u jedinstvenu bazu koja sadrži 6 aktivnih klasa: crvena jabuka, zelena jabuka, limun, banana, orah i naranča. Primijenjene su augmentacije poput rotacije, šuma i promjene osvjetljenja kako bi se povećala robusnost modela.

## 3. METODOLOGIJA
Za treniranje je korišten model **YOLOv8n-seg**.
- **Parametri treniranja**: 
  - Broj epoha: 100
  - Rezolucija slika: 640x640 piksela
  - Batch size: 16
  - Hardver: Lokalni NVIDIA GeForce RTX 4060 Laptop GPU (8GB VRAM)
- **Podjela podataka**: Skup je podijeljen u omjeru 80% za trening, 10% za validaciju i 10% za testiranje.

## 4. REZULTATI I ANALIZA
Treniranje je uspješno završeno uz visoku točnost na testnom skupu podataka. 
Model je postigao **mAP50 od 0.9489** za detekciju (boxing) i **0.9462** za segmentaciju (masking).

Analizom krivulja učenja (Slika 1), vidljivo je da se mAP vrijednost stabilizirala nakon 80. epohe. Iako se box loss nastavio blago smanjivati, klasifikacijski gubitak na validacijskom skupu je dosegao plato, što ukazuje na to da je **100 epoha optimalno** za ovaj volumen podataka. Daljnje povećanje broja epoha vjerojatno ne bi donijelo značajne dobitke u točnosti, već bi moglo dovesti do prenaučenosti (overfitting).

![Krivulja mAP](file:///d:/truenas_kresimir_share_cp/FSB/semestar_10/humanoidna/zadaca_02/final_report/assets/premium_map_curve.png)
*Slika 1. Prikaz mAP50 metrike kroz 100 epoha treniranja za detekciju i segmentaciju.*

![Analiza gubitaka](file:///d:/truenas_kresimir_share_cp/FSB/semestar_10/humanoidna/zadaca_02/final_report/assets/premium_loss_curves.png)
*Slika 2. Analiza konvergencije gubitaka lokacije (Box) i klasifikacije (Cls).*

| Klasa | Slučajeva (Test) | mAP50 (Box) | mAP50 (Mask) |
| :--- | :--- | :--- | :--- |
| Crvena jabuka | 120 | 0.931 | 0.922 |
| Zelena jabuka | 108 | 0.955 | 0.954 |
| Limun | 48 | 0.983 | 0.954 |
| Banana | 52 | 0.938 | 0.878 |
| Orah | 44 | 0.989 | 0.989 |
| Naranča | 69 | 0.930 | 0.902 |

Model pokazuje iznimnu preciznost u prepoznavanju oraha i limuna, dok su kod banana uočena manja odstupanja u preciznosti maske zbog specifičnog izduženog oblika.

## 5. VIZUALIZACIJA REZULTATA
U nastavku su prikazani primjeri predikcija na testnom skupu koji potvrđuju sposobnost modela da precizno segmentira objekte čak i u grupnim scenama.

![Primjer 3 - Grupna scena](file:///d:/truenas_kresimir_share_cp/FSB/semestar_10/humanoidna/zadaca_02/runs/segment/output/predictions/exp/grupno_image_20260410_0009_png_png.rf.mX6PV07J4w1yCDpBv9Zd.jpg)
*Slika 3. Detekcija i segmentacija više različitih plodova u jednoj sceni.*

![Primjer 4 - Crvena jabuka](file:///d:/truenas_kresimir_share_cp/FSB\semestar_10\humanoidna\zadaca_02\runs\segment\output\predictions\exp\jabuka_image_20260410_0015_png_png.rf.alC3Jw6RBh8F6qvVgl9X.jpg)
*Slika 4. Precizna segmentacija crvenih jabuka (osobno prikupljen skup).*

## 6. ZAKLJUČAK
Implementirani sustav temeljen na YOLOv8 nano arhitekturi pokazao je izvrsne performanse u zadatku instancne segmentacije 3D printanih modela voća. Unatoč maloj veličini mreže, model postiže visoku mAP vrijednost (>0.94), što ga čini idealnim za primjenu u robotici gdje je potrebna brza obrada slika na rubnim uređajima. Budući rad mogao bi se fokusirati na testiranje modela u uvjetima ekstremne okluzije te proširenje skupa podataka na stvarne plodove.

## 7. LITERATURA
[1] J. Redmon, S. Divvala, R. Girshick, and A. Farhadi, "You Only Look Once: Unified, Real-Time Object Detection," CVPR, 2016.  
[2] Ultralytics, "YOLOv8 Docs," [Online]. Available: https://docs.ultralytics.com.  
[3] G. Jocher, A. Chaurasia, and J. Qiu, "YOLO by Ultralytics," 2023.
