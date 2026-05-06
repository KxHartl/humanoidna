---
university: "SVEUČILIŠTE U ZAGREBU"
faculty: "FAKULTET STROJARSTVA I BRODOGRADNJE"
course: "Humanoidna robotika"
author: "Krešimir Hartl"
title: "Detekcija voća i procjena poze primjenom dubokog učenja i 3D vizije"
location_date: "Zagreb, 2026."
---

# POPIS SLIKA

# POPIS TABLICA

# UVOD
U ovom seminaru obrađuje se implementacija sustava za detekciju i procjenu 3D poze objekata (voća) u prostoru, što je ključan korak u razvoju sustava za robotsku manipulaciju i vizualnu percepciju humanoidnih robota. Rad se temelji na izradi vlastitog skupa podataka, obučavanju modela dubokog učenja za segmentaciju instanci te obradi trodimenzionalnih oblaka točaka (engl. point cloud) dobivenih pomoću RGB-D senzora. Cilj je bio osposobiti sustav za prepoznavanje različitih vrsta voća te točno određivanje njihovog težišta (centroida) u složenim scenama s višestrukim objektima. Uspješno rješavanje ovog problema omogućuje robotu interakciju s okolinom, kao što je prepoznavanje i hvatanje željenog objekta iz posude.

# PRIKUPLJANJE I PRIPREMA SKUPA PODATAKA

## Postupak prikupljanja slika
Prvi korak u razvoju algoritma dubokog učenja bila je izrada prilagođenog skupa podataka koji reprezentira stvarne uvjete u okolini robota. Korišteno je 6 klasa voća (npr. crvena jabuka, zelena jabuka, limun, banana, avokado, orah, kruška - u obliku 3D printanih modela). Za svaku klasu prikupljeno je između 100 i 120 fotografija pojedinačnih plodova, pri čemu se vodilo računa o varijaciji kuta snimanja, mjerila, osvjetljenja te pojavi djelomičnih okluzija.

Nadalje, snimljeno je i 80 do 120 slika složenih scena u kojima se više komada različitog voća nalazi u posudama ili na stolu. Na ovim slikama voće se preklapa, čime se simuliraju stvarni uvjeti s kojima se robot susreće pri zadatku sortiranja ili odabira.

## Označavanje i integracija podataka
Nakon prikupljanja slika, izvedeno je označavanje (anotacija) na razini piksela, odnosno primijenjena je metoda segmentacije instanci (engl. instance segmentation). Prije početka označavanja, kreirana je zajednička `.csv` datoteka kojom je standardizirano nazivlje klasa. Svi sudionici u projektu su nezavisno označavali svoj podskup slika, bez primjene prethodne augmentacije. 

Nakon završene anotacije, svi su pojedinačni skupovi podataka integrirani u jedan centralni skup unutar platforme Roboflow. Tek je na spojenom i usklađenom skupu podataka izvedena augmentacija slika, čime je dodatno povećana robusnost konačnog dataseta namijenjenog treniranju konvolucijskih neuronskih mreža.

<img src="slike/dataset_primjer.png" width="100%" alt="Primjer skupa podataka i okluzija" />
*Slika 1. Prikaz voća u složenoj sceni i primjeri okluzija*

| Parametar | Vrijednost |
| :--- | :--- |
| Broj klasa voća | 6 |
| Slika po klasi (pojedinačne) | 100 - 120 |
| Slika složenih scena | 80 - 120 |
| Tip anotacije | Segmentacija instanci |
*Tablica 1. Parametri prikupljenog skupa podataka*

# TRENING MODELA I DETEKCIJA
Za zadatak detekcije i segmentacije voća korišten je model iz YOLO arhitekture (YOLO varijanta prilagođena za segmentaciju). Spojeni i augmentirani skup podataka s Roboflow platforme preuzet je i korišten za treniranje. Model je evaluiran i optimiziran kako bi s visokom pouzdanošću detektirao maske pojedinačnih plodova voća, čak i kada su oni djelomično prekriveni drugim objektima. Dobiveni model predstavlja osnovu za daljnje mapiranje slikovnih informacija u trodimenzionalni prostor.

# IZDVAJANJE 3D ZNAČAJKI I PROCJENA POZE

## Mapiranje i ekstrakcija oblaka točaka
Nakon uspješne primjene obučenog 2D modela za segmentaciju instanci na RGB slikama iz složenih scena (npr. posuda s preklapajućim voćem), uslijedio je zadatak određivanja trodimenzionalne pozicije svake instance. Korištenjem dubinskih (Depth) podataka i intrinzičnih parametara kamere, izvršeno je RGB-D poravnanje. Svaki piksel iz 2D maske detektiranog objekta projiciran je u 3D prostor, čime je dobiven izolirani oblak točaka (engl. point cloud) za svaku pojedinu instancu voća u sceni.

## Određivanje centroida i usporedba metoda
Za svaki ekstrahirani oblak točaka voća, izvršena je procjena 3D centroida (težišta objekta). Algoritam za izračun centroida implementiran je korištenjem dvije različite metode:
1. **Pristup fitingom geometrijskog modela (Geometric model fitting):** Korištenje algoritama za prilagodbu aproksimativnih geometrijskih oblika (poput sfere ili elipsoida) na oblak točaka i pronalazak središta tako aproksimiranog tijela.
2. **Ekstrakcija 3D granične kutije (3D Bounding Box):** Određivanje najmanjeg kvadra koji obuhvaća sve točke instance te izračun njegovog geometrijskog središta.

Obje metode su uspoređene s obzirom na točnost pozicioniranja u uvjetima kada vidljivi oblak točaka predstavlja samo prednju plohu voća (zbog okluzije i kuta kamere). Rezultati su detaljno vizualizirani, a svaki student je analizu i evaluaciju odradio isključivo na klasi voća koja mu je dodijeljena.

<img src="slike/3d_poza_primjer.png" width="100%" alt="Procjena poze 3D modelom" />
*Slika 2. Vizualizacija oblaka točaka voća s ucrtanim procijenjenim težištem*

# ZAKLJUČAK
U ovom seminaru prikazan je cjelovit sustav za detekciju, segmentaciju i 3D lokalizaciju objekata u prostoru. Kroz zadatke prikupljanja prilagođenog skupa podataka, obučavanja modela i 3D obrade oblaka točaka demonstriran je kompletan inženjerski proces vizualne percepcije za robote. Rezultati primjene različitih metoda za procjenu težišta ukazuju na važnost odabira prikladnog algoritma s obzirom na nesavršenost i nepotpunost senzorskih očitavanja u stvarnom svijetu. Implementirani pipeline direktno doprinosi mogućnostima humanoidnih robota za složene zadatke manipulacije i samostalne interakcije s okolinom.

# LITERATURA
1. Fakultet strojarstva i brodogradnje (2026). Materijali s predavanja i vježbi iz kolegija Humanoidna robotika.
2. Roboflow (2026). Roboflow Documentation and Best Practices for Dataset Management.
