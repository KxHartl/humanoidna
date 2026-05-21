# Zadaci za rješavanje problema s Pick-and-Place aplikacijom

- `[x]` **Segmentacija i filtracija (PCL)**
  - Otkriti zašto se izdvaja previše šuma ili krivi klasteri -> Riješeno! Napravljen je `tune_perception.py` alat za Grid Search koji je uspješno pronašao optimalne parametre na temelju metrike preklapanja.
  - Podesiti parametre `RANSAC` ravnine i `DBSCAN` klasteriranja -> Riješeno automatskim pretraživanjem.

- `[/]` **Klasifikacija i određivanje objekata (YOLO)**
  - Otkriti zašto YOLO pronalazi 0 objekata na spremljenoj slici -> Integrirana je robusna logika iz Zadaće 2 (projekcija svih točaka klastera na 2D YOLO pixel-masku).
  - Provjeriti sprema li se RGB slika iz RealSense kamere ispravno -> Potvrđeno, skripta sada uspješno prepoznaje voće (npr. u testu 15/16/17 YOLO je pronašao do 3 objekta po sceni!).
  - Podesiti threshold za YOLO ako su detekcije preslabe -> Ugađanje je u tijeku kroz Grid Search.

- `[ ]` **Kontrola robota (UR5e)**
  - Dijagnosticirati zašto se stvarni robot fizički ne pomiče unatoč tome što skripta pošalje naredbu na port 30003 bez greške.
  - Provjeriti mora li robot biti u izričitom *Remote Control* modu na Teach Pendantu.
  - Otkriti blokira li možda neka sigurnosna granica (safety plane) izvršenje, jer UR često odbaci skripte ako je zadana točka van sigurnih granica ili bi uzrokovala koliziju.
  - Riješiti gripper (konfiguracija izlaza DO4 i DO5).
