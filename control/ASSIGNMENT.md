# ZADACI VJEŽBE: Osnove upravljanja

## ZADATAK 1: Implementacija P regulatora, step referenca
*   **Opis zadatka:** Potrebno je prilagoditi postojeći simulacijski pipeline tako da robot prati step referencu u joint space-u.
*   **Referenca:** Početno $q_i = [0, 0, 0]$, željeno $q_f = [1, -0.8, 0.5]$. Vrijeme skoka $t_s = 1.0 s$.
*   **Regulator:** Proporcionalni regulator ($K_p = 80$ Nm/rad za svaki zglob).
*   **Ciljevi:** Spremiti podatke u `sim_data_zadatak1.npz`, vizualno prikazati rezultate, izračunati metrike (overshoot, settling time, steady-state error).
*   **Uputa:** Zaustaviti simulaciju kada prestane oscilirati.

## ZADATAK 2: Implementacija P regulatora, sinusna referenca
*   **Opis zadatka:** Prilagoditi pipeline tako da robot prati sinusoidnu referencu u joint space-u.
*   **Referenca:** $q_d(t) = q_c + A \sin(2\pi f t)$, gdje su $A = [0.5, 0.4, 0.3]$, $f = 0.5$ Hz, $q_c = [0, -0.3, 0.2]$.
*   **Regulator:** Proporcionalni regulator ($K_p = 80$ Nm/rad).
*   **Ciljevi:** Spremiti podatke u `sim_data_zadatak2.npz`, izračunati RMSE. Simulacija traje cca 10s.

## ZADATAK 3: Implementacija PD regulatora
*   **Opis zadatka:** Dodavanje derivacijskog člana za praćenje step reference.
*   **Regulator:** PD regulator. $K_p = 80$. Vrijednost $K_d$ odabrati eksperimentalno tako da se postigne manje oscilacija, manji overshoot te brže smirivanje. Moguće različite vrijednosti po zglobu.
*   **Ciljevi:** Izračunati metrike, vizualizirati, spremiti u `sim_data_zadatak3_step.npz`.

## ZADATAK 4: Implementacija PD regulatora uz disturbance (smetnju)
*   **Opis zadatka:** Uvođenje vanjske smetnje (disturbance) te analiziranje njenog utjecaja na PD regulator iz Zadatka 3.
*   **Regulator:** PD regulator. Ista pojačanja $K_p$ i $K_d$ kao u Z3.
*   **Smetnja:** Dodavanje smetnje na upravljački signal: $u_{total} = u + d$, gdje je $d = [5.0, 5.0, 5.0]$ Nm.
*   **Ciljevi:** Izračunati metrike, vizualizirati, spremiti u `sim_data_zadatak4_step.npz`.

## ZADATAK 5: Implementacija PID regulatora
*   **Opis zadatka:** Uklanjanje steady-state errora izazvanog smetnjom (Z4) dodavanjem integralnog člana.
*   **Regulator:** PID regulator. $K_p$ i $K_d$ iz Z4. Vrijednost $K_i$ odabrati eksperimentalno po zglobu da se ukloni stacionarna pogreška uz zadržavanje stabilnosti.
*   **Ciljevi:** Izračunati metrike, vizualizirati, spremiti u `sim_data_zadatak5_step.npz`.

## ZADATAK 6: PID u prisutnosti white noise-a
*   **Opis zadatka:** Dodavanje mjernog šuma na očitavanje pozicije te analiza utjecaja šuma na rad PID-a.
*   **Regulator:** PID regulator. Pojačanja mogu ostati ista ili se eksperimentalno ugoditi za bolji odziv pod šumom.
*   **Šum:** Gaussov white noise sa standardnom devijacijom $\sigma = 0.02$ dodaje se na izmjereni $q$.
*   **Ciljevi:** Izračunati metrike, vizualizirati, spremiti u `sim_data_zadatak6_step.npz`.
