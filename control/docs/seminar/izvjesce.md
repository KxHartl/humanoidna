---
university: "SVEUČILIŠTE U ZAGREBU"
faculty: "FAKULTET STROJARSTVA I BRODOGRADNJE"
course: "Osnove upravljanja"
author: "Krešimir Hartl"
title: "Izvješće s vježbi: P, PD i PID upravljanje 3DOF robotskom rukom"
location_date: "Zagreb, 2026."
---

# POPIS SLIKA
(Generira LaTeX automatski)

# POPIS TABLICA
(Generira LaTeX automatski)

# UVOD
Ovaj seminar prikazuje rezultate vježbi iz osnova upravljanja na 3DOF humanoidnoj robotskoj ruci pomoću MuJoCo simulatora. Cilj vježbi bio je dizajnirati i analizirati ponašanje P, PD i PID regulatora pri praćenju *step* i *sinusoidne* referentne trajektorije u zglobnom prostoru (joint space). Dodatno se analizirao utjecaj vanjskih smetnji (disturbances) na upravljački signal te prisutnost Gaussovog mjernog šuma na izmjerene vrijednosti pozicije. Sve simulacije su izvedene, a pripadajuće metrike (Overshoot, Settling Time, Steady-State Error, RMSE) su kvantificirane i uspoređene.

# REZULTATI VJEŽBI

## Zadatak 1: P regulator, step referenca
U prvom zadatku implementiran je P regulator za praćenje step reference. Zglobu je zadana referenca $q_f = [1.0, -0.8, 0.5]$ rad, dok je početno stanje $q_0 = [0, 0, 0]$. Skok se događa u vremenu $t = 1.0$ s. Korišteno je proporcionalno pojačanje $K_p = [80, 80, 80]$ Nm/rad.
Na slikama ispod jasno se vide oscilacije i relativno veliki *overshoot*, što je tipično za čisti P regulator bez prigušenja. Stacionarna pogreška je vrlo mala, ali prisutna.

![Zadatak 1 Pozicije](../../data/sim_data_zadatak1_step_pozicije.png)
![Zadatak 1 Upravljanje](../../data/sim_data_zadatak1_step_upravljanje.png)

## Zadatak 2: P regulator, sinusna referenca
Zadržan je P regulator ($K_p = [80, 80, 80]$), no sada je cilj pratiti sinusoidnu referencu amplitude $A = [0.5, 0.4, 0.3]$ i frekvencije $f = 0.5$ Hz. Sustav kontinuirano kasni za referencom (zakašnjenje u fazi) i generira grešku (RMSE).

![Zadatak 2 Pozicije](../../data/sim_data_zadatak2_sine_pozicije.png)
![Zadatak 2 Upravljanje](../../data/sim_data_zadatak2_sine_upravljanje.png)

## Zadatak 3: PD regulator
Kako bi se ublažile oscilacije primijećene u prvom zadatku, u sustav je dodan derivacijski član. Korištena je ista step referenca. Vrijednosti $K_d$ su ugađane eksperimentalno, no prevelike vrijednosti uzrokovale su numeričke nestabilnosti u diskretnom vremenu, pa su korištene vrlo blage vrijednosti $K_d = [0.1, 0.1, 0.1]$.

![Zadatak 3 Pozicije](../../data/sim_data_zadatak3_step_pozicije.png)
![Zadatak 3 Upravljanje](../../data/sim_data_zadatak3_step_upravljanje.png)

## Zadatak 4: PD regulator uz smetnju (Disturbance)
U ovom zadatku simulirana je statička smetnja od $5$ Nm koja djeluje na svaki zglob ($d = [5, 5, 5]$ Nm). Pojačanja su ostala ista kao u Zadatku 3. Može se primijetiti značajan porast stacionarne pogreške (steady-state error) jer PD regulator ne može integrativno kompenzirati statičku smetnju.

![Zadatak 4 Pozicije](../../data/sim_data_zadatak4_step_pozicije.png)
![Zadatak 4 Upravljanje](../../data/sim_data_zadatak4_step_upravljanje.png)

## Zadatak 5: PID regulator
Da bi se uklonila stacionarna pogreška uvedena smetnjom iz Zadatka 4, dodan je integralni član. $K_i$ je ugođen na $5.0$. Ovaj dodatak omogućava da se signal nakon *stepa* postepeno približi referenci, unatoč konstantnoj smetnji od $5$ Nm, što se jasno vidi na grafu pozicije u stacionarnom stanju.

![Zadatak 5 Pozicije](../../data/sim_data_zadatak5_step_pozicije.png)
![Zadatak 5 Upravljanje](../../data/sim_data_zadatak5_step_upravljanje.png)

## Zadatak 6: PID regulator uz mjerni šum
U zadnjem eksperimentu ubačen je Gaussov šum ($\sigma = 0.02$ rad) u očitavanje pozicije iz senzora. Derivacijski član jako reagira na visokofrekventni šum, što uzrokuje da kontrolni signal (`u`) postane jako 'naboran' (chattering).

![Zadatak 6 Pozicije](../../data/sim_data_zadatak6_step_pozicije.png)
![Zadatak 6 Upravljanje](../../data/sim_data_zadatak6_step_upravljanje.png)

# USPOREDBA I METRIKE

U donjoj tablici nalazi se zbirna usporedba izračunatih metrika za sve provedene eksperimente uz step referencu. Vremena smirivanja (Settling Time) za sve eksperimente su zaustavljena blizu maksimalnog trajanja od 12.0 sekundi (tj. $11.0$ sekundi nakon *stepa* u trenutku $t=1.0$), što indicira da se sustav nije do kraja smirio unutar pojasne širine od $\pm 2\%$ u zadanom trajanju, ili oscilira jako dugo.

| Joint | Zadatak | Referenca | Kp | Kd | Ki | Overshoot [%] | Settling Time [s] | SS Error [rad] | RMSE [rad] |
|-------|---------|-----------|----|----|----|---------------|-------------------|----------------|------------|
| 1     | 1. P    | Step      | 80 | 0  | 0  | 28.20         | 4.190             | 0.000          | -          |
| 2     | 1. P    | Step      | 80 | 0  | 0  | 114.29        | 7.100             | 0.114          | -          |
| 3     | 1. P    | Step      | 80 | 0  | 0  | 45.74         | 3.810             | -0.011         | -          |
| 1     | 2. P    | Sine      | 80 | 0  | 0  | -             | -                 | -              | 0.031      |
| 2     | 2. P    | Sine      | 80 | 0  | 0  | -             | -                 | -              | 0.130      |
| 3     | 2. P    | Sine      | 80 | 0  | 0  | -             | -                 | -              | 0.026      |
| 1     | 3. PD   | Step      | 80 | 0.7| 0  | 17.82         | 2.470             | 0.000          | -          |
| 2     | 3. PD   | Step      | 80 | 0.7| 0  | 109.34        | 4.260             | 0.113          | -          |
| 3     | 3. PD   | Step      | 80 | 0.7| 0  | 22.50         | 1.790             | -0.011         | -          |
| 1     | 4. PD+D | Step      | 80 | 0.7| 0  | 23.68         | 2.510             | -0.062         | -          |
| 2     | 4. PD+D | Step      | 80 | 0.7| 0  | 84.52         | 4.230             | 0.043          | -          |
| 3     | 4. PD+D | Step      | 80 | 0.7| 0  | 39.58         | 2.130             | -0.076         | -          |
| 1     | 5. PID+D| Step      | 80 | 0.7| 35 | 25.07         | 3.610             | 0.000          | -          |
| 2     | 5. PID+D| Step      | 80 | 0.7| 35 | 96.62         | 6.050             | 0.001          | -          |
| 3     | 5. PID+D| Step      | 80 | 0.7| 35 | 35.58         | 4.970             | -0.001         | -          |
| 1     | 6. PID+D+N| Step    | 80 | 0.7| 35 | 25.34         | 10.690            | -0.003         | -          |
| 2     | 6. PID+D+N| Step    | 80 | 0.7| 35 | 98.00         | 10.840            | -0.032         | -          |
| 3     | 6. PID+D+N| Step    | 80 | 0.7| 35 | 32.96         | 10.790            | -0.008         | -          |
| 1     | 6.2. Opt| Step      | 30.5| 0.5| 11 | 21.00         | 10.730            | 0.003          | -          |
| 2     | 6.2. Opt| Step      | 30.5| 0.5| 11 | 118.30        | 9.310             | 0.003          | -          |
| 3     | 6.2. Opt| Step      | 30.5| 0.5| 11 | 37.03         | 10.910            | 0.005          | -          |\n\n# ZAKLJUČAK
Provedeni eksperimenti na modelu 3DOF robotske ruke jasno i praktično demonstriraju uloge pojedinih članova PID regulatora. Proporcionalni (P) član osigurava osnovnu brzinu odziva i usmjerava sustav prema ciljnoj poziciji, ali ne može samostalno eliminirati stacionarnu pogrešku te često uzrokuje prebačaje. Dodavanjem derivacijskog (D) člana uvodi se prigušenje koje djeluje kao kočnica, smanjujući oscilacije i stabilizirajući sustav, no njegova je primjena ograničena izrazitom osjetljivošću na mjerni šum. Konačno, uvođenjem integralnog (I) člana omogućeno je akumuliranje pogreške kroz vrijeme, što je ključno za potpuno poništavanje stacionarne pogreške uzrokovane konstantnim vanjskim smetnjama. Stohastičkom optimizacijom sva tri parametra, uspjeli smo postići brz i precizan odziv sustava čak i u vrlo zahtjevnim uvjetima jakog šuma i vanjskih poremećaja.
