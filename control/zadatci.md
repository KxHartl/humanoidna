# Zadatci - Upravljanje i interakcija

U ovom dokumentu popisani su svi zadatci izvučeni iz **nove** verzije PDF prezentacije `04_Upravljanje_interakcija.pdf` u njihovom izvornom obliku te razrađeni koraci (TODO liste) što točno treba napraviti za njihovo uspješno rješavanje.

---

## ZADATAK 1: Task Space upravljanje — Franka Emika Panda

### Izvorni oblik
> **Opis zadatka:** 
> • Zadajemo željenu trajektoriju end-effektora kao kružnicu u YZ ravnini (task space), a referenca pozicije prati sljedeći zakon:
> $x_d(t) = x_0 + [0, \ r \cos(\omega t)-r, \ r \sin(\omega t)]^T$, r = 0.08 m, $\omega = 3.5 \ rad/s$
> • Zadajemo željenu trajektoriju end-effektora kao kružnicu u YZ ravnini (task space), a referenca brzine prati sljedeći zakon:
> $\dot{x}_d(t) = [0, \ -r\omega \sin(\omega t), \ r\omega \cos(\omega t)]^T$
> • Orijentacija end-effektora drži se konstantnom: $R_d = R_0 = const.$, $\omega_d = 0$
> 
> **Pogreška u prostoru zadatka:**
> • Pozicijska pogreška: $e_{pos} = x_d - x$
> • Rotacijska pogreška: $e_r = \frac{1}{2} (R_d R^T - R R_d^T)^\vee$ (iz antisimetrične razlike rotacijskih matrica)
> *Napomena: operator $(\cdot)^\vee$ izvlači vektor iz antisimetrične matrice.*
>
> **Upravljački zakon u task space-u:**
> • Željene brzine u task spaceu (6D):
> $\begin{bmatrix} v_{cmd} \\ \omega_{cmd} \end{bmatrix} = \begin{bmatrix} \dot{x}_d + K_{pos} \cdot e_{pos} \\ \omega_d + 2K_{rot} \cdot e_{rot} \end{bmatrix}$ , $K_{pos} = 15, K_{rot} = 10$
>
> **Preslikavanje u prostor zglobova:**
> • Jacobian matrica $J \in \mathbb{R}^{6 \times 7}$ dijeli se na translacijski i rotacijski dio: $J = \begin{bmatrix} J_p \\ J_R \end{bmatrix}$
> • Željene brzine zglobova dobivaju se pseudoinverzom:
> $\dot{q}_{cmd} = J^\dagger \begin{bmatrix} v_{cmd} \\ \omega_{cmd} \end{bmatrix}$
> $J^\dagger = J^T (J J^T + \lambda^2 I)^{-1}$ , $\lambda = 0.03$
>
> **Integracija i pozicijska naredba:**
> • Referentni kutovi zglobova se iz brzina dobivaju Eulerovom integracijom: 
> $q_{ref}(t + \Delta t) = q_{ref}(t) + \dot{q}_{cmd} \cdot \Delta t$
> • Referentni kutovi zglobova šalju se kao pozicijska naredba PD aktuatorima: $u = q_{ref}$

### Što treba napraviti (TODO)
- [ ] Definirati konstante zadatka ($r = 0.08$, $\omega = 3.5$, $K_{pos} = 15$, $K_{rot} = 10$, $\lambda = 0.03$).
- [ ] Zabilježiti početnu rotaciju end-effektora ($R_0$) te postaviti referentnu rotaciju $R_d = R_0$.
- [ ] U simulacijskoj petlji izračunati željenu poziciju $x_d(t)$ te željenu linearnu brzinu $\dot{x}_d(t)$ kružne putanje u YZ ravnini.
- [ ] Izračunati pozicijsku grešku $e_{pos}$ i rotacijsku grešku $e_r$.
- [ ] Definirati upravljački zakon i izračunati $v_{cmd}$ i $\omega_{cmd}$ komponente.
- [ ] Dobaviti Jacobian matricu $J$ iz MuJoCo simulacije te implementirati Damped Least Squares (DLS) pseudoinverz ($J^\dagger$).
- [ ] Pomnožiti pseudoinverz s task-space brzinom kako bi se dobile željene brzine zglobova $\dot{q}_{cmd}$.
- [ ] Eulerovom numeričkom integracijom pomaknuti trenutni referentni kut zgloba ($q_{ref}$) primjenom izračunate brzine.
- [ ] Poslati dobiveni niz kutova kao upravljački signal (data.ctrl) aktuatorima.

---

## ZADATAK 2: Vizualizacija podataka

### Izvorni oblik
> • Nakon provedene simulacije task space upravljanja, logirali ste signale u datoteku "data/sim_data_task_space.npz"
> • Potrebno je napisati ‘.py’ skriptu kojom ćete vizualizirati:
> 1. Praćenje pozicije - $q_{ref}$ vs $q_{meas}$
> 2. Praćenje brzine - $\dot{q}_{ref}$ vs $\dot{q}_{meas}$
> 3. Upravljački signal - $u(t)$

### Što treba napraviti (TODO)
- [ ] Napisati novu Python skriptu koja učitava `data/sim_data_task_space.npz`.
- [ ] Konstruirati grafove na kojima se uspoređuje $q_{ref}$ (naredba/referenca) te $q_{meas}$ (mjerenje iz MuJoCo) po zglobovima kroz vrijeme.
- [ ] Prikazati brzinu zglobova te ju usporediti sa željenom brzinom dobivenom preko preslikavanja.
- [ ] Prikazati iznos upravljačkog signala $u(t)$ po svim kontroliranim zglobovima manipulatora.

---

## ZADATAK 3: Impedance upravljanje — Franka Emika Panda

### Izvorni oblik
> **Motivacija i osnovna ideja:**
> Robot se ponaša kao virtualni sustav masa-prigušivač-opruga:
> $M_d \ddot{e} + D_d \dot{e} + K_d e = F_{ext}$
> Parametri iznose:
> • $M_d = diag(0.5, 0.5, 0.5) \ kg$
> • $D_d = diag(80, 80, 80) \ Ns/m$
> • $K_d = diag(400, 400, 400) \ N/m$
> • $e = x_d - x$ (pozicijska greška)
> 
> **Referenca i scena:**
> • Robot se spušta prema stolu po minimum jerk trajektoriji duž Z osi
> • $x_{target}$ definiran je prema visini gornje plohe stola, $x_{target} = 0.22m$
> • Orijentacija EEF drži se konstantnom - $R_d = R_0 = const.$
> $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5, \tau = \frac{t}{T}, \ T = 6s$
> $x_d(t) = x_0 + (x_{target} - x_0) \cdot s(t/T)$
> 
> **Upravljački zakon:**
> $\ddot{x}_{cmd} = \ddot{x}_d + M_d^{-1} (D_d \dot{e} + K_d e + F_{ext})$
> Vanjska sila čita se iz simulatora: `F_ext = data.cfrc_ext[hand][3:6]`
>
> **Preslikavanje u prostor zglobova:**
> $\tau_{task} = M(q) \cdot J_p^\dagger \cdot \ddot{x}_{cmd}$
> $J_p^\dagger = J_p^T (J_p J_p^T + \lambda^2 I)^{-1}, \lambda = 0.01$
> *Napomena: $M(q) \in \mathbb{R}^{7 \times 7}$ je matrica inercije robota.*
> 
> **Null-space projektor:**
> $N = I - J_p^\dagger J_p$
> $\tau_{rot} = N (K_{p,rot}(q_{init} - q) - K_{d,rot} \dot{q})$
> $K_{p,rot} = 300, K_{d,rot} = 20$
>
> **Ukupna naredba momenta:**
> $\tau_{cmd} = \tau_{task} + \tau_{rot} + \tau_{grav}$
> Upisuje se u `data.qfrc_applied`!
>
> **Procjena kontaktne sile:**
> $\hat{F}_c = J_p^{\dagger T} (\tau_{cmd} - \tau_{grav})$

### Što treba napraviti (TODO)
- [ ] Definirati matrice zadatka $M_d$, $D_d$, i $K_d$.
- [ ] Implementirati $s(\tau)$ funkciju minimum jerk trajektorije.
- [ ] U simulacijskoj petlji generirati skalirani ciljni položaj i njegove derivacije ($\ddot{x}_d$ pretpostavljamo ovisi o derivaciji $s$).
- [ ] Očitati mjerenu vanjsku silu `F_ext = data.cfrc_ext[hand_id][3:6]`.
- [ ] Izračunati $\ddot{x}_{cmd}$ na bazi razlike trenutne ($x$) i željene ($x_d$) pozicije ($e$) i brzine ($\dot{e}$).
- [ ] Dobaviti translacijski Jacobian ($J_p$) i matricu inercije ($M(q)$).
- [ ] Implementirati pseudoinverz sa $\lambda = 0.01$.
- [ ] Izračunati $\tau_{task}$.
- [ ] Izračunati null-space projektor $N$ i $\tau_{rot}$ kako bi se kompenzirala redundantnost (sa zadanim $K_{p,rot}$ i $K_{d,rot}$).
- [ ] Dohvatiti ili izračunati vektor gravitacijske kompenzacije ($\tau_{grav}$) i definirati cjeloviti željeni moment $\tau_{cmd}$.
- [ ] Postaviti moment izravno u varijablu `data.qfrc_applied`.
- [ ] Izračunati procjenu sile pomoću transponiranog pseudoinverza Jacobiana $\hat{F}_c$.
- [ ] Logirati sve varijable u datoteku `sim_data_impedance.npz`.

---

## ZADATAK 4: Vizualizacija i analiza

### Izvorni oblik
> • Nakon provedene simulacije task space upravljanja, logirali ste signale u datoteku "data/sim_data_impedance.npz"
> • Potrebno je napisati ‘.py’ skriptu kojom ćete vizualizirati:
> 1. Praćenje pozicije - $q_{ref}$ vs $q_{meas}$
> 2. Sila eksterna - $F_{ext}(t)$
> 3. Procijenjena sila iz momenta - $\hat{F}_{est}(t)$
> 4. Momenti u joint space-u za sve zglobove

### Što treba napraviti (TODO)
- [ ] Napisati Python skriptu za ispis podataka u `data/sim_data_impedance.npz`.
- [ ] Napraviti linijski subplot ili više grafova na kojima se vidi odstupanje robota kad dođe u kontakt sa stolom.
- [ ] Iscrtati $F_{ext}(t)$ x,y,z komponente mjerene eksterne sile.
- [ ] Iscrtati $\hat{F}_{est}(t)$ i provjeriti poklapa li se procjena silama koje stvarno djeluju po senzorima simulatora.
- [ ] Prikazati raspodjelu momenata po svim zglobovima ($u(t)$) uzrokovanu impedance controlom i null-space kompenzacijom.
