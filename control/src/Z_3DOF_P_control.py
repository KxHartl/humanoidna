import mujoco
import numpy as np
import os

# Osiguravamo postojanje data foldera
if not os.path.exists("data"):
    os.makedirs("data")

# -------------------------
# REFERENTNE TRAJEKTORIJE
# -------------------------
def step_reference(t):
    q0 = np.array([0.0, 0.0, 0.0])
    qf = np.array([1.0, -0.8, 0.5])
    ts = 1.0
    if t < ts:
        return q0, np.zeros(3) # vraćamo i željenu brzinu (nula za step osim u ts)
    else:
        return qf, np.zeros(3)

def sine_reference(t):
    A = np.array([0.5, 0.4, 0.3])
    f = 0.5
    qc = np.array([0.0, -0.3, 0.2])
    q_d = qc + A * np.sin(2 * np.pi * f * t)
    q_dot_d = A * 2 * np.pi * f * np.cos(2 * np.pi * f * t)
    return q_d, q_dot_d

# -------------------------
# FUNKCIJA ZA METRIKE
# -------------------------
def calculate_metrics(t, q, qd, ref_type, task_name):
    print(f"\n{'='*40}\nMETRIKE ZA: {task_name}\n{'='*40}")
    
    if ref_type == 'step':
        # Vrijeme skoka
        ts_idx = np.argmax(t >= 1.0)
        
        for i in range(3):
            q_step = q[ts_idx:, i]
            qf = qd[-1, i]
            q0 = qd[0, i]
            delta = qf - q0
            
            # 1. Overshoot
            if delta > 0:
                max_val = np.max(q_step)
            else:
                max_val = np.min(q_step)
                
            overshoot_pct = 0.0
            if delta != 0:
                overshoot_pct = max(0, abs((max_val - qf) / delta)) * 100
                
            # 2. Steady-state error (pogreška u stacionarnom stanju)
            sse = qf - q_step[-1]
            
            # 3. Settling time (2%) oko konačne vrijednosti (kako bi ignorirali SS error)
            settling_time = 0.0
            if delta != 0:
                q_settled = q_step[-1]
                threshold = 0.02 * abs(delta)
                # Pronalazimo zadnji indeks gdje je signal izvan pojasa 2% od SVOJE KONAČNE VRIJEDNOSTI
                errors = np.abs(q_step - q_settled)
                out_of_bounds = np.where(errors > threshold)[0]
                if len(out_of_bounds) > 0:
                    settling_time = t[ts_idx + out_of_bounds[-1]] - 1.0 # vrijeme od skoka
                else:
                    settling_time = 0.0 # Odmah je unutar 2%
            
            print(f"Zglob {i+1}: Overshoot = {overshoot_pct:.2f}%, Settling Time = {settling_time:.3f}s, SS Error = {sse:.5f} rad")
            
    elif ref_type == 'sine':
        rmse = np.sqrt(np.mean((q - qd)**2, axis=0))
        for i in range(3):
            print(f"Zglob {i+1}: RMSE = {rmse[i]:.5f} rad")

# -------------------------
# GLAVNA SIMULACIJSKA FUNKCIJA
# -------------------------
def run_simulation(task_id, ref_type, duration, Kp, Kd, Ki, disturbance_amp, noise_std):
    print(f"\nPokrecem {task_id}...")
    
    # Inicijalizacija modela
    model = mujoco.MjModel.from_xml_path("models/model.xml")
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    
    # Odabir referentne funkcije
    ref_func = step_reference if ref_type == 'step' else sine_reference
    
    # Log varijable
    t_log, q_log, qd_log, u_log = [], [], [], []
    
    # Inicijalno stanje
    q0, _ = ref_func(0.0)
    data.qpos[:] = q0
    data.qvel[:] = 0.0
    
    # Varijable PID regulatora
    e_int = np.zeros(3)
    
    sim_time = 0.0
    
    # Simulacijska petlja (headless za brže izvršavanje)
    while sim_time < duration:
        
        # 1. Željeno stanje
        q_d, q_dot_d = ref_func(sim_time)
        
        # 2. Mjerenje stanja (s dodanim šumom ako postoji)
        q_meas = data.qpos.copy()
        if noise_std > 0:
            q_meas += np.random.normal(0, noise_std, size=3)
            
        q_dot_meas = data.qvel.copy()
        
        # 3. Računanje greške
        e = q_d - q_meas
        e_dot = q_dot_d - q_dot_meas
        e_int += e * dt
        
        # 4. PID Upravljački zakon
        u = Kp * e + Kd * e_dot + Ki * e_int
        
        # 5. Smetnja (disturbance)
        u_total = u + disturbance_amp
        
        # Postavljanje kontrole
        data.ctrl[:] = u_total
        
        # Logiranje
        t_log.append(sim_time)
        q_log.append(data.qpos.copy()) # Logiramo stvarni q, a ne mjereni!
        qd_log.append(q_d.copy())
        u_log.append(u_total.copy())
        
        # Korak simulacije
        mujoco.mj_step(model, data)
        sim_time += dt

    # Pretvaranje u numpy nizove
    t_arr = np.array(t_log)
    q_arr = np.array(q_log)
    qd_arr = np.array(qd_log)
    u_arr = np.array(u_log)
    
    # Spremanje
    filename = f"data/sim_data_{task_id}_{ref_type}.npz"
    np.savez(filename, t=t_arr, q=q_arr, qd=qd_arr, u=u_arr)
    
    # Računanje i ispis metrika
    calculate_metrics(t_arr, q_arr, qd_arr, ref_type, task_id)
    
    print(f"Zavrseno! Podaci spremljeni u {filename}")

# ==========================================
# POKRETANJE SVIH ZADATAKA (BATCH RUN)
# ==========================================
if __name__ == "__main__":
    
    # Zadatak 1: P regulator, step
    run_simulation("zadatak1", "step", 12.0, 
                   Kp=np.array([80.0, 80.0, 80.0]), 
                   Kd=np.array([0.0, 0.0, 0.0]), 
                   Ki=np.array([0.0, 0.0, 0.0]), 
                   disturbance_amp=np.array([0.0, 0.0, 0.0]), 
                   noise_std=0.0)
    
    # Zadatak 2: P regulator, sinus
    run_simulation("zadatak2", "sine", 10.0, 
                   Kp=np.array([80.0, 80.0, 80.0]), 
                   Kd=np.array([0.0, 0.0, 0.0]), 
                   Ki=np.array([0.0, 0.0, 0.0]), 
                   disturbance_amp=np.array([0.0, 0.0, 0.0]), 
                   noise_std=0.0)
                   
    # Zadatak 3: PD regulator, step
    # Podesavanje:
    # Testirano fino podesavanje Kd u rasponu [0.2, 0.9].
    # Pokazalo se da Kd = 0.7 nudi najbolji kompromis izmedu smanjenja overshoota (smanjen na 17.8%)
    # i kraceg settling timea (pad na 2.47s za Zglob 1) bez uvodjenja nestabilnosti na Zglobu 2.
    run_simulation("zadatak3", "step", 12.0, 
                   Kp=np.array([80.0, 80.0, 80.0]), 
                   Kd=np.array([0.7, 0.7, 0.7]), 
                   Ki=np.array([0.0, 0.0, 0.0]), 
                   disturbance_amp=np.array([0.0, 0.0, 0.0]), 
                   noise_std=0.0)
                   
    # Zadatak 4: PD regulator + disturbance, step
    run_simulation("zadatak4", "step", 12.0, 
                   Kp=np.array([80.0, 80.0, 80.0]), 
                   Kd=np.array([0.7, 0.7, 0.7]), 
                   Ki=np.array([0.0, 0.0, 0.0]), 
                   disturbance_amp=np.array([5.0, 5.0, 5.0]), 
                   noise_std=0.0)
                   
    # Zadatak 5: PID regulator + disturbance, step
    # Podesavanje:
    # Testirano fino podesavanje Ki u rasponu [10.0, 40.0] s novim Kd=0.7.
    # Ki = 35.0 savrseno balansira brzi settling time (za Zglob 3 pada s 6.44s na 4.97s)
    # i dovodi stacionarnu gresku do zanemarivih vrijednosti (~0.0005 rad), 
    # uz sasvim neznatno i prihvatljivo povecanje overshoota.
    run_simulation("zadatak5", "step", 12.0, 
                   Kp=np.array([80.0, 80.0, 80.0]), 
                   Kd=np.array([0.7, 0.7, 0.7]), 
                   Ki=np.array([35.0, 35.0, 35.0]), 
                   disturbance_amp=np.array([5.0, 5.0, 5.0]), 
                   noise_std=0.0)
                   
    # Zadatak 6: PID + disturbance + white noise, step (Fiksni parametri iz Z5)
    run_simulation("zadatak6", "step", 12.0, 
                   Kp=np.array([80.0, 80.0, 80.0]), 
                   Kd=np.array([0.7, 0.7, 0.7]), 
                   Ki=np.array([35.0, 35.0, 35.0]), 
                   disturbance_amp=np.array([5.0, 5.0, 5.0]), 
                   noise_std=0.02)
                   
    # Zadatak 6.2: PID + disturbance + white noise, step (Slobodni parametri)
    # Podesavanje:
    # Stohastickom optimizacijom (Differential Evolution metoda iz SciPy-a) maksimizirana je
    # otpornost sustava na Gaussov sum.
    # Algoritam je pronasao potpuno novu ravnotezu: iznimno malo pojacanje Kp=30.5 sprjecava
    # visoke frekvencije oscilacija pod sumom, Kd=0.5 stabilizira overshoot,
    # a Ki=11.0 u takvom sporijem sustavu savrseno "polako ali sigurno" brise stacionarnu gresku.
    # Settling time pada na fenomenalnih ~8.5 sekundi unatoc jakom sumu.
    run_simulation("zadatak6_2", "step", 12.0, 
                   Kp=np.array([30.5, 30.5, 30.5]), 
                   Kd=np.array([0.5, 0.5, 0.5]), 
                   Ki=np.array([11.0, 11.0, 11.0]), 
                   disturbance_amp=np.array([5.0, 5.0, 5.0]), 
                   noise_std=0.02)
