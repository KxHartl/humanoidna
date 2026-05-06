Zadatak 4:
• Za UR3e robota (6SSG) potrebno je generirati point-to-point trajektoriju TCP-a u task
space-u koristeći polinom 5. reda te uzeti u obzir ograničenja brzine i akceleracije
robota
• Zadano je:
• početna i konačna pozicija TCP-a 𝑝𝑖
, 𝑝𝑓 te početna i konačna orijentacija 𝑅𝑖
, 𝑅𝑓
• ograničenja:
• maksimalna translacijska brzina 𝑣𝑚𝑎𝑥 = 0.5 m/s
• maksimalna akceleracija 𝑎𝑚𝑎𝑥 = 1.0 𝑚/𝑠
2
• maksimalna kutna brzina 𝜔𝑚𝑎𝑥 = 1.0 𝑟𝑎𝑑/𝑠
Generiranje point-to-point trajektorije u task space-u – polinom petog stupnja
𝑝𝑖 = -0.30, -0.120, 0.3 𝑚, 𝑝𝑓= -0.225, -0.120, 0.33 𝑚
𝑅𝑖 = 90, 0, 0 ˚, 𝑅𝑖 = 120, 0, 45 ˚
• Interpolacija translacije: 𝑝 𝑡 = 𝑝𝑖 + 𝑠 𝑡 𝑝𝑓 − 𝑝𝑖
, interpolacija rotacije: SLERP
(scipy.spatial.transform)
