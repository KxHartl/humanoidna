Zadatak 2:
• Za UR3e robota (6SSG) početne konfiguracije 𝑞𝑖
i završne konfiguracije 𝑞𝑓 potrebno je:

1. Izračunati ukupni pomak svakog zgloba (𝐷)
2. Odrediti minimalno potrebno vrijeme trajektorije pojedinog zgloba tako da vrijedi
   uvjet 𝑡𝑓 = max(𝑡𝑣𝑒𝑙,𝑡𝑎𝑐𝑐)
3. Generirati sinkroniziranu trajektoriju tako da svi zglobovi krenu i završe u istom
   trenu
4. Za sve zglobove koristiti interpolaciju polinomom petog stupnja
5. Prikazati (plot) poziciju, brzinu i akceleraciju svih 6 zglobova
   Generiranje point-to-point trajektorije u joint space-u – polinom petog stupnja
   𝑞𝑖 = −1.571, −2.094, -1.571, 0.524, 1.571, 0.000 𝑟𝑎𝑑
   𝑞𝑓 = -0.785, −2.094, 0.000,−1.571, 0.000, 0.000 𝑟𝑎𝑑
   • Jednadžba trajektorije: 𝑞 𝑡 = 𝑞
   ⅈ + 𝑠 𝑡 𝑞
   𝑓 − 𝑞
   ⅈ
   • Diskretizaciju trajektorije provesti na 1000 vremenskih koraka
