# Bachelor Thesis

Za razvoj i evaluaciju PI, PD i PID kontrolera s fokusom na upravljanje redundantnom robotskom rukom, preporuča se sljedeći pojednostavljeni pristup:
1. Upoznavanje s Franka Control Interface (FCI)
  - Pročitajte dokumentaciju FCI-a za razumijevanje osnovnih funkcija i parametara za kontrolu robotske ruke.
2. Analiza Primjera `motion_with_control.cpp`
  - Proučite kako se koristi generator brzine zglobova i kontrola zakretnog momenta.
  - Obratite pažnju na hvatanje i zapisivanje logova pri iznimkama.
3. Implementacija Kontrolnih Algoritama
  - Implementirajte PI, PD, i PID algoritme u C++, koristeći `motion_with_control.cpp` kao referencu.
  - Testirajte različite parametre ($K_P$, $K_D$, $K_I$) za svaki algoritam.
4. Kreiranje Profila Gibanja
  - Kreirajte profil gibanja vrha alata, baziran na primjeru iz `motion_with_control.cpp`.
  - Ovaj korak ćete u budućnosti unaprijediti da omogućite:
Rad s Kartezijanskim Položajem i Brzinom – proučite  primjere `generate_cartesian_pose_motion.cpp` i `generate_cartesian_velocity_motion.cpp` za razvoj algoritama koji upravljaju kartezijanskim položajem i brzinom.
Praćenje određene krivulje ili profila. To može uključivati linearne trajektorije, kružne putanje ili složenije oblike gibanja u kartezijanskom prostoru.
5. Eksperimentalno Testiranje
  - Testirajte svaki kontrolni algoritam s definiranim profilom gibanja.
  - Zabilježite relevantne podatke i evaluirajte performanse.


https://medium.com/@jaems33/understanding-robot-motion-pid-control-8931899c31df