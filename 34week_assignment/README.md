# E-puck Robot Végső Feladat: Akadálykerülés és Falmentén Haladás (FSM)

Ez a projekt a Webots szimulációs környezetben futó e-puck robot vezérlését valósítja meg egy **Véges Állapotú Gép (FSM)** segítségével. A robot célja, hogy elkerülje az útjában álló két akadályt, majd a második akadály mellett haladva elérje a fal végét, és ott megálljon.

---

## ⚙️ Állapotgép (FSM) Áttekintés

A robot viselkedését az alábbi állapotgép (lásd: `34week_assignment_diagram.png`) határozza meg, amely a fő logikát biztosítja:

| Állapot               | Leírás                                       | Átmeneti feltétel                                      | Következő Állapot     |
| :-------------------- | :------------------------------------------- | :----------------------------------------------------- | :-------------------- |
| **FORWARD_TO_FIRST**  | Haladás az első akadály felé.                | Elülső akadály észlelése (`ps0` vagy `ps7` szenzorok). | **TURN_180**          |
| **TURN_180**          | 180°-os fordulás végrehajtása (IMU alapján). | A kívánt 180°-os elfordulás befejezése.                | **FORWARD_TO_SECOND** |
| **FORWARD_TO_SECOND** | Haladás a második akadály felé.              | Elülső akadály észlelése (`ps0` vagy `ps7` szenzorok). | **TURN_RIGHT**        |
| **TURN_RIGHT**        | Jobbra fordulás.                             | Bal oldali akadály észlelése (`ps5` szenzor).          | **WALL_FOLLOWING**    |
| **WALL_FOLLOWING**    | Fal követése a bal oldalon.                  | Bal oldali akadály (fal) végének elérése.              | **STOPPED**           |
| **STOPPED**           | Végállapot: a robot megáll.                  | -                                                      | -                     |

---

## 🛠️ Megvalósítás

A vezérlő a `controller.py` fájlban található, és a következő fő részekből áll:

### 1. Inicializáció (`__init__`)

- **Robot és Lépésidő:** A Webots `Robot` objektum és az `e-puck` alapvető időegysége (`timestep`) beállítása.
- **Motorok:** A `left wheel motor` és `right wheel motor` inicializálása, beállítása végtelen pozícióra (folyamatos forgás) és kezdeti sebesség nullára.
- **Szenzorok:** Az `Inerciális Egység (IMU)` és a 8 darab `Távolságérzékelő (ps0-ps7)` inicializálása és engedélyezése.
- **Paraméterek:** A sebesség- és küszöbértékek (pl. `MAX_SPEED`, `FRONT_DISTANCE_THRESHOLD`, `TARGET_ANGLE` a 180°-os forduláshoz) definiálása.
- **Kezdő Állapot:** Az FSM kezdőállapotának beállítása: `State.FORWARD_TO_FIRST`.

### 2. Segédmetódusok

A kód tartalmazza a robot mozgásához, szenzoradatok feldolgozásához és szögkezeléshez szükséges segédmetódusokat:

- `get_yaw_angle`: Lekéri az aktuális dőlésszöget (Yaw, a függőleges tengely körüli elfordulás) az IMU-tól.
- `normalize_angle`: Normalizálja a szöget a $[-\pi, \pi]$ tartományba.
- `detect_obstacle_front` / `detect_obstacle_left`: Bool értékkel jelzi az akadály észlelést az elülső (`ps0`, `ps7`) vagy bal oldali (`ps5`) szenzorok alapján.
- `set_motor_speeds` / `stop_motors` / `move_forward` / `turn_right`: Alapvető motorvezérlési parancsok.

### 3. Állapotkezelő Metódusok (`state_...`)

Minden FSM állapotnak van egy dedikált metódusa (`state_forward_to_first`, `state_turn_180`, stb.), amely végrehajtja az állapotra jellemző műveleteket, ellenőrzi az átmeneti feltételeket, és szükség esetén beállítja a következő állapotot.

- A **`state_turn_180`** állapot az IMU adatokat használja a pontos szög alapú fordulás végrehajtására.
- A **`state_turn_right`** és **`state_wall_following`** állapotok a bal oldali szenzort (`ps5`) használják a fal eléréséhez és követéséhez.

### 4. Fő FSM Ciklus (`run_fsm_step`)

- Ez a metódus a fő futási ciklusban minden időegységben lefut.
- Beolvassa a szenzorértékeket.
- A `self.current_state` alapján meghívja a megfelelő állapotkezelő metódust, ezzel végrehajtva az aktuális FSM lépést.

---

## 🚀 Futtatás

A projekt a **Webots** szimulációs környezetben futtatható.

1. Nyissa meg a Webots-ot.
2. Töltse be a világfájlt, amely tartalmazza az e-puck robotot és a két akadályt (a `34week_assignment.png` képen látható környezet).
3. Győződjön meg róla, hogy az e-puck vezérlőjének neve a kódnak megfelelően van beállítva.
4. Indítsa el a szimulációt.

A robotnak automatikusan el kell kezdenie a futást és az alábbi sorrendben kell végigmennie a feladaton:

1. Halad az első akadályig.
2. Megfordul 180°-kal.
3. Halad a második akadályig.
4. Jobbra fordul, amíg falat nem észlel a bal oldalán.
5. Falmentén halad, amíg el nem éri a fal végét.
6. Megáll (`STOPPED` állapot).
