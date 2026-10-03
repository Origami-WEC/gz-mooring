# Mooring.cc — Plugin di ormeggio MoorDyn per gz-sim

## Cosa fa

Questo plugin accoppia MoorDyn v2.7.1 a gz-sim10. Ad ogni step di fisica
calcola le forze di ormeggio sui fairlead del modello WEC e le applica
come wrench al link.

### Flusso

```
Configure          → legge SDF, trova link, abilita velocità
InitMooring        → MoorDyn_Create + MoorDyn_Init (primo PreUpdate)
PreUpdate (ogni t) → posa/velocità fairlead → MoorDyn_Step → AddWorldWrench
PostUpdate (30 Hz) → marker LINE_LIST + Float_V su /mooring/tension
~Mooring           → MoorDyn_Close
```

### API MoorDyn usata (verificata da source/MoorDyn2.h e source/Line.h)

| Funzione | Ruolo |
|---|---|
| `MoorDyn_Create(file)` | Legge lines.txt, alloca strutture. NO "MoorDyn_Init" da solo. |
| `MoorDyn_NCoupledDOF(sys, &n)` | Ritorna `n = 3 × nCoupledPoints`. |
| `MoorDyn_Init(sys, x, xd)` | Condizioni iniziali. `x`/`xd` = posizioni/velocità iniziali dei Coupled point. |
| `MoorDyn_Step(sys, x, xd, f, &t, &dt)` | Avanza di `dt`. `t` e `dt` sono **puntatori in/out**. `f` = forze in output. |
| `MoorDyn_Close(sys)` | Dealloca. |
| `MoorDyn_GetNumberLines(sys, &n)` | Numero cavi. |
| `MoorDyn_GetLine(sys, idx)` | Handle cavo (idx da 1). |
| `MoorDyn_GetLineNumberNodes(line, &n)` | Nodi del cavo (= NumSegs + 1). |
| `MoorDyn_GetLineNodePos(line, i, pos)` | Posizione nodo `i` in world. |
| `MoorDyn_GetLineFairTen(line, &t)` | Tensione al fairlead (N). |

### Dimensioni array

`x`, `xd`, `f` sono array di `3 × nCoupledPoints` double.
Con 2 fairlead → 6 double:
```
x  = [x1, y1, z1,  x2, y2, z2]   posizioni world dei Coupled point
xd = [vx1, vy1, vz1, vx2, vy2, vz2]  velocità world
f  = [fx1, fy1, fz1, fx2, fy2, fz2]  forze world (output)
```

### Calcolo del wrench

`AddWorldWrench(_ecm, F, τ)` applica al **link origin** (non al COM).
Il momento va calcolato a mano:

```
F = Σ f_i                          (forza totale)
τ = Σ (R · p_local_i) × f_i       (momento rispetto al link origin)
```

dove `R · p_local_i` è il braccio dal link origin al fairlead in world.
Questo è necessario perché `ExternalWorldWrenchCmd` ignora `force_offset`
(dichiarato in ExternalWorldWrenchCmd.hh:38).

### Velocità del fairlead

`WorldLinearVelocity(_ecm, offset)` con `offset = p_local` ritorna
direttamente la velocità del punto sul body nel world frame.
Non serve calcolare `v_com + ω × r` a mano.

---

## Come creare lines.txt (formato MoorDyn v2)

Il file è generato da `src/simulation/build_simulation.py` → `generate_mooring_config()`
a partire da `config/mooring.json`. Se serve rigenerarlo a mano, seguire
questo schema.

### Sezioni obbligatorie (in ordine)

```
--------------------- MoorDyn Input File ------------------------------------
<descrizione>
----------------------- LINE TYPES ------------------------------------------
TypeName   Diam    Mass/m     EA         BA/-zeta    EI         Cd     Ca     CdAx    CaAx
(name)     (m)     (kg/m)     (N)        (N-s/-)     (N-m^2)    (-)    (-)    (-)     (-)
chain      0.090   160.0      7.5e8      -0.8        0          1.4    1.0    0.5     0.0
---------------------- POINT PROPERTIES --------------------------------
ID  Type     X       Y       Z       Mass   Volume  CdA    Ca
(#) (-)      (m)     (m)     (m)     (kg)   (m^3)   (m^2)  (-)
1   Fixed    50.0    0.0     -30.0   0      0       0      0
2   Coupled  2.5     0.0     -0.5    0      0       0      0
3   Fixed    -50.0   0.0     -30.0   0      0       0      0
4   Coupled  -2.5    0.0     -0.5    0      0       0      0
---------------------- LINES ----------------------------------------
ID  LineType AttachA AttachB UnstrLen  NumSegs LineOutputs
(#) (name)   (#)     (#)     (m)       (-)     (-)
1   chain    1       2       64.30     20      t
2   chain    3       4       64.30     20      t
---------------------- OPTIONS -----------------------------------------
2       writeLog
0.0005  dtM
3.0e6   kBot
3.0e5   cBot
1025.0  WtrDnsty
30      WtrDpth
1.0     dtIC
100.0   TmaxIC
4.0     CdScaleIC
0.001   threshIC
1       disableOutTime
---------------------- OUTPUTS -----------------------------------------
FairTen1
FairTen2
------------------------- need this line --------------------------------------
```

### Regole vincolanti

| Regola | Dettaglio |
|---|---|
| **10 colonne** LINE TYPES | `TypeName Diam Mass/m EA BA/-zeta EI Cd Ca CdAx CaAx` |
| **9 colonne** POINTS | `ID Type X Y Z Mass Volume CdA Ca` |
| **7 colonne** LINES | `ID LineType AttachA AttachB UnstrLen NumSegs LineOutputs` |
| **Type** punti | `Fixed` (ancora) · `Coupled` (fairlead, accoppiato a gz-sim) · `Connect` (snodo intermedio) |
| **LineOutputs** | `t` = tensione per segmento → file `lines_LineN.out`. Vuoto = nessun output per-line. |
| **BA/-zeta negativo** | Interpreta il valore come damping ratio. MoorDyn calcola BA = -zeta · 2 · √(k · m). Consigliato. |
| **ID Lines sequenziali** | Devono partire da 1 senza salti (1, 2, 3, ...) |
| **Riga finale** | `---... need this line ...---` **obbligatoria** |

### Numero di punti Coupled = numero di fairlead SDF

Ogni punto `Coupled` in lines.txt corrisponde a un `<fairlead>` nel plugin
SDF. L'ordine deve essere lo stesso. Il plugin fallisce se i conti non
tornano.

### Output di MoorDyn

MoorDyn scrive nella **stessa directory di lines.txt** (il cosiddetto
`_basepath`, vedi MoorDyn2.cpp:120):

| File | Contenuto |
|---|---|
| `lines.out` | Output principale: colonne = canali sezione OUTPUTS (FairTen1, ...) |
| `lines_Line1.out` | Dettaglio per-segmento cavo 1 (se LineOutputs ha flag) |
| `lines_Line2.out` | Dettaglio per-segmento cavo 2 |
| `lines.log` | Log (se writeLog ≥ 1) |

Poiché lines.txt è in `logs/mooring/`, tutti gli output finiscono lì.

### Geometria ancore-fairlead (frame di riferimento)

| Grandezza | Frame | Dove si scrive |
|---|---|---|
| Fairlead (`pos_local`) | **link** (es. `hull`) | `mooring.json → fairleads[].pos_local` |
| Ancora (`anchor_world`) | **mondo**, sul fondale | `mooring.json → lines[].anchor_world` |
| Superficie del mare | world `z = 0` | — |
| Fondale | world `z = -water_depth_m` | `mooring.json → water_depth_m` |
| Spawn del modello | world `[x y z roll pitch yaw]` | `mooring.json → model_spawn_pose` |

**Posizionamento dell'ancora**: `lines[].anchor_world = [x, y, z]` è la
posizione dell'ancora sul fondale nel **frame mondo**. È l'unico modo per
piazzare l'ancora dove si vuole.

- `lines[].unstretched_length_m` = lunghezza a riposo del cavo. Se `null` →
  `unstretched_length_scale × ‖anchor_world − fairlead_world0‖`.
- Il plugin **ricalcola UnstrLen all'init** dalle pose reali (`recompute_unstretched_length`)
  e valida lo slack in `[slack_min_ratio, slack_max_ratio]`.
- `dtM` è calcolato automaticamente dal **CFL assiale** del cavo
  (`0.25 × (L/N) / sqrt(EA/μ)`). Non usare `dtM` fisso con EA elevata.

Esempio (3 linee a 120°, fondale a −30 m):

```json
"lines": [
  {"name": "line_a", "fairlead": "fairlead_a",
   "anchor_world": [15.0, 0.0, -30.0], "unstretched_length_m": null},
  {"name": "line_b", "fairlead": "fairlead_b",
   "anchor_world": [-7.5, 12.99, -30.0], "unstretched_length_m": null},
  {"name": "line_c", "fairlead": "fairlead_c",
   "anchor_world": [-7.5, -12.99, -30.0], "unstretched_length_m": null}
]
```

### Fallimento del solver

Con `on_failure: "stop"` (default) un `MoorDyn_Step` fallito o forze NaN:
1. `gzerr` con diagnosi complete (t, tensioni, nodi, slack)
2. report in `logs/mooring/FAILURE.txt`
3. exit code **3** → `src/simulation/run_simulation.sh` stampa `MOORING FAILURE`

`on_failure: "disable"` azzera le forze e continua (solo debug).

### Onde sui cavi

`waves.enabled = true` genera componenti JONSWAP e le passa a MoorDyn via
`MoorDyn_ExternalWaveKinSet` (WaveKin = 1). Con `enabled = false` (decay test)
i cavi vedono acqua calma (`WaveKin = 0`).

### Rottura cavi

`break_tension_n > 0` abilita `MoorDyn_BreakLine` alla tensione di rottura.

---

## Come creare il blocco SDF

Generato da `src/simulation/build_simulation.py` → `_add_mooring()`. Schema:

```xml
<plugin filename="MooringSystem" name="gz::sim::systems::MooringSystem">
  <!-- percorso a lines.txt (relativo al CWD della simulazione) -->
  <mooring_file>logs/mooring/lines.txt</mooring_file>

  <!-- un <line> per ogni Coupled point in lines.txt, stesso ordine -->
  <line>
    <anchor>2.5 0 -30</anchor>          <!-- world, sul fondale -->
    <unstretched_length>-1</unstretched_length>  <!-- -1 = scale * dist -->
    <scale>1.15</scale>
  </line>

  <!-- un <fairlead> per ogni Coupled point in lines.txt, stesso ordine -->
  <fairlead link="hull">
    <pos>2.5 0 -0.5</pos>       <!-- posizione nel frame del link -->
  </fairlead>
  <fairlead link="hull">
    <pos>-2.5 0 -0.5</pos>
  </fairlead>

  <waves enabled="true">
    <component>
      <amplitude>0.2</amplitude>
      <omega>1.0</omega>
      <wavenumber>0.1</wavenumber>
      <phase>0.0</phase>
      <dir_deg>0</dir_deg>
    </component>
  </waves>

  <tension_topic>/mooring/tension</tension_topic>
  <marker_rate>30</marker_rate>  <!-- Hz per marker + topic tensione -->
</plugin>
```

Valori strutturati (anchor, scale, componente onda) come **elementi figli**.
Il parser accetta anche la forma legacy ad attributi (`<line anchor="…"/>`).

### Parametri

| Elemento | Tipo | Default | Descrizione |
|---|---|---|---|
| `<mooring_file>` | string | `logs/mooring/lines.txt` | Input MoorDyn (relativo al workspace, risolto via `MARITIME_WS`) |
| `<line><anchor>` | Vector3 | — | Ancora nel frame mondo (metri) |
| `<line><unstretched_length>` | double | `-1` | Lunghezza a riposo; `-1` = `scale * dist` |
| `<line><scale>` | double | `1.15` | Fattore di slack su `unstretched_length` |
| `<fairlead link="...">` | ripetuto | — | Nome link del punto di attacco |
| `<fairlead><pos>` | Vector3 | — | Posizione fairlead nel frame del link (metri) |
| `<waves enabled>` + `<component>` | — | — | Componenti d'onda (Airy) per i cavi |
| `<break_tension>` | double | `0` (off) | Tensione di rottura [N] |
| `<on_failure>` | string | `stop` | `stop` \| `disable` |
| `<slack_min>` / `<slack_max>` | double | `0.05` / `0.45` | Finestra slack accettata |
| `<water_depth>` | double | — | Profondità fondale [m] |
| `<tension_topic>` | string | `/mooring/tension` | Topic gz::msgs::Float_V con tensioni FairTen |
| `<marker_rate>` | double | `30` | Frequenza aggiornamento marker e topic (Hz) |

### Collegatevi al topic tensione

```bash
gz topic -e -t /mooring/tension
# Float_V { data: [FairTen1, FairTen2, ...] }
```

---

## Build

Dalla root del workspace:

```bash
make plugin
```

oppure a mano:

```bash
# 1. MoorDyn (una tantum)
cmake -S src/MoorDyn -B src/MoorDyn/build \
      -DPYTHON_WRAPPER=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
make -C src/MoorDyn/build -j4

# 2. Plugin
cmake -S src/gz-mooring -B src/gz-mooring/build -DCMAKE_BUILD_TYPE=Release
make -C src/gz-mooring/build -j4
```

`src/simulation/run_simulation.sh` aggiunge automaticamente `src/gz-mooring/build` a
`GZ_SIM_SYSTEM_PLUGIN_PATH` e `src/MoorDyn/build/source` a
`DYLD_LIBRARY_PATH`.
