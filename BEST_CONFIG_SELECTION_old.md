# Best Config Selection — Per-Run Analysis

For each run: all available configs are plotted, divergence checked, and the best choice recorded.
Plot script: `plot_config_comparison.py <run_name> <out_png>`
Plots saved under: `/cluster/scratch/ademirtas/results/plots_final/`

Divergence threshold: XY spread > 500 m.
Slowrate = `--rate 0.5` playback.

---

## floor_6

### floor_6 / 2025-07-07 / run_1

**Plot:** `plots_final/floor_6/floor_6_2025-07-07_run_1_configs.png`
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | Status |
|--------|----------|---|--------|-------|--------|
| baseline | 1× | 809 | 14977×3737 | 9862.6 | DIVERGED |
| dynA | 1× | 1239 | 20×25 | 0.7 | OK |
| imuB | 1× | — | — | — | empty (failed to init) |
| expC | 1× | — | — | — | empty (failed to init) |
| fastinit | 1× | — | — | — | empty (failed to init) |
| baseline | 0.5× | 2173 | 3850×5392 | 2992.3 | DIVERGED |
| dynA | 0.5× | 2155 | 680×1053 | 119.5 | DIVERGED |
| imuB | 0.5× | 2173 | 21×25 | 0.5 | OK |
| expC | 0.5× | 2151 | 31×27 | 3.7 | OK |
| fastinit | 0.5× | 2176 | 20×25 | 0.4 | **OK** |

**Selected:** `fastinit (0.5×)` — tightest Z (0.4 m), smallest footprint (20×25 m), most poses (2176). imuB 0.5× is essentially identical; fastinit chosen for consistency. Previous submission used expC 0.5× (Z 3.7 m) which was plausible but clearly worse.

**Notes:** All 1× configs except dynA failed to init or diverged. dynA 0.5× and baseline 0.5× diverge catastrophically. Three slowrate survivors (imuB, expC, fastinit) all produce consistent ~20-31×25-27 m trajectories; fastinit/imuB are tighter than expC. dynA 1× (20×25 m, Z 0.7 m) independently confirms the correct footprint scale despite lower pose count (~57% coverage).

---

## floor_2

### floor_2 / 2025-05-05 / run_1

**Plot:** `plots_final/floor_2/floor_2_2025-05-05_run_1_configs.png`

| Config | Slowrate | n | XY (m) | Z (m) | ATE (m) | Status |
|--------|----------|---|--------|-------|---------|--------|
| baseline | 1× | 3517 | 37×45 | 1.3 | 0.690 | OK |
| expC | 0.5× | 5354 | 57994×79891 | 69142.6 | — | DIVERGED |
| fastinit | 0.5× | — | — | — | — | empty (failed to init) |
| baseline 0.5× (vi_ready rerun) | 0.5× | 5560 | 37×46 | 0.4 | **0.438** | **OK** |
| submission (baseline 1× upsampled) | — | 5692 | 37×45 | 2.1 | 0.687 | OK |

**Selected:** `baseline 0.5×` (the vi_ready rerun) — best ATE 0.438 m. Confirmed via SLURM log (job 66400523): used `hilti_openvins` base config at `--rate 0.5`. The ATE improvement over 1× baseline (0.690 m) is entirely from slower playback giving better optical flow, **not** from VI bundle adjustment — BA has not been run yet.

**Current submission uses:** baseline 1× upsampled (ATE 0.687 m) — should be replaced with the 0.5× rerun.

**Notes:** expC diverged despite slowrate. fastinit produced an empty file. No dynA/imuB slowrate results exist for this run. Once VI optimization (`colmap_imu/python/examples/vi_optimization.py`) is run on the `vi_ready` colmap reconstruction, further improvement is expected.

### floor_2 / 2025-12-02 / run_1

**Plot:** `plots_final/floor_2/floor_2_2025-12-02_run_1_configs.png`
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | Status |
|--------|----------|---|--------|-------|--------|
| baseline | 1× | 4213 | 36608×32217 | 17192.9 | DIVERGED |
| dynA | 1× | 3114 | 426×1301 | 86.9 | DIVERGED |
| dynA (flat dir) | 1× | 2152 | 39170×49120 | 36038.9 | DIVERGED |
| imuB | 1× | 2581 | 37×36 | 1.2 | OK |
| fastinit | 1× | — | — | — | empty (failed to init) |
| baseline | 0.5× | 4144 | 39×34 | 1.1 | OK |
| dynA | 0.5× | 4216 | 39×34 | 1.0 | **OK — best** |
| imuB | 0.5× | 4215 | 39×36 | 1.1 | OK |
| expC | 0.5× | — | — | — | DIVERGED |
| submission (dynA 0.5× upsampled) | — | 4552 | 39×34 | 1.0 | **OK — current** |

**Selected:** `dynA (0.5×)` — swapped from imuB 1× on 2026-05-14. All three 0.5× survivors (baseline, dynA, imuB) agree on a 39×34–36 m footprint with Z 1.0–1.1 m. dynA chosen for tightest Z (1.0 m) and consistency with other runs. Upsampled: 9.4 s backward extrapolation, 1.7 s end gap → 4552 poses, ~100% server coverage.

**Previous:** imuB 1× (2581 poses, 61.4% raw coverage, Z 2.5 m after upsample) — coverage limited by 1× playback outputting at 16–22 Hz.

---

## floor_EG

### floor_EG / 2025-12-02 / run_1

**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | Status |
|--------|----------|---|--------|-------|--------|
| baseline | 1× | — | — | — | (not checked separately) |
| dynA | 0.5× | 3680 | 12×27 | 0.9 | **OK** |
| imuB | 0.5× | 3678 | 12×28 | 1.0 | **OK** |
| expC | 0.5× | — | — | — | DIVERGED |
| fastinit | 0.5× | 3673 | 12×28 | 0.9 | **OK** (current submission) |

**Selected:** `fastinit (0.5×)` — current submission, 99.9% coverage. dynA and imuB slowrate are nearly identical alternatives.

---

### floor_EG / 2025-12-02 / run_2

**Plot:** `plots_final/floor_EG/floor_EG_2025-12-02_run_2_configs.png`  
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | init_delay | Status | Notes |
|--------|----------|---|--------|-------|------------|--------|-------|
| baseline | 1× | 2352 | 16098×1767 | 3583.8 | 61.6 s | DIVERGED | |
| dynA | 1× | 2756 | 272×366 | 29.3 | 20.7 s | DIVERGED (slow) | under 500 m threshold but visually a long drift line |
| imuB | 1× | 2221 | 27×15 | 1.8 | 59.1 s | OK | |
| expC | 0.5× | 4801 | 280×72 | 62.7 | 5.1 s | DIVERGED (slow) | small cluster + long tail drift |
| fastinit | 0.5× | 4753 | 23832×30918 | 13223.9 | 6.9 s | DIVERGED | |
| dynA | 0.5× | 4416 | 1636×854 | 312.7 | 17.0 s | DIVERGED | |
| imuB | 0.5× | 3199 | 27×14 | 1.5 | 58.8 s | **OK** | more poses, lower Z than 1× |
| expC | 1× | 2962 | 13158×8952 | 8855.5 | 16.8 s | DIVERGED | |
| fastinit | 1× | 4188 | 19016×16149 | 5173.0 | 7.1 s | DIVERGED | |
| baseline | 0.5× | 0 | — | — | — | failed to init | |
| cam1only | — | 0 | — | — | — | failed to init | cam1 (opposite hemisphere) also fails |
| imuB_dyninit | 0.5× | 3200 | 28×15 | 1.6 | 58.7 s | OK | dyn init makes no difference |
| imuB_highdisp | 0.5× | 3200 | 27×15 | 1.5 | 58.7 s | OK | max_disp 40 makes no difference |
| imuB_zupt | 0.5× | — | — | — | — | FAILED (wait_for_jerk=false needs is_still, but cam is moving at 59s jerk) | |
| imuB_clahe | 0.5× | 3200 | 27×14 | 1.8 | 58.7 s | OK (no improvement) | CLAHE contrast enhancement |
| imuB_lowfeat | 0.5× | — | — | — | — | FAILED | init_max_features:8 breaks geometry |
| imuB_clahe_lowfeat | 0.5× | — | — | — | — | FAILED | |
| imuB_dynfastinit | 0.5× | 4753 | 15×30 | 1.65 | **5.6 s** | **OK** | init_window_time:1.0 + init_dyn_use:true |
| submission (imuB_dynfastinit 0.5× upsampled) | — | 5017 | 15×30 | 1.65 | — | **OK** | **current** — replaced 2026-05-14 |

**Selected:** `imuB_dynfastinit (0.5×)` — **5.6 s init delay** (vs 59 s for all other imuB variants), Z 1.65 m (vs 3.9 m for imuB 0.5× upsampled). Dynamic init with 1 s window succeeds because within the first 1 s window the camera is momentarily still while the man adjusts his grip, then the shake provides enough rotation for dynamic init. imuB noise prevents divergence through dark rooms. Updated submission 2026-05-14.

**Notes:** Every static-init config has 59 s delay. Root cause: `wait_for_jerk = (updaterZUPT == nullptr)` in VioManagerHelper.cpp:106 — static init waits for still→moving jerk which only happens at 59 s. Dynamic init with short window (`init_window_time: 1.0`) fires during the early camera shake motion. imuB noise is critical for surviving dark rooms after init. Also submitted reversed bag run (job 66500594, still pending) — reversed bag starts in well-lit corridor, potentially eliminating the dark-room tracking problem entirely.

---

## floor_7

### floor_7 / 2025-12-02 / run_1

**Plot:** `plots_final/floor_7/floor_7_2025-12-02_run_1_configs.png`  
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | ATE | Status |
|--------|----------|---|--------|-------|-----|--------|
| baseline | 1× | 2059 | 5115×2067 | 812.4 | — | DIVERGED |
| dynA | 1× | — | — | — | — | empty |
| imuB | 1× | 2749 | 41×18 | 0.6 | — | **OK** |
| expC | 0.5× | 3336 | 46×29 | 2.0 | — | OK |
| fastinit | 0.5× | 3338 | 40×18 | 0.6 | — | OK |
| submission (fastinit upsampled) | — | 3685 | 40×18 | 0.6 | — | OK |

**Selected:** `fastinit (0.5×)` — current submission, matches imuB shape with identical XY/Z spread. expC has a slightly larger footprint (46×29 vs 40×18). No GT to differentiate further; fastinit and imuB look consistent with each other, expC diverges slightly in shape.

**Notes:** baseline diverges immediately. dynA produced an empty file.

---

### floor_7 / 2025-12-02 / run_2

**Plot:** `plots_final/floor_7/floor_7_2025-12-02_run_2_configs_v2.png`  
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | ATE | Status |
|--------|----------|---|--------|-------|-----|--------|
| baseline | 1× | 3053 | 6651×2663 | 1165.1 | — | DIVERGED |
| dynA | 1× | 2573 | 42×15 | 2.6 | — | OK |
| imuB | 1× | 2805 | 42×21 | 1.6 | — | OK |
| expC | 0.5× | 4560 | 36×25 | 1.9 | — | OK |
| dynA | 0.5× | 4606 | 42×19 | 1.5 | — | **OK** |
| fastinit | 0.5× | 4569 | 41×20 | 1.5 | — | OK |
| imuB | 0.5× | 0 | — | — | — | failed to init |
| submission (dynA 0.5× interpolated) | — | 4611 | 42×19 | 1.5 | — | **OK** |

**Selected:** `dynA slowrate (0.5×)` — best available. Marginally more poses than fastinit slowrate (4606 vs 4569), same Z drift (1.5 m), and tighter than expC (1.9 m Z). Shape matches 1× dynA/imuB reference well. Upsampled with interpolation only (no extrapolation); 3.5 s init delay and 1.6 s end gap left as-is → 4611 poses, ~96.8% raw coverage.

**Notes:** baseline diverges immediately. imuB slowrate failed to initialize. dynA and fastinit slowrate are nearly identical; both clearly better than expC (lower Z drift, consistent shape, same pose density).

---

### floor_7 / 2025-12-03 / run_1

**Plot:** `plots_final/floor_7/floor_7_2025-12-03_run_1_configs_v3.png`
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | Status |
|--------|----------|---|--------|-------|--------|
| baseline | 1× | 3477 | 27×39 | 0.6 | OK |
| dynA | 1× | 3536 | 26×40 | 0.6 | OK |
| imuB | 1× | 0 | — | — | FAILED TO INIT |
| baseline | 0.5× | 5853 | 26×39 | 0.7 | OK |
| expC | 0.5× | 5539 | 40×42 | 1.5 | OK (larger footprint) |
| dynA | 0.5× | 5842 | 25×39 | 0.5 | **OK — tightest** |
| fastinit | 0.5× | 5814 | 32×38 | 0.7 | OK |
| imuB | 0.5× | 5853 | 30×42 | 0.7 | OK |
| submission (expC 0.5× interpolated + back-extrap) | — | 6043 | 40×42 | 1.5 | TRIED — scored 27/100 |
| submission (dynA 0.5× interpolated + back-extrap) | — | 6043 | 25×39 | 0.5 | OK — ~99.1% coverage — current |

**Selected:** `dynA (0.5×)` — final choice after two failed alternatives. fastinit scored 65/100; expC scored 27/100 (diverged globally despite plausible local shape). dynA 0.5× is the tightest result (25×39 m, Z 0.5 m) and agrees with baseline 1×, dynA 1×, and baseline 0.5× — four independent configs landing on ~26×39 m. Submission: interpolated to 30 Hz + backward-extrapolated 6.5 s to bag_start; end gap 1.87 s left as-is → 99.1% server coverage.

Server scores: fastinit 0.5× → 65/100; expC 0.5× → 27/100; dynA 0.5× → TBD.

**Notes:** imuB 1× failed to initialize. expC 0.5× looks locally reasonable (40×42 m) but scored catastrophically (27/100) — global drift not visible in XY spread alone. The four-config consensus on 26×39 m is strong evidence dynA 0.5× is correct.

---

## floor_UG1

### floor_UG1 / 2025-10-16 / run_1

**Plot:** `plots_final/floor_UG1/floor_UG1_2025-10-16_run_1_configs_overlay.png`
**GT available:** yes

| Config | Slowrate | n | XY (m) | Z (m) | ATE (m) | Status |
|--------|----------|---|--------|-------|---------|--------|
| expC | 0.5× | 5982 | 319×77 | 43.1 | 104.202 | DIVERGED (XY ok but high ATE) |
| dynA | 0.5× | 5385 | 47×60 | 0.5 | **0.522** | OK |
| fastinit | 0.5× | 5968 | 44×60 | 2.2 | 1.117 | OK |
| vi_ready (baseline 0.5×) | 0.5× | 5965 | 47×59 | 0.6 | **0.371** | **OK** |

**Selected:** `vi_ready (baseline 0.5×)` — best ATE 0.371 m, beats fastinit (1.117 m) and dynA (0.522 m). Updated submission 2026-05-14.

**Notes:** expC diverges (high ATE despite plausible XY spread). The vi_ready rerun is baseline config at 0.5× — the improvement over fastinit is purely from the baseline config being more conservative on this sequence. No BA run yet; further improvement expected once VI optimization is applied.

---

### floor_UG1 / 2025-12-02 / run_1

**Plot:** `plots_final/floor_UG1/floor_UG1_2025-12-02_run_1_configs.png`
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | Status |
|--------|----------|---|--------|-------|--------|
| baseline | 1× | 7219 | 38068×79670 | 216865.2 | DIVERGED |
| dynA | 1× | 4516 | 33559×105134 | 77338.5 | DIVERGED |
| imuB | 1× | — | — | — | empty |
| baseline | 0.5× | — | — | — | empty (failed to init) |
| expC | 0.5× | 7516 | 81×46 | 7.8 | OK (elevated Z) |
| dynA | 0.5× | 7492 | 59×54 | 0.7 | **OK** |
| fastinit | 0.5× | — | — | — | empty (failed to init) |
| imuB | 0.5× | 7527 | 60×56 | 0.6 | **OK** |

**Selected:** `dynA slowrate` — consistent shape with imuB slowrate (59×54 m, Z 0.7 m vs 60×56 m, Z 0.6 m); dynA preferred for consistency with other runs where dynA is the best config. Updated submission 2026-05-14.

**Notes:** All 1× configs diverge badly. fastinit and baseline slowrate both failed to init. expC slowrate is OK but Z 7.8 m is suspicious for an indoor floor. dynA and imuB slowrate are equivalent in shape; dynA chosen over imuB for submission.

---

### floor_UG1 / 2025-12-02 / run_2

**Plot:** `plots_final/floor_UG1/floor_UG1_2025-12-02_run_2_configs.png`
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | Status |
|--------|----------|---|--------|-------|--------|
| baseline | 1× | 5026 | 30890×3707 | 4313.3 | DIVERGED |
| dynA | 1× | 3858 | 56519×9100 | 9221.5 | DIVERGED |
| imuB | 1× | 5297 | 46603×32531 | 16019.2 | DIVERGED |
| baseline | 0.5× | — | — | — | empty (failed to init) |
| expC | 0.5× | 6505 | 1090×3225 | 594.6 | DIVERGED |
| dynA | 0.5× | 6546 | 57×61 | 0.6 | **OK** |
| fastinit | 0.5× | 6610 | 58×61 | 0.6 | **OK** |
| imuB | 0.5× | 6610 | 58×63 | 0.7 | **OK** |

**Selected:** `fastinit slowrate` — consistent with dynA and imuB slowrate (all three ~58×62 m, Z ≤ 0.7 m). Updated submission 2026-05-14.

**Notes:** All 1× configs and expC slowrate diverge. Three slowrate variants (dynA, fastinit, imuB) all produce consistent ~58×62 m trajectories with Z ≤ 0.7 m — any of the three would be equivalent.

---

## floor_4

### floor_4 / 2025-12-02 / run_1

**Plot:** `plots_final/floor_4/floor_4_2025-12-02_run_1_configs.png`  
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | Status |
|--------|----------|---|--------|-------|--------|
| baseline | 1× | 4833 | 27915×17305 | 13028.3 | DIVERGED |
| dynA | 1× | 3442 | 40×25 | 0.8 | **OK** |
| imuB | 1× | — | — | — | empty |
| expC | 0.5× | 5496 | 6705×6726 | 3768.3 | DIVERGED |
| dynA | 0.5× | 2287 | 70115×30298 | 145976.9 | DIVERGED |
| fastinit | 0.5× | 5159 | 29×37 | 0.7 | **OK** |
| submission (dynA 1× upsampled) | — | 5529 | 40×25 | 0.8 | OK |

**Selected:** `dynA (1×)` — current submission. fastinit slowrate also OK (5159 poses, 29×37 m, Z 0.7 m) but shape differs from dynA; no GT to arbitrate.

**Notes:** baseline and expC diverge. imuB 1× empty. dynA slowrate diverged catastrophically (145 km spread). slowrate_imuB never run.

---

## floor_3

### floor_3 / 2025-05-19 / run_1

**Plot:** `plots_final/floor_3/floor_3_2025-05-19_run_1_configs.png`  
**GT available:** no

| Config | Slowrate | n | XY (m) | Z (m) | ATE | Status |
|--------|----------|---|--------|-------|-----|--------|
| baseline | 1× | 1933 | 27×38 | 2.9 | — | OK |
| baseline | 0.5× | 3470 | 23×39 | 0.7 | — | **OK** |
| expC | 0.5× | 3275 | 11420×6557 | 1983.2 | — | DIVERGED |
| dynA | 0.5× | 3438 | 24×38 | 0.6 | — | **OK** |
| fastinit | 0.5× | 3450 | 34480×5773 | 10520.4 | — | DIVERGED |
| imuB | 0.5× | 3469 | 24×38 | 0.6 | — | **OK** |
| submission (dynA 0.5×, raw) | — | 3438 | 24×38 | 0.6 | — | **OK** |

**Selected:** `dynA 0.5×` — swapped into submission as raw (no upsample). Init delay 3.2 s is within the first 5 s which is excluded from scoring, so no coverage penalty. Raw coverage ~95.9% (3438 poses over 119.5 s bag at 30 Hz); the evaluation skips the first 5 s, making effective coverage ~100% of the scored window.

**Notes:** expC and fastinit slowrate both diverged (same as before). Three slowrate survivors (baseline, dynA, imuB) all produce consistent 24×38 m trajectories with Z < 1 m. dynA chosen; any of the three would be equivalent. Previous submission used baseline 1× upsampled with ~55.9% raw coverage and Z 3.6 m from extrapolation — both worse.

---

## floor_1

### floor_1 / 2025-12-02 / run_1

**Plot:** `plots_final/floor_1/floor_1_2025-12-02_run_1_configs.png`

| Config | Slowrate | n | XY (m) | Z (m) | Status |
|--------|----------|---|--------|-------|--------|
| baseline | 1× | 5876 | 16252×34841 | 2908.6 | DIVERGED |
| dynA | 1× | 6313 | 32×43 | 1.4 | **OK** |
| imuB | 1× | 5787 | 61600×24386 | 26472.7 | DIVERGED |
| expC | 0.5× | 8147 | 17309×21725 | 9714.4 | DIVERGED |
| submission (dynA upsampled) | — | 8402 | 32×43 | 1.4 | **OK** |

**Selected:** `dynA (1×)` — only non-diverged result. Coverage ~77.4%; no other config produced a usable trajectory. Submission uses the upsampled version.

**Notes:** expC diverged at slowrate despite having more wall-clock time per frame, suggesting the combined IMU noise + feature changes in expC destabilise this bag specifically. dynA's dynamic initialiser is the key enabler here.
