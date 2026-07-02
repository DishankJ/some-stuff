# Camera Network Design for Horizontal RLV Landing Tracking

I went through all three papers plus derived the rest from first-principles stereo photogrammetry. Your setup (E-W stereo gates straddling the centerline, RLV flying N→S) is actually a **converging "gate" stereo geometry**, which is different from all three papers' configurations — Tong et al. use side-by-side stacked pairs looking *across* a vertical trajectory, Sumetheeprasit et al. use parallel wide-baseline UAV pairs, and Xu et al. use a trinocular in-line rig. Your gate geometry is actually *favorable* for your primary goal (cross-track/centerline deviation), for reasons I'll explain.

## 1. Assumptions I'm using (state your real numbers and I'll redo the math)

Since you didn't give vehicle specifics, I assumed a generic horizontal-landing RLV in the Shuttle/Dream Chaser/Buran class:

| Parameter | Assumed value |
|---|---|
| Length / span | ~15–25 m |
| Approach/flare speed | 90–110 m/s |
| Touchdown speed | 70–90 m/s |
| Rollout deceleration | ~1.5–2.5 m/s² |
| Flare-phase altitude | 0–40 m AGL |
| Zone of interest (flare→touchdown→initial rollout) | ~500–1500 m of runway |

## 2. Why the gate geometry helps you specifically

In your layout, the camera-to-camera baseline vector points **E-W — exactly the axis you care about (centerline deviation)**. In stereo triangulation, the coordinate *along* the baseline is constrained directly by the ray-intersection angle and its error scales roughly with **GSD (ground sample distance) = z·p/f**, not z². The coordinate along the *depth/bisector* axis (here, roughly N-S / down-range) is the one that suffers the z²/(b·f)·εd blow-up described in Sumetheeprasit et al. (Eq. 2) and Xu et al. (Eq. 1). That's good news: your highest-value measurement (E-W deviation) is the cheap, high-accuracy one; your lower-priority measurement (down-range position) is the noisier one — but you have redundancy there anyway because you'll have multiple gates.

## 3. Camera specifications

Following Tong et al.'s design choices (global-shutter CMOS, hardware-triggered sync, µs-level jitter) but re-derived for your speed regime:

**Frame rate / shutter — driven by motion, not by the rocket-descent logic in Tong (which assumed ~2 m/s descent)**
- Motion blur budget: keep blur ≤ 1 pixel. `t_exposure ≤ GSD / v`. At GSD ≈ 2 cm/px and v = 100 m/s → **t_exposure ≤ ~200 µs**. Use a global-shutter machine-vision sensor capable of electronic shutter down to tens of µs (same class as the LUPA1300 Tong et al. used).
- Frame rate: you want enough samples across each gate's field of view for the tracking algorithm (Tong's scale-adaptive block matching) to keep inter-frame displacement small. At v = 100 m/s and ~20 m of along-track coverage per gate (see §5), you have the vehicle in-frame for only ~0.2 s — you need enough frames in that window to fit a trajectory. **200–500 fps** is a reasonable range; go toward 500 fps if you want sub-frame velocity/attitude-rate estimates through the gate, or if your vehicle is faster than assumed.
- **Recommend: 1280×1024–2048×1536 px, global shutter, 200–500 fps, exposure ≤ 200 µs, hardware-triggered sync (µs-level) across all gates**, exactly as Tong et al. implemented for their 200 Hz network.

**Resolution / focal length — driven by desired centerline-accuracy**

Using GSD = z·p/f (z = standoff, p = pixel pitch, f = focal length):

| Standoff z | Focal length f | Pixel pitch | GSD (≈ E-W accuracy) | H-FOV coverage | V-FOV (along-track) coverage |
|---|---|---|---|---|---|
| 25 m | 35 mm | 14 µm | ~1 cm/px | ~13 m | ~10 m |
| 25 m | 16 mm | 14 µm | ~2.2 cm/px | ~28 m | ~22 m |
| 40 m | 25 mm | 14 µm | ~2.2 cm/px | ~28 m | ~23 m |

This is the key tradeoff you'll face: **tighter focal length = better centerline accuracy but shorter along-track coverage per gate (more gates needed)**. Given that fault-analysis-grade centerline deviation is usually acceptable at a few cm (not sub-mm, unlike Tong's static-rocket case), I'd lean toward the 16–25 mm option — it buys you much wider along-track coverage per gate for a manageable accuracy cost.

## 4. Stereo pair geometry (baseline & standoff)

- **Baseline** = full E-W separation between the two cameras of a gate. Since you're straddling the centerline, baseline ≈ 2 × (camera standoff from centerline). For a runway/landing corridor with margin, this typically works out to **30–50 m**.
- **Standoff** (camera to centerline) ≈ half the baseline, i.e., **15–25 m**, chosen for safety clearance and to keep the vehicle inside the vertical FOV during flare (when it's higher above the ground).
- **Convergence/toe-in angle**: aim each camera slightly inward so the optical axes intersect near the expected flight-path centerline at the nominal altitude for that gate's phase (steeper convergence for touchdown-zone gates near ground level, shallower for flare-zone gates where altitude varies more). This is analogous to the "inward yaw tilt" Sumetheeprasit et al. used to fight baseline-induced occlusion (their Eq. 4-ish tilt-per-meter heuristic) — except here your convergence is driven by geometry (aim at centerline), not by an anti-occlusion correction.
- **Do not go past baseline/standoff ratios that create the occlusion problem Xu et al. describe** (very wide baseline relative to range → one camera loses line-of-sight to the underside/near-side of the vehicle, especially during a crosswind crab angle where the fuselage yaws relative to the runway heading). With b ≈ 30–50 m and z ≈ 15–25 m, your b/z ratio (~1.5–2) is on the wide side — worth adding a **third, narrow-baseline camera per gate** (trinocular, per Xu et al.) to recover occluded-side geometry during bank/crab, the same way their multi-view module fills in wide-baseline occlusion using a narrow-baseline "helper" view.

## 5. Along-track range usable per stereo pair before handoff

This is bounded by the camera's vertical FOV footprint at the relevant standoff (Table above): **~10–25 m of along-track coverage per gate**, depending on the focal length you pick. Beyond that, oblique viewing angle degrades both resolution and the assumption of near-vertical rays, so accuracy drops even before the vehicle physically exits the frame.

Handle the handoff exactly as Tong et al. do for their stacked-FOV crossover (Section 2.3, Step 6): use the known relative extrinsics between adjacent gates to predict where the vehicle will enter the next gate's frame, and use a Kalman filter to bridge any gap where FOVs don't overlap, rather than requiring hard geometric overlap everywhere.

## 6. Spacing between successive stereo gates

Set gate spacing ≈ (along-track coverage per gate) with a **10–20% overlap** for reliable handoff and continuous target re-acquisition (mirroring Tong's overlap requirement for the up/down FOV crossover). Given the numbers above:

- Tight-accuracy configuration (35 mm, z=25 m): **~8–9 m spacing** (dense — appropriate for a short, high-value touchdown zone or a *scaled* test article, similar in spirit to Tong's scaled-rocket test which only spanned 10 m of altitude with 4 cameras)
- Wider-FOV configuration (16–25 mm, z=25–40 m): **~18–20 m spacing**

**Practical recommendation:** don't use uniform spacing over the whole runway. Use the tight/dense configuration only over the flare + touchdown + initial rollout zone (say, the critical ±300–500 m around nominal touchdown point) where centerline control is most safety-critical, and switch to wider-FOV, sparser gates (every 50–100 m) for the remaining rollout distance where you mainly want gross trajectory/deceleration monitoring rather than cm-level centerline accuracy. This is the same "match sensor allocation to what you actually need to measure" logic behind Sumetheeprasit et al.'s multi-baseline simultaneous-coverage scheme (their Fig. 10, using different baselines to cover different depth bands at once) — here applied along-track instead of in depth.

## 7. Summary table

| Parameter | Recommended |
|---|---|
| Sensor | Global shutter CMOS, 1280×1024–2048×1536 |
| Frame rate | 200–500 fps |
| Exposure | ≤ 200 µs (electronic shutter) |
| Sync | Hardware trigger, µs-level jitter across all gates |
| Lens (touchdown zone gates) | ~30–35 mm |
| Lens (rollout zone gates) | ~16–25 mm |
| Baseline per gate | 30–50 m (2× standoff) |
| Standoff per gate | 15–25 m from centerline |
| Toe-in/convergence | Aimed at expected flight-path altitude for that gate's phase |
| Along-track coverage/gate | 10–25 m depending on lens |
| Gate spacing, touchdown zone | ~8–20 m (dense, overlapped) |
| Gate spacing, rollout zone | 50–100 m (sparse) |
| Occlusion mitigation | Add 3rd narrow-baseline camera per gate (Xu et al. trinocular concept) for crab-angle/bank occlusion recovery |
| Targets | Circular retro-reflective/white targets (Tong et al.) if surface allows, or CNN keypoint tracking on natural features (nose, wingtips, gear) if surface is tiled/textured like a real thermal-protection skin |

If you give me the actual vehicle (length, touchdown speed, expected deviation tolerance, and runway/test-track length), I can redo this with exact numbers rather than the representative values above — the formulas (GSD = z·p/f, εz = z²εd/(bf), motion-blur budget) are all reusable directly.
