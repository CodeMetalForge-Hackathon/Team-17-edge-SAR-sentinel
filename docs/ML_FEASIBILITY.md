# ML and data feasibility plan

Checked on 2026-11-07 against the sources linked below. Nothing has been downloaded or run: no checkpoint,
no inference and no accuracy result exists yet. All "verified" entries mean an API, file listing or page
confirmed the fact. Licence notes are facts about what each source says, not legal advice.

## 1. Decision

**Choose Option A: an offline pre/post-disaster building damage module, cached.**

| | A: Offline damage context | B: Live scene change or close-up defects |
|---|---|---|
| Matches "reference imagery gives context, local observations update it" | Yes, directly | Only the second half |
| Compatible pretrained weights | **Yes**, ChangeOS Swin-T on xBD | Crack: none official. Scene change: needs no model |
| New value over what we have | A real pre/post damage model and a recommendation that updates | Scene change repeats LiDAR change detection; cracks need a close-up camera we do not have |
| Hardware risk | None, runs on a laptop and is cached | Camera is already owned by the Hailo pipeline. Venue lighting changes will cause false alarms |
| Can it break the live demo? | No, it reads a file | Yes, it adds load and alarms on the live path |
| Weakness | Not live. Satellite imagery does not transfer to our set, and the zone link is staged | Weak link to disaster response; a crack does not establish stability |
| Licence | xBD is CC BY-NC-SA 4.0. **Needs a human decision** | SDNET2018 is CC BY 4.0 (clean) |

**Why B loses even though A is staged.** B's scene-change variant has no model and duplicates LiDAR. Its crack
variant needs a new close-up camera, training, a Hailo compile, and then would still only show a crack on
cardboard. A is honest about what it is and cannot affect the live path, which keeps hardware integration first.

**What the demo can truthfully say:** "This model ran on a laptop beforehand, on real satellite pairs from a
public dataset. These results are cached. We manually associated this tile with this mock zone for the
demonstration. Live sensors then reorder the recommendations, and they alone can create a hazard or dispatch."

## 2. Source comparison

| Source | Input | Access (verified) | Size (verified) | Licence as stated | Usable checkpoint? |
|---|---|---|---|---|---|
| [xBD / xView2](https://xview2.org/dataset) | Satellite pre + post, 6 disaster types | Official route needs registration | Archive size not visible without registering | **CC BY-NC-SA 4.0** | See below |
| [EVER-Z/torchange_xView2](https://huggingface.co/datasets/EVER-Z/torchange_xView2) (mirror) | Same data as parquet: images, masks, `image_name` | Ungated. Row API works | 24.3 GB; hold is 933 rows in 6 shards of about 470 MB | **None declared.** Contents still derive from xBD | n/a |
| **[ChangeOS Swin-T](https://huggingface.co/EVER-Z/torchange_example_changeos_swint_on_xview2_best42k)** | 6-channel pre+post RGB | **Ungated, HEAD request returned 200** | **166 MB** safetensors | Card says Apache-2.0. **Training data is NC-SA** | **Yes, selected** |
| [xView2_baseline](https://github.com/DIUx-xView/xView2_baseline) | Pre + post | Release v1.0 reachable (200) | 497 MB + 69 MB | Code BSD-3 style | Old Keras/TF stack. Rejected: harder to run |
| [xView2_first_place](https://github.com/DIUx-xView/xView2_first_place) | Pre + post | GitHub split weights listed. **S3 link returned 403** | About 5.2 GB | Code MIT | Rejected: large, NVIDIA apex |
| [RescueNet](https://github.com/BinaLab/RescueNet-A-High-Resolution-Post-Disaster-UAV-Dataset-for-Semantic-Segmentation) | UAV, **post only**, 4,494 images | Dropbox or figshare | Not checked | Dataset **CC BY-NC-ND** (no derivatives). Repo shows MIT, so keep them separate | None in repo. Rejected |
| [BRIGHT](https://github.com/ChenHongruixuan/BRIGHT) | Pre optical + **post SAR** | HF `Kullervo/BRIGHT` ungated; [Zenodo checkpoints](https://zenodo.org/records/15349462) | 27.9 GB (HF); 0.12-0.17 GB per checkpoint | HF card CC BY-SA 4.0; Zenodo checkpoints CC BY 4.0; some regions' optical data cannot be redistributed | Yes. Includes zero-shot held-out-event checkpoints. **License-cleaner alternate**, more preprocessing, not selected |
| [CRASAR-U-DROIDs](https://huggingface.co/datasets/CRASAR/CRASAR-U-DROIDs) | Orthomosaics: satellite pre and post, crewed and sUAS post; per-building JDS labels with lat/lon | Ungated | 218.8 GB; median GeoTIFF about 61 MB | **CC BY 4.0** | None. **Best ground-truth fallback** |
| [SDNET2018](https://doi.org/10.15142/T3TD19) | Close-up concrete crack images | Direct download | 516 MB | CC BY 4.0 | None official. Option B only |

**Hailo.**
- The zoo lists [no change-detection model for Hailo-8L](https://github.com/hailo-ai/hailo_model_zoo/tree/master/docs/public_models/HAILO8L), though it does list person detection and ordinary segmentation.
- A custom model would need the Hailo Dataflow Compiler, Hailo account access and an x86 Linux compile. **A 6-channel Swin-T ChangeOS is not assumed to compile.** We will not attempt it.
- Hailo-8/8L use Model Zoo v2.x with the v3.x compiler per the zoo README. `hailo-all` is for AI Kit and AI HAT+, and `hailo-h10-all` is for AI HAT+ 2. They cannot coexist.
- `hailo-apps` main says it was most recently tested with HailoRT v5.4.0. **Whether that matches the `hailo-all` package for our HAT is unresolved** and is checked in HW-B2.

## 3. Runtime separation

| Where | Runs | Status |
|---|---|---|
| Core `.venv` | `pi/` services. No torch | DRAFTED (BS-00) |
| Separate `ml/.venv` (Python 3.11+ required; laptop has 3.11.9) | torch CPU, torchvision, torchange, pyarrow. Never imported from `pi/` | PROPOSED |
| Laptop inference | **Selected** | Unmeasured |
| Pi CPU inference | Stretch only, after everything else, and only with a measured number | Not planned |
| Hailo inference | **Not planned** | n/a |

The laptop's global Python has CPU torch 2.10 and no torchvision, and it has an RTX 5070 8 GB. CPU comes first
because results are cached anyway. Use CUDA only if CPU misses the gate, and only with a new budget.

## 4. Spike procedure

**Pins.** All verified as of this pass.

| Item | Pin |
|---|---|
| Code | `Z-Zheng/pytorch-change-models` at commit `190b07a2eb4c389a5dc3f72802c67f428cdfd6b7` |
| Checkpoint | revision `4007dd10c5eaee67161e2d4a45a43da3ead6c722`, sha256 `81ae6386ceebe225ab6c072cb987b57312fdcfdb2a7f72ab9b30e5e8f1041cc4` |
| Mirror | revision `cda47e44377258222166dac63a85068e8bdca446` |
| Data shard | `data/hold-00000-of-00006.parquet`, 477.3 MB, sha256 `c80ff36508bde5de1a6c8a56f4f36780228956c3195448ab4c5d60839b8412bc` |

**Why the code commit and not PyPI.** PyPI's `torchange` 0.0.4 dates from January 2026, and the checkpoint card
from August 2026. The card's `ChangeOS.from_pretrained` may need the later code. **Unresolved until the spike runs.**

**Commands (human-run, once approved).** Run from the repo root in PowerShell.

```bash
python -m venv ml/.venv
```

```bash
ml/.venv/Scripts/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

```bash
ml/.venv/Scripts/python -m pip install "git+https://github.com/Z-Zheng/pytorch-change-models@190b07a2eb4c389a5dc3f72802c67f428cdfd6b7" pyarrow pillow huggingface_hub safetensors
```

```bash
ml/.venv/Scripts/python -m pip freeze > docs/evidence/ML-01/pip-freeze.txt
```

```bash
ml/.venv/Scripts/python -c "from huggingface_hub import hf_hub_download as d; print(d('EVER-Z/torchange_example_changeos_swint_on_xview2_best42k','model.safetensors',revision='4007dd10c5eaee67161e2d4a45a43da3ead6c722',local_dir='ml/cache/ckpt')); print(d('EVER-Z/torchange_xView2','data/hold-00000-of-00006.parquet',repo_type='dataset',revision='cda47e44377258222166dac63a85068e8bdca446',local_dir='ml/cache/data'))"
```

**Inference outline** (`ml/run_inference.py`, written by the builder):

1. Refuse to run if the sha256 values above do not match.
2. Read the parquet with pyarrow. Seed 2026. Take the first 24 rows after a seeded shuffle of `image_name`, capped so no event is more than half. Write the list to `sample_ids.json` **before any inference**.
3. Preprocess per the card: ImageNet normalization. Stack pre and post into `(B, 6, H, W)`. Native tile size is checked and recorded, not assumed.
4. Apply the card's postprocess: `loc = t1_semantic_prediction > 0.5`, `dam = loc * change_prediction.argmax(1)`, with `B=1`.
5. Record wall time and peak RSS per tile, device name and thread count.
6. Compute the metrics in section 5. Write overlays and `damage_context.json` with full provenance.

## 5. Evaluation, and what it can and cannot show

- **Split:** `hold` only. The card says the checkpoint trained on `train` + `tier3` and was selected on `test`.
- **Event separation is not available here.** xBD's splits appear to be by image and not by event: sampled `hold` names include `socal-fire`, `midwest-flooding` and `hurricane-florence`, which are events the checkpoint likely saw. So neither the card's hold F1 of 76.91 nor our number says how it does on **a new disaster**. ML-01 lists event names per split to confirm this.
- **Optional stretch (not part of the gate):** one satellite pre/post pair from a CRASAR test-split disaster that xBD lacks, such as Hurricane Idalia or the Mayfield tornado, as a true new-event look. Budget at most 1 GB. Results are descriptive only, n is tiny, and alignment is untested.
- **Report:** per-event and pooled building-pixel F1 and per-class pixel counts with n stated. Label as "pixel metric on N tiles, shard 0 of hold, not comparable to the official xView2 score".
- **Demo examples are chosen after evaluation** and shown as "selected examples", never as accuracy.
- **No transfer claim.** Satellite damage models do not apply to an indoor camera or a cardboard set. We do not run this on camera frames.

### Gate: go or no-go for ML-02

All must hold, by **Sat 18:00 or 3 hours of work, whichever comes first**.

| # | Check | Pass |
|---|---|---|
| G1 | Human approved budget and licence route in writing | Record in `docs/evidence/ML-01/decision.md` |
| G2 | Install and load from pinned sources, hash match | Evidence file |
| G3 | Inference completes for all 24 tiles offline after caching (Wi-Fi off for at least one tile) | Evidence file |
| G4 | Measured CPU time per tile at most 120 s | Measured |
| G5 | Sanity: pooled building-pixel F1 at least 0.5 on the 24 tiles, **and** output is not all one class. Threshold is a garbage filter chosen before running, not a quality claim | Metrics file |
| G6 | A human looks at 6 pre-selected overlays and agrees they look plausible | Human note |
| G7 | Checkpoint 1 hardware is not being starved by this work | D's call |

**If G2-G5 fail: no retries past the budget.** Go to fallback 1, ground truth only.
**If G1 fails on licence grounds:** use CRASAR ground truth (CC BY 4.0), or evaluate BRIGHT only with a new budget.
**Fallback 2:** cut the ML panel. Say in the demo that it was tried and did not make it.

## 6. Budgets proposed for human approval

**Not approved yet.** Paid API use: none.

| Item | Proposed limit |
|---|---|
| Wall time | 3 hours |
| Downloads | Python packages 2.5 GB, checkpoint 166 MB, data 478 MB: **at most 3.2 GB total** |
| Inference runs | At most 3 full runs of the 24 tiles |
| Disk | At most 6 GB in `ml/`, gitignored |
| Laptop CPU | One laptop, plugged in |
| Stretch CRASAR | At most 1 GB, only if everything else passed and D agrees |

**Licence decision the human must record** (in `decision.md`):

> Is CC BY-NC-SA 4.0 material acceptable for this hackathon demo? Who confirmed? Attribution text agreed?
> Do we use the ungated mirror, or the official registered download?

**Proposed default, pending human approval:** use the mirror at the pinned revision. Attribute xBD. Commit no
imagery, only derived JSON. Disclose the licence in `THIRD_PARTY.md` and on the panel, and state the checkpoint's
Apache-2.0 tag separately from its NC-SA training data. **Whether the training data's terms reach the
checkpoint is unresolved.** Treat use as non-commercial demonstration only.

## 7. Provenance and repository rules

- Commit `damage_context.json`, `zone_association.json`, metrics and `sample_ids.json` only if the human agrees.
- **Never commit** imagery, overlays, parquet or weights: `ml/cache/`, `ml/.venv/`, `demo_assets/images/` are gitignored in the BS-00 draft.
- Overlays copied to the Pi go in `demo_assets/images/` by hand, listed by sha256 in `provenance.json`.
- `THIRD_PARTY.md` gets separate rows for code, checkpoint and dataset (INT-03).

## 8. Interface into the system

`demo_assets/damage_context.json`, schema `forge.damage_context/1`:

```json
{
  "schema": "forge.damage_context/1",
  "status": "CACHED",
  "kind": "prediction",
  "generated_at": "ISO-8601",
  "runtime": {"device": "laptop_cpu", "seconds_per_tile_median": 0.0, "peak_rss_mb": 0.0},
  "dataset": {"name": "xBD", "split": "hold", "source": "repo@revision", "license": "CC BY-NC-SA 4.0"},
  "checkpoint": {"repo": "...", "revision": "...", "sha256": "...", "license_declared": "apache-2.0",
                 "training_data_license": "CC BY-NC-SA 4.0"},
  "preprocessing": "text description, with versions",
  "samples": [{"sample_id": "hold/images/socal-fire_00000754", "event": "socal-fire",
               "class_pixels": {"no_damage": 0, "minor": 0, "major": 0, "destroyed": 0},
               "summary_class": "major", "overlay": "images/x.png", "selection": "eval"}]
}
```

`kind` is one of `prediction`, `ground_truth`, `simulated`. The numbers above are placeholders.

`demo_assets/zone_association.json`:

```json
{"association": "STAGED_MANUAL", "operator": "name", "note": "tile chosen for illustration, not a real location",
 "zones": [{"zone": "Z1", "name": "storefront", "polygon": [[1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0]],
            "sample_id": "hold/images/socal-fire_00000754"}]}
```

**Recommendation rules** (`pi/recon.py`, ML-03). First match wins for each zone, and zones rank by tier:

| Tier | Zone has | Recommendation | Note |
|---|---|---|---|
| 0 | Fresh person **and** hazard | `GROUND_TEAM_ONLY` | Drone held, as `decide()` already rules |
| 1 | Fresh person | `CONFIRM_SURVIVOR` | Live evidence beats any prior |
| 2 | Hazard, no person | `AVOID_HAZARD` | |
| 3 | Fresh obstruction | `CHECK_ACCESS` | Obstruction is not routed around |
| 4 | Prior major or destroyed, nothing fresh | `RECON_CANDIDATE` | Prior only, labelled with kind and sample ID |
| 5 | Otherwise | `NO_PRIORITY` | |

The list is advice to an operator. It publishes nothing to `forge/action`. It does not call `decide()`, and it
never changes `avoid_zones`. The `decide()` code path is untouched, and a test enforces this.
