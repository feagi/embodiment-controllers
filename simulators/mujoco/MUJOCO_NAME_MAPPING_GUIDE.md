# MuJoCo Name Mapping Guide

This guide defines how to create and maintain human-readable naming for MuJoCo models used by the FEAGI embodiment controller.

## Goal

Keep model XML files unchanged, while providing readable labels for:

- motor channels (actuators)
- sensor channels
- joint-derived channels (for source/entity labeling)

The controller reads model-local mapping files and applies those labels during registration payload generation.

## Current design (authoritative)

- Mapping file is **per model**.
- Mapping file lives in the **same folder as the model entry XML**:
  - `.../embodiments/<mujoco_model>/model/mujoco_feagi_mappings.json`
- Mapping is **explicit only** (exact key -> exact value).
- No decoder-ring/token expansion fallback should be relied on.
- If a key is absent, the controller falls back to the original MuJoCo name.

## Mapping file schema

```json
{
  "schema_version": 1,
  "joints": {},
  "actuators": {},
  "sensors": {},
  "source_entities": {}
}
```

### Section semantics

- `joints`: readable names for MuJoCo joint identifiers.
- `actuators`: readable names for MuJoCo actuator identifiers (FEAGI motor-facing labels).
- `sensors`: readable names for sensor identifiers (including derived names like `jointpos_*`, `jointvel_*`, `actuatorfrc_*` if you want custom overrides).
- `source_entities`: optional explicit override map for source labels.

## Naming conventions

- Use lowercase snake_case.
- Prefer concise anatomy-first naming:
  - `left_front_hip_abduction_adduction`
  - `right_hind_knee_flexion_extension`
- Avoid internal simulator graph terms in UI labels when possible (`virtual_*`, `eq_*`, etc.).
- For repeated chains, use stable index conventions:
  - `toe_3_segment_2`
  - `toe_2_reverse_segment_1`

## Workflow for a new model

1. Identify the model entry XML (the file passed to `--model_xml`).
2. Create `mujoco_feagi_mappings.json` in that entry XML folder.
3. Populate all four sections.
4. Start with exact keys discovered from XML/runtime names.
5. Replace cryptic keys with curated readable values.
6. Keep labels stable over time (avoid churn once external tools rely on them).

## Example

```json
{
  "schema_version": 1,
  "joints": {
    "LF_HAA": "left_front_hip_abduction_adduction"
  },
  "actuators": {
    "LF_HAA": "left_front_hip_abduction_adduction"
  },
  "sensors": {
    "jointpos_LF_HAA": "servo_encoder_position_left_front_hip_abduction_adduction",
    "jointvel_LF_HAA": "joint_velocity_left_front_hip_abduction_adduction"
  },
  "source_entities": {}
}
```

## Validation checklist

- JSON parses successfully:
  - `python3 -m json.tool /path/to/mujoco_feagi_mappings.json >/dev/null`
- Controller can start with model and no mapping-load warning.
- Hover/registration labels in BV show mapped values as expected.
- No accidental fallback-only files for intended curated models.

## Important notes

- Do not modify the model XML just to improve naming.
- Keep mapping files model-local and independent.
- If actuator names differ from joint names, map both sections explicitly.
- For heavily engineered models (soft robotics, tendon/virtual chains), prioritize human readability over internal mechanism verbosity.

