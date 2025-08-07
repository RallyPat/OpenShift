# Clean-room Importer Spec

Goal: Allow users to convert proprietary calibration files to the open `CalibrationFile` JSON without bundling proprietary logic.

- Input: path to a vendor file
- Output: JSON to stdout matching `protocol::calibration::CalibrationFile`
- Exit status: 0 for success; non-zero for failure

Schema (abbrev):
- `schema_version`: string semver, e.g. "0.1.0"
- `metadata`: { `id`, `name`, `author?`, `description?`, `created_utc?` }
- `transmission`: { `model`, `gear_count` }
- `scalars`: [ { `name`, `unit?`, `value` } ]
- `tables`: [ { `name`, `x_axis`, `y_axis?`, `values` } ]
- `io_mapping`: { `analog_inputs`, `digital_inputs`, `pwm_outputs` }

Security & Legal:
- No reverse engineering guidance or proprietary schemas are included here.
- Users must ensure they have the right to transform their own files.
- Importers should be written from public info or user-owned data only.

Example CLI usage:

```
calibration-cli import --converter ./usshift_import_example.py --input MyCal.bin > MyCal.toml
```