#!/usr/bin/env python3
"""
Example clean-room importer. This does not parse any proprietary format.
It simply demonstrates the contract by producing a trivial calibration from an input path.
Replace the body with your own logic that you own and are authorized to use.
"""
import json, sys

if len(sys.argv) != 2:
    print("usage: usshift_import_example.py <file>", file=sys.stderr)
    raise SystemExit(2)

path = sys.argv[1]
name = path.split('/')[-1]
cal = {
    "schema_version": "0.1.0",
    "metadata": {"id": name, "name": f"Imported {name}"},
    "transmission": {"model": "unknown", "gear_count": 4},
    "scalars": [],
    "tables": [],
    "io_mapping": {"analog_inputs": [], "digital_inputs": [], "pwm_outputs": []},
}
print(json.dumps(cal))