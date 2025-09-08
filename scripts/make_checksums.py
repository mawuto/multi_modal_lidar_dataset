#!/usr/bin/env python3
import hashlib, sys, pathlib, json
def sha256sum(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1<<20), b''):
            h.update(chunk)
    return h.hexdigest()
root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "dataset")
records = {}
for p in root.rglob("*"):
    if p.is_file():
        records[str(p)] = {"sha256": sha256sum(p)}
print(json.dumps(records, indent=2))
