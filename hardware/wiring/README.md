# WireViz Wiring Diagrams

WireViz is installed locally in the repository at `venv/wireviz`. The
environment is ignored by Git and does not modify the Pi's system Python.

Run WireViz from the repository root through the wrapper:

```bash
./scripts/wireviz.sh --version
```

Each diagram has its own subject folder. The editable YAML source and its
generated HTML, PNG, SVG, and bill of materials stay together:

- `power/` — battery, protection, distribution, converter, Pi, and Due path
- `motor-encoder/` — both six-wire motor and encoder groups
- `control-logic/` — complete and focused Due, driver, and translator views
- `pi/` — Pi power alternatives, installed storage/cooling, direct Due and
  future LiDAR links, and the owned `#2998` hub feeding the BNO055 branch
- `examples/smoke-test/` — local installation test only

Connection coverage against the authoritative interface matrix is tracked in
`coverage.md`, which also records where every owned wiring/connector BOM item
is used or why it remains spare.

Render or refresh a diagram from the repository root:

```bash
./scripts/wireviz.sh \
  --format hpst \
  hardware/wiring/power/main-power-path.yml
```

Without `--output-dir`, WireViz writes the generated files beside the YAML
source in that diagram's folder.

Graphviz is a separate WireViz prerequisite. This Pi already provides `dot` at
`/usr/bin/dot`.
