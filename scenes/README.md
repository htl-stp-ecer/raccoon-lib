# Simulation Scenes

This directory holds `.ftmap` scenes loaded by the mock platform's `SimWorld`.

## Format

Scenes use the `TableMapFileV1` schema produced by the web-ide's map editor.
It's JSON (which parses through yaml-cpp as YAML flow syntax):

```json
{
  "format": "flowchart-table-map",
  "version": 1,
  "table": { "widthCm": 200, "heightCm": 100 },
  "lines": [
    { "kind": "line", "startX": 50, "startY": 50, "endX": 150, "endY": 50, "widthCm": 1.5 },
    { "kind": "wall", "startX": 0,  "startY": 0,  "endX": 0,   "endY": 100, "widthCm": 0 }
  ]
}
```

Fields:
- `table.widthCm` / `heightCm` — table dimensions.
- `lines[].kind` — `"line"` (black line, sampled by line sensors) or `"wall"` (collision).
- `startX/Y` / `endX/Y` — endpoint coordinates in cm. **Origin is the top-left**
  (matches the web-ide editor). `WorldMap::loadFtmap` flips Y at load time so the
  in-memory representation has origin bottom-left (standard math).
- `widthCm` — line thickness (for collision expansion on walls, for hit-test
  threshold on lines, minimum threshold 0.75 cm).

## Authoring

Either:
1. Use the web-ide's map editor, save to the robot, copy the `.ftmap` here, OR
2. Hand-edit the JSON.

## Fixtures in this directory

- `empty_table.ftmap` — bare 200×100 cm table, no walls or lines. For drive/turn tests.
- `single_line.ftmap` — one horizontal 100 cm black line across the middle. For lineup tests.
- `wall_box.ftmap` — 100×100 cm table with a box of internal walls. For collision tests.
