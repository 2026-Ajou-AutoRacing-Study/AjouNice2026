# Repository context router

- Start with [docs/README.md](docs/README.md), then read only the requested domain's
  current router and its local `AGENTS.md`. Do not load all historical reports.
- Current state belongs in each domain's `CURRENT` document. A dated report is
  evidence for its recorded revision, not authority for today's runtime defaults.
- `legacy/` is historical material, excluded from ordinary `rg` discovery by
  `.ignore`. Inspect it explicitly only for a concrete reproduction or comparison.
  See [archive policy](docs/maintenance/CONTEXT_MAINTENANCE.md).
- Perception: [docs/perception/README.md](docs/perception/README.md).
- Dataset: [dataset/morai_od/docs/README.md](dataset/morai_od/docs/README.md).
- Localization: [docs/localization/README.md](docs/localization/README.md).
- Map: [kcity_map/README.md](kcity_map/README.md).
- Sensor/network: their own `configs/*/README.md`; do not infer profiles from a
  numbered filename, a historical screenshot, or the newest modification time.
- Planning/control are separately owned. Read their local instructions and avoid
  changing them during another domain's cleanup or optimization.
- Preserve unrelated dirty/staged work. Do not change raw data, native GT, frozen
  collector inventories, resume ledgers, model files, or submodules for doc cleanup.
- Distinguish source, installed build, candidate tests, and production settings.
  Moving documentation must not silently change launch defaults or live processes.
- On luv's workstation, use the private-OAuth Drive remote `ajou_drive_private`.
  If authentication expires, request reconnect; do not silently fall back to a
  public/shared OAuth client. Other PCs may use different explicit remote names.
