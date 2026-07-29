# AjouNice BEVFusion runtime

This directory builds and runs the four-camera MORAI BEVFusion inference
process. Sensor geometry comes only from `configs/sensors/Sensor.json`.
Algorithm policy comes from `configs/perception/bevfusion_sensor4.yaml`.

The ROS host connects to a port published only on `127.0.0.1`. The simulator
boundary remains the competition-provided UDP interface. See
`docs/bevfusion/260720_morai_four_camera_zero_shot_full.md` for the protocol,
competition-rule caveat, and the Unix-domain-socket fallback.

```bash
# Create the local Compose environment file once.
cd ~/AjouNice2026/docker/bevfusion
cp .env.example .env

# Start the model server.
docker compose up -d inference
docker compose logs -f inference

# Check protocol and model readiness.
PYTHONPATH=. python3 scripts/healthcheck.py
```

Set `PROJECTION_ONLY=true` in `.env` to validate calibration without loading
the model checkpoint.

