# Team evaluation baselines

Official `2026_molit_comp_sample_scene.json` derivatives for human-driven
evaluation-bag recording. The source scenario is not modified.

## Shared contract

- Ego starts in Keyboard/manual control mode, stationary, and in `P`.
- Every original sample actor and mission remains in the scenario.
- Only one additional mission element is added per family.
- Each family has four files whose only intended difference is the parked Ego
  start pose.

## Families

### `sample_gps_shadow_obstacle_*`

- Clones one official static-obstacle asset (`DataID=40100001`, scale
  `2 x 3 x 2`) into the GPS-shadow corridor.
- The obstacle is aligned with the road rather than placed across it.

### `sample_gps_shadow_vehicle_*`

- Adds one SpawnPoint near the entrance of the GPS-shadow corridor.
- Spawn speed is `0 km/h`, target speed is `50 km/h`, and maximum live vehicle
  count is one.
- Destination mode is `DESTROY`, with the destination near the exit of the
  GPS-shadow corridor.
- The generated start and end are on the same directed MGeo link through the
  shade area. This imitates the GPS-shadow vehicle-interaction mission; it is
  unrelated to the overall competition START and END.

## Ego anchors

| Suffix | Approximate course position |
| --- | --- |
| `ego_start` | Official START pose. |
| `ego_signal_to_roundabout` | Between the signal mission and roundabout. |
| `ego_roundabout_to_highway` | Between the roundabout and highway merge. |
| `ego_surround_to_gps_shadow` | Between surrounding-vehicle recognition and GPS shadow. |

`manifest.json` records the exact generated positions, headings, MGeo route
audit, and added actor settings.

## Rebuild

```bash
cd ~/AjouNice2026
python3 dataset/morai_od/scripts/build_team_evaluation_scenarios.py
```

Copy the selected JSON files into the personal launcher scenario directory,
then load them from MORAI Scenario Edit.

```bash
cp configs/scenario/team_evaluation_baselines/*.json \
  ~/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/
```
