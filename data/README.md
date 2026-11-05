# Data files

Everything the simulation needs, in files you can edit without touching the code.
A run is started from one run file:

```
cargo run -- data/runs/baseline.toml
```

```
data/
  cars/baseline.toml           mass, geometry, aero, brakes, powertrain, suspension
  tyres/baseline.toml          friction and wheel radius
  tracks/straight_1000m.csv    braking zones
  environments/standard.toml   air density and gravity
  runs/baseline.toml           picks the files above, time settings and start state
```

The baseline files hold exactly the values of the Rust `Default` impls, so the baseline run
gives the same results as the code defaults.

## Rules for every file

* Every TOML file starts with `format_version = 1`. Any other version is rejected.
* Every key has its SI unit in its name, written with the unit's real capitalisation:
  `N` newton, `Hz` hertz, `m` meter, `kg` kilogram, `s` second. `_per_` means divided by,
  `m2` / `m3` mean squared / cubed. Dimensionless values have no unit (`front_bias`, `mu_x_ref`).
* Every key is required unless this page says it is optional. A missing key, an unknown key
  (a typo, or a unit with the wrong capitalisation such as `spring_rate_n_per_m`) is an
  error that names the file and the key.
* Paths inside a file are relative to the folder of that file, so a run works from any
  working directory.
* After loading, the whole config is checked (`SimulationConfig::validate`, see README.MD),
  e.g. `front_bias` must be in 0..=1 and `front_kg + rear_kg` must equal `total_kg`.

## Run file (`runs/*.toml`)

| Key | Unit | Meaning |
|---|---|---|
| `format_version` | | Must be `1` |
| `car` | path | Car file |
| `tyres` | path | Optional. Tyre file. If missing, the car's `default_tyres` is used |
| `track` | path | Track CSV file |
| `environment` | path | Environment file |
| `start_time_s` | s | Start time of the simulation |
| `end_time_s` | s | End time of the simulation (it also stops when the car has braked to a stop) |
| `time_step_s` | s | Time step. Must be > 0 and satisfy the suspension stability limit omega_max * time_step < 2 (docs/11 section 11.8) |

`[initial_state]`: the starting conditions of the car at `start_time_s`.

| Key | Unit | Meaning |
|---|---|---|
| `speed_m_per_s` | m/s | Speed, >= 0 |
| `distance_m` | m | Distance along the track |
| `gear` | | Gear, 1 is first gear. Must exist in `gear_ratios` |
| `shift_time_left_s` | s | Time left of a gear shift in progress, >= 0. `0.0` = not shifting |

`[initial_state.suspension]`: body motion measured from static equilibrium (docs/11). All `0.0` = at rest.

| Key | Unit | Meaning |
|---|---|---|
| `heave_m` | m | Downward displacement of the CoG from static, positive down |
| `heave_velocity_m_per_s` | m/s | Heave velocity, positive down |
| `pitch_rad` | rad | Body rotation from static, positive nose down |
| `pitch_rate_rad_per_s` | rad/s | Pitch rate, positive nose down |

The rest of the car state is not written in the file, because it follows from the values above
and the car, track and environment. It is calculated before the run starts
(`SimulationConfig::prepare`): the active braking zone and whether the car is braking (from
`distance_m` and the track), the locked engine speed (from speed and gear, docs/12 equation 12.2)
and the acceleration. Writing one of them (e.g. `revs_hz`) is an unknown key error.

## Car file (`cars/*.toml`)

| Key | Unit | Meaning |
|---|---|---|
| `format_version` | | Must be `1` |
| `default_tyres` | path | Tyre file used when the run file has no `tyres` |

`[mass]` (docs/05, docs/06). The front/rear split also sets the CoG position (docs/10 section 10.6).

| Key | Unit | Meaning |
|---|---|---|
| `total_kg` | kg | Total mass of the car |
| `front_kg` | kg | Mass on the front axle |
| `rear_kg` | kg | Mass on the rear axle. `front_kg + rear_kg` must equal `total_kg` |

`[geometry]` (docs/10)

| Key | Unit | Meaning |
|---|---|---|
| `wheelbase_m` | m | Distance between the front and rear axle, > 0 |
| `cog_height_m` | m | Height of the centre of gravity above the ground, >= 0 |

`[aero]` (docs/04, docs/13)

| Key | Unit | Meaning |
|---|---|---|
| `drag_coefficient` | | Drag coefficient |
| `frontal_area_m2` | m² | Frontal area, reference area for drag and downforce |
| `downforce_coefficient` | | Downforce coefficient, positive pushes the car down |
| `downforce_centre_of_pressure_from_front_axle_m` | m | Where the downforce acts, measured back from the front axle |
| `drag_centre_of_pressure_height_m` | m | Height above the ground where the drag acts |

`[brakes]` (docs/09)

| Key | Unit | Meaning |
|---|---|---|
| `front_bias` | | Fraction of the brake force on the front axle, 0..=1 |

`[powertrain]` (docs/12)

| Key | Unit | Meaning |
|---|---|---|
| `torque_curve_Hz_and_N_m` | [Hz, N·m] | Full-throttle torque curve as `[engine speed, torque]` pairs, engine speeds strictly increasing, torques >= 0. Must cover `min_revs_Hz` to `max_revs_Hz` |
| `max_revs_Hz` | Hz | Rev limit |
| `min_revs_Hz` | Hz | Lowest engine speed before stalling, < `max_revs_Hz` |
| `primary_ratio` | | Ratio between the engine and the gearbox input, > 0. `1.0` = engine drives the gearbox directly |
| `gear_ratios` | | Gear ratios, first gear first, strictly decreasing, > 0 |
| `final_drive` | | Final drive ratio between the gearbox output and the wheels, > 0 |
| `shift_time_s` | s | Time without drive during a gear shift, >= 0 |
| `downshift_hysteresis_m_per_s` | m/s | Downshift happens this far below the upshift speed of the gear below, >= 0 |

The shift tables (clutch lock speeds and upshift speeds) are not in the file: they are
calculated from the torque curve, gearing and wheel radius before the simulation starts.

`[powertrain.drivetrain]`

| Key | Unit | Meaning |
|---|---|---|
| `type` | | `"rear_wheel_drive"`, `"front_wheel_drive"` or `"all_wheel_drive"` |
| `front_torque_split` | | Only with `"all_wheel_drive"`, and required there. Fraction of the drive force on the front axle, 0..=1 |

`[suspension]` (docs/11)

| Key | Unit | Meaning |
|---|---|---|
| `pitch_inertia_kg_m2` | kg·m² | Pitch moment of inertia of the body about its CoG, > 0 |
| `static_ride_height_front_m` | m | Front ride height standing still on level ground |
| `static_ride_height_rear_m` | m | Rear ride height standing still on level ground |

`[suspension.front_left]`, `[suspension.front_right]`, `[suspension.rear_left]`,
`[suspension.rear_right]`: one corner each, all values measured at the wheel.

| Key | Unit | Meaning |
|---|---|---|
| `spring_rate_N_per_m` | N/m | Wheel rate (motion ratio included), > 0 |
| `damping_N_s_per_m` | N·s/m | Damping coefficient, >= 0 |
| `bump_gap_m` | m | Compression from static before the bump stop touches, >= 0 |
| `bump_rate_N_per_m` | N/m | Bump stop stiffness, >= 0 |
| `droop_gap_m` | m | Extension from static before the droop limiter engages, >= 0 |
| `droop_rate_N_per_m` | N/m | Droop stop stiffness, >= 0 |

## Tyre file (`tyres/*.toml`) (docs/03, docs/14)

| Key | Unit | Meaning |
|---|---|---|
| `format_version` | | Must be `1` |
| `mu_x_ref` | | Longitudinal peak friction coefficient at `reference_load_N`, limits traction and braking, > 0 |
| `mu_y_ref` | | Lateral (cornering) peak friction coefficient at `reference_load_N`, > 0. Equal to `mu_x_ref` = friction circle |
| `reference_load_N` | N | Tyre load where `mu_x_ref` and `mu_y_ref` apply, > 0 |
| `load_sensitivity` | | Fraction by which mu drops when the tyre load rises by `reference_load_N`, >= 0. 0 = constant mu |
| `mu_rolling` | | Rolling resistance coefficient while moving |
| `mu_breakaway` | | Resistance coefficient to start moving from standstill |
| `wheel_radius_m` | m | Wheel radius, > 0 |

## Environment file (`environments/*.toml`)

| Key | Unit | Meaning |
|---|---|---|
| `format_version` | | Must be `1` |
| `air_density_kg_per_m3` | kg/m³ | Air density |
| `g_acceleration_m_per_s2` | m/s² | Gravitational acceleration |

## Track file (`tracks/*.csv`)

A header row `start_m,end_m`, then one braking zone per row. Lines starting with `#` are
comments. A bad row is an error that names the file and the line number.

| Column | Unit | Meaning |
|---|---|---|
| `start_m` | m | Distance where the braking zone starts |
| `end_m` | m | Distance where the braking zone ends, >= `start_m` |

The car brakes at full threshold braking inside a zone and accelerates everywhere else.
Zones should be in driving order. With no rows, the car never brakes.
