//! Loads a simulation config from data files: a run file that points to a car, tyre,
//! environment and track file (data/README.md).
//!
//! Paths inside a file are relative to that file's own folder, so a run works from any
//! working directory.

use crate::physics::aero::Aero;
use crate::physics::braking::Brakes;
use crate::physics::car::{Car, ChassisGeometry, Mass};
use crate::physics::chassis::Suspension;
use crate::physics::environment::Environment;
use crate::physics::powertrain::Powertrain;
use crate::physics::simulation::SimulationConfig;
use crate::physics::states::CarState;
use crate::physics::track::Track;
use crate::physics::tyres::Tyre;
use serde::Deserialize;
use serde::de::DeserializeOwned;
use std::fs;
use std::path::{Path, PathBuf};

/// The only data file format version this build can read
const FORMAT_VERSION: i64 = 1;

/// Header row a track CSV must start with
const TRACK_HEADER: [&str; 2] = ["start_m", "end_m"];

/// Just the version key of a TOML data file. The other keys are ignored here, they are
/// checked when the whole file is read.
#[derive(Deserialize)]
struct FormatVersion {
    /// Data file format version, must be `FORMAT_VERSION`
    format_version: i64,
}

/// Run file: which car, tyres, track and environment to simulate, and the time settings
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RunFile {
    /// Already checked by `check_format_version` before the file is read
    #[serde(rename = "format_version")]
    _format_version: i64,
    /// Car file, relative to the run file's folder
    car: PathBuf,
    /// Tyre file, relative to the run file's folder. If missing, the car's `default_tyres` is used.
    tyres: Option<PathBuf>,
    /// Track CSV file, relative to the run file's folder
    track: PathBuf,
    /// Environment file, relative to the run file's folder
    environment: PathBuf,
    /// Start time of the simulation in seconds, s
    #[serde(rename = "start_time_s")]
    start_time: f64,
    /// End time of the simulation in seconds, s
    #[serde(rename = "end_time_s")]
    end_time: f64,
    /// Time step of the simulation in seconds, s
    #[serde(rename = "time_step_s")]
    time_step: f64,
    /// State of the car at the start time
    initial_state: InitialState,
}

/// Starting values of the car state that a run file sets. The rest of `CarState` is output
/// or recalculated every step, so it starts from `CarState::default()`.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct InitialState {
    /// Speed in meters per second, m/s
    #[serde(rename = "speed_m_per_s")]
    speed: f64,
    /// Distance along the track in meters, m
    #[serde(rename = "distance_m")]
    distance: f64,
    /// Gear, 1 is the first gear
    gear: u8,
}

/// Car file: every `Car` section except the tyres, which come from their own file
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CarFile {
    /// Already checked by `check_format_version` before the file is read
    #[serde(rename = "format_version")]
    _format_version: i64,
    /// Tyre file used when the run file has no `tyres`, relative to the car file's folder
    default_tyres: PathBuf,
    /// Mass distribution
    mass: Mass,
    /// Chassis dimensions
    geometry: ChassisGeometry,
    /// Aerodynamic model
    aero: Aero,
    /// Brake system
    brakes: Brakes,
    /// Engine, gearbox and drivetrain
    powertrain: Powertrain,
    /// Springs, dampers, end stops and pitch inertia
    suspension: Suspension,
}

/// Loads a run file and every file it points to, and checks the result with
/// `SimulationConfig::validate`, so the returned config is always valid.
///
/// Returns a message naming the file (and the key or CSV line when there is one) on error.
pub fn load_run(path: &Path) -> Result<SimulationConfig, String> {
    let run: RunFile = load_toml(path)?;
    let run_folder = folder_of(path);

    let car_path = run_folder.join(&run.car);
    let car: CarFile = load_toml(&car_path)?;

    let tyre_path = match &run.tyres {
        Some(tyres) => run_folder.join(tyres),
        None => folder_of(&car_path).join(&car.default_tyres),
    };
    let tyre: Tyre = load_flat_toml(&tyre_path)?;

    let environment: Environment = load_flat_toml(&run_folder.join(&run.environment))?;
    let track = load_track(&run_folder.join(&run.track))?;

    let simulation_config = SimulationConfig {
        initial_car: CarState {
            speed: run.initial_state.speed,
            distance: run.initial_state.distance,
            gear: run.initial_state.gear,
            ..CarState::default()
        },
        car: Car {
            mass: car.mass,
            powertrain: car.powertrain,
            tyre,
            aero: car.aero,
            geometry: car.geometry,
            brakes: car.brakes,
            suspension: car.suspension,
        },
        environment,
        track,
        start_time: run.start_time,
        end_time: run.end_time,
        time_step: run.time_step,
    };

    simulation_config
        .validate()
        .map_err(|message| format!("{}: invalid config: {}", path.display(), message))?;

    Ok(simulation_config)
}

/// Folder a data file is in, which its relative paths start from
fn folder_of(path: &Path) -> &Path {
    path.parent().unwrap_or(Path::new(""))
}

/// Reads a whole data file as text
fn read_file(path: &Path) -> Result<String, String> {
    fs::read_to_string(path)
        .map_err(|error| format!("{}: cannot read file: {}", path.display(), error))
}

/// Checks that a TOML data file has `format_version = FORMAT_VERSION`
fn check_format_version(path: &Path, text: &str) -> Result<(), String> {
    let version: FormatVersion =
        toml::from_str(text).map_err(|error| format!("{}: {}", path.display(), error))?;

    if version.format_version != FORMAT_VERSION {
        return Err(format!(
            "{}: format_version must be {}, got {}",
            path.display(),
            FORMAT_VERSION,
            version.format_version
        ));
    }

    Ok(())
}

/// Reads a TOML data file: checks the format version first, then reads every key
fn load_toml<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text = read_file(path)?;
    check_format_version(path, &text)?;

    toml::from_str(&text).map_err(|error| format!("{}: {}", path.display(), error))
}

/// Reads a TOML data file without sections (tyres, environment) straight into `T`, whose
/// struct has no `format_version`: checks the version, then reads every other key.
///
/// The keys go through a table with `format_version` removed. Wrapping `T` with
/// `#[serde(flatten)]` instead would report a typo as the missing correct key, not as the typo.
fn load_flat_toml<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text = read_file(path)?;
    check_format_version(path, &text)?;

    let mut table: toml::Table =
        toml::from_str(&text).map_err(|error| format!("{}: {}", path.display(), error))?;
    table.remove("format_version");

    T::deserialize(table).map_err(|error| format!("{}: {}", path.display(), error))
}

/// Reads a track CSV: a `start_m,end_m` header, then one braking zone per row in meters, m.
/// Lines starting with `#` are comments.
fn load_track(path: &Path) -> Result<Track, String> {
    let text = read_file(path)?;
    let mut reader = csv::ReaderBuilder::new()
        .comment(Some(b'#'))
        .trim(csv::Trim::All)
        .from_reader(text.as_bytes());

    let header = reader
        .headers()
        .map_err(|error| format!("{}: {}", path.display(), error))?;
    if header.iter().ne(TRACK_HEADER) {
        return Err(format!(
            "{}: line {}: header must be `{}`, got `{}`",
            path.display(),
            header.position().map_or(1, |position| position.line()),
            TRACK_HEADER.join(","),
            header.iter().collect::<Vec<_>>().join(",")
        ));
    }

    let mut braking_zones = Vec::new();
    for record in reader.records() {
        let record = record.map_err(|error| csv_error_message(path, &error))?;
        let line = record.position().map_or(0, |position| position.line());

        let start = parse_csv_number(path, line, TRACK_HEADER[0], &record[0])?;
        let end = parse_csv_number(path, line, TRACK_HEADER[1], &record[1])?;
        braking_zones.push((start, end));
    }

    Ok(Track { braking_zones })
}

/// Describes a csv error in the same `path: line N: ...` form as the other track errors
fn csv_error_message(path: &Path, error: &csv::Error) -> String {
    match error.kind() {
        csv::ErrorKind::UnequalLengths {
            pos: Some(position),
            expected_len,
            len,
        } => format!(
            "{}: line {}: expected {} columns, got {}",
            path.display(),
            position.line(),
            expected_len,
            len
        ),
        _ => format!("{}: {}", path.display(), error),
    }
}

/// Parses one CSV cell as a number, naming the file, line and column on error
fn parse_csv_number(path: &Path, line: u64, column: &str, cell: &str) -> Result<f64, String> {
    cell.parse().map_err(|_| {
        format!(
            "{}: line {}: {} must be a number, got `{}`",
            path.display(),
            line,
            column,
            cell
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::powertrain::Drivetrain;
    use crate::physics::simulation::start_simulation;

    /// The repo's data folder, found from the manifest so tests don't depend on the working directory
    fn data_folder() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("data")
    }

    /// The baseline run file in the repo
    fn baseline_run() -> PathBuf {
        data_folder().join("runs").join("baseline.toml")
    }

    /// The config main.rs built before data files existed
    fn expected_baseline() -> SimulationConfig {
        SimulationConfig {
            end_time: 100000.0,
            ..SimulationConfig::default()
        }
    }

    /// A folder in the system temp folder holding a copy of the baseline data files, laid out
    /// like data/. Removed when dropped.
    struct TempData {
        /// Root of the copy
        root: PathBuf,
    }

    impl TempData {
        /// Copies the baseline data files into a new temp folder. `name` must be unique per test,
        /// because tests run in parallel.
        fn new(name: &str) -> Self {
            let root = std::env::temp_dir().join(format!(
                "f1_digital_twin_{}_{}",
                std::process::id(),
                name
            ));
            let _ = fs::remove_dir_all(&root);

            for file in [
                "runs/baseline.toml",
                "cars/baseline.toml",
                "tyres/baseline.toml",
                "environments/standard.toml",
                "tracks/straight_1000m.csv",
            ] {
                let to = root.join(file);
                fs::create_dir_all(to.parent().unwrap()).unwrap();
                fs::copy(data_folder().join(file), &to).unwrap();
            }

            Self { root }
        }

        /// Path of a file inside the copy, e.g. "cars/baseline.toml"
        fn path(&self, file: &str) -> PathBuf {
            self.root.join(file)
        }

        /// The copied run file
        fn run(&self) -> PathBuf {
            self.path("runs/baseline.toml")
        }

        /// Replaces the first `from` in a copied file with `to`. Panics if `from` isn't there,
        /// so a test can't silently pass on an unchanged file.
        fn replace(&self, file: &str, from: &str, to: &str) {
            let path = self.path(file);
            let text = fs::read_to_string(&path).unwrap();
            assert!(text.contains(from), "`{}` not found in {}", from, file);
            fs::write(&path, text.replacen(from, to, 1)).unwrap();
        }

        /// Overwrites a copied file (or writes a new one)
        fn write(&self, file: &str, text: &str) {
            fs::write(self.path(file), text).unwrap();
        }
    }

    impl Drop for TempData {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.root);
        }
    }

    #[test]
    fn test_baseline_equals_defaults() {
        let loaded = load_run(&baseline_run()).unwrap();
        let expected = expected_baseline();

        // Every field, exact float equality: the files must hold the exact Default values
        assert_eq!(loaded.initial_car, expected.initial_car);
        assert_eq!(loaded.car.mass, expected.car.mass);
        assert_eq!(loaded.car.geometry, expected.car.geometry);
        assert_eq!(loaded.car.aero, expected.car.aero);
        assert_eq!(loaded.car.brakes, expected.car.brakes);
        assert_eq!(loaded.car.powertrain, expected.car.powertrain);
        assert_eq!(loaded.car.suspension, expected.car.suspension);
        assert_eq!(loaded.car.tyre, expected.car.tyre);
        assert_eq!(loaded.environment, expected.environment);
        assert_eq!(loaded.track, expected.track);
        assert_eq!(loaded.start_time, expected.start_time);
        assert_eq!(loaded.end_time, expected.end_time);
        assert_eq!(loaded.time_step, expected.time_step);
        // And the whole struct, so a field added later can't be missed
        assert_eq!(loaded, expected);
    }

    #[test]
    fn test_baseline_gives_same_results() {
        let mut loaded = load_run(&baseline_run()).unwrap();
        let mut expected = expected_baseline();
        // Long enough to pass the braking zone at 900 m to 1000 m and shift up several times
        loaded.end_time = 30.0;
        expected.end_time = 30.0;

        let loaded_state = start_simulation(&mut loaded, false, false);
        let expected_state = start_simulation(&mut expected, false, false);

        assert_eq!(loaded_state, expected_state);
        // The shift tables computed from the loaded data are the same too
        assert_eq!(loaded, expected);
    }

    #[test]
    fn test_tyre_override() {
        let data = TempData::new("tyre_override");
        data.write(
            "tyres/grippy.toml",
            "format_version = 1\nmu_static = 1.2\nmu_rolling = 0.03\nmu_breakaway = 0.035\nwheel_radius_m = 0.33\n",
        );

        // The run file's tyres win over the car's default_tyres
        data.replace(
            "runs/baseline.toml",
            "tyres = \"../tyres/baseline.toml\"",
            "tyres = \"../tyres/grippy.toml\"",
        );
        let tyre = load_run(&data.run()).unwrap().car.tyre;
        assert_eq!(tyre.mu_static, 1.2);
        assert_eq!(tyre.wheel_radius, 0.33);

        // Without tyres in the run file, the car's default_tyres is used
        data.replace("runs/baseline.toml", "tyres = \"../tyres/grippy.toml\"", "");
        assert_eq!(load_run(&data.run()).unwrap().car.tyre, Tyre::default());
    }

    #[test]
    fn test_paths_relative_to_file() {
        // A different layout from data/, loaded with the repo root as the working directory
        // (cargo test), where none of these relative paths exist
        let data = TempData::new("relative_paths");
        let nested_run = data.path("runs/nested/deeper/run.toml");
        fs::create_dir_all(nested_run.parent().unwrap()).unwrap();
        let run_text = fs::read_to_string(data.run())
            .unwrap()
            .replace("\"../", "\"../../../");
        fs::write(&nested_run, run_text).unwrap();
        // The car's default_tyres stays relative to the car file
        data.replace(
            "runs/baseline.toml",
            "tyres = \"../tyres/baseline.toml\"",
            "",
        );

        assert_eq!(load_run(&nested_run).unwrap(), expected_baseline());
        assert_eq!(load_run(&data.run()).unwrap(), expected_baseline());
    }

    #[test]
    fn test_initial_state_from_run_file() {
        let data = TempData::new("initial_state");
        data.replace(
            "runs/baseline.toml",
            "speed_m_per_s = 10.0",
            "speed_m_per_s = 0.0",
        );
        data.replace(
            "runs/baseline.toml",
            "distance_m = 100.0",
            "distance_m = 0.0",
        );
        data.replace("runs/baseline.toml", "gear = 1", "gear = 2");

        // The rest of the state starts from the default
        assert_eq!(
            load_run(&data.run()).unwrap().initial_car,
            CarState {
                speed: 0.0,
                distance: 0.0,
                gear: 2,
                ..CarState::default()
            }
        );
    }

    #[test]
    fn test_unknown_key_rejected() {
        // Typo in the unit
        let data = TempData::new("unknown_key_typo");
        data.replace(
            "cars/baseline.toml",
            "spring_rate_N_per_m",
            "spring_rate_N_per_mm",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("unknown field `spring_rate_N_per_mm`"),
            "{}",
            message
        );
        assert!(message.contains("../cars/baseline.toml"), "{}", message);

        // Wrong capitalisation of a unit
        let data = TempData::new("unknown_key_case");
        data.replace(
            "cars/baseline.toml",
            "spring_rate_N_per_m",
            "spring_rate_n_per_m",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("unknown field `spring_rate_n_per_m`"),
            "{}",
            message
        );

        // Unknown key in a flat file and in the run file
        let data = TempData::new("unknown_key_flat");
        data.replace("tyres/baseline.toml", "mu_static", "mu_peak");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("unknown field `mu_peak`"), "{}", message);
        assert!(message.contains("../tyres/baseline.toml"), "{}", message);

        let data = TempData::new("unknown_key_run");
        data.replace("runs/baseline.toml", "end_time_s", "end_time");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("unknown field `end_time`"), "{}", message);
    }

    #[test]
    fn test_missing_key_rejected() {
        let data = TempData::new("missing_key_car");
        data.replace("cars/baseline.toml", "cog_height_m = 0.3", "");
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("missing field `cog_height_m`"),
            "{}",
            message
        );
        assert!(message.contains("../cars/baseline.toml"), "{}", message);

        let data = TempData::new("missing_key_environment");
        data.replace(
            "environments/standard.toml",
            "g_acceleration_m_per_s2 = 9.81",
            "",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("missing field `g_acceleration_m_per_s2`"),
            "{}",
            message
        );
        assert!(
            message.contains("../environments/standard.toml"),
            "{}",
            message
        );

        let data = TempData::new("missing_key_run");
        data.replace("runs/baseline.toml", "gear = 1", "");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("missing field `gear`"), "{}", message);
        assert!(
            message.contains(&data.run().display().to_string()),
            "{}",
            message
        );
    }

    #[test]
    fn test_wrong_format_version_rejected() {
        let data = TempData::new("format_version_wrong");
        data.replace(
            "cars/baseline.toml",
            "format_version = 1",
            "format_version = 2",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("../cars/baseline.toml"), "{}", message);
        assert!(
            message.contains("format_version must be 1, got 2"),
            "{}",
            message
        );

        let data = TempData::new("format_version_missing");
        data.replace("environments/standard.toml", "format_version = 1", "");
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("missing field `format_version`"),
            "{}",
            message
        );
    }

    #[test]
    fn test_missing_file_rejected() {
        let data = TempData::new("missing_file");
        data.replace(
            "runs/baseline.toml",
            "\"../cars/baseline.toml\"",
            "\"../cars/no_such_car.toml\"",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("../cars/no_such_car.toml"), "{}", message);
        assert!(message.contains("cannot read file"), "{}", message);

        let missing_run = data.path("runs/no_such_run.toml");
        let message = load_run(&missing_run).unwrap_err();
        assert!(
            message.contains(&missing_run.display().to_string()),
            "{}",
            message
        );
    }

    #[test]
    fn test_bad_csv_row_rejected() {
        let track = "tracks/straight_1000m.csv";

        // Not a number, on line 3
        let data = TempData::new("csv_not_a_number");
        data.write(track, "start_m,end_m\n900.0,1000.0\n1200.0,abc\n");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("straight_1000m.csv"), "{}", message);
        assert!(message.contains("line 3"), "{}", message);
        assert!(
            message.contains("end_m must be a number, got `abc`"),
            "{}",
            message
        );

        // Comment lines still count for the line number
        let data = TempData::new("csv_comment");
        data.write(
            track,
            "# Braking zones\nstart_m,end_m\n900.0,1000.0\nx,1300.0\n",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("line 4"), "{}", message);

        // Wrong number of columns
        let data = TempData::new("csv_columns");
        data.write(track, "start_m,end_m\n900.0,1000.0,5.0\n");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("straight_1000m.csv"), "{}", message);
        assert!(message.contains("line 2"), "{}", message);

        // Wrong header
        let data = TempData::new("csv_header");
        data.write(track, "start,end\n900.0,1000.0\n");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("line 1"), "{}", message);
        assert!(
            message.contains("header must be `start_m,end_m`"),
            "{}",
            message
        );

        // End before start is caught by validate, which names the zone
        let data = TempData::new("csv_end_before_start");
        data.write(track, "start_m,end_m\n900.0,1000.0\n1200.0,1100.0\n");
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("track.braking_zones[1] end must be >= start"),
            "{}",
            message
        );
    }

    #[test]
    fn test_drivetrain_parses_from_tagged_table() {
        let data = TempData::new("drivetrain_awd");
        data.replace(
            "cars/baseline.toml",
            "type = \"rear_wheel_drive\"",
            "type = \"all_wheel_drive\"\nfront_torque_split = 0.4",
        );
        assert_eq!(
            load_run(&data.run()).unwrap().car.powertrain.drivetrain,
            Drivetrain::AllWheelDrive {
                front_torque_split: 0.4
            }
        );

        let data = TempData::new("drivetrain_fwd");
        data.replace(
            "cars/baseline.toml",
            "type = \"rear_wheel_drive\"",
            "type = \"front_wheel_drive\"",
        );
        assert_eq!(
            load_run(&data.run()).unwrap().car.powertrain.drivetrain,
            Drivetrain::FrontWheelDrive
        );

        // AWD needs its split
        let data = TempData::new("drivetrain_awd_no_split");
        data.replace(
            "cars/baseline.toml",
            "type = \"rear_wheel_drive\"",
            "type = \"all_wheel_drive\"",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("missing field `front_torque_split`"),
            "{}",
            message
        );

        // A split on a two wheel drive car is a mistake, not a silently ignored key
        let data = TempData::new("drivetrain_rwd_split");
        data.replace(
            "cars/baseline.toml",
            "type = \"rear_wheel_drive\"",
            "type = \"rear_wheel_drive\"\nfront_torque_split = 0.4",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("unknown field `front_torque_split`"),
            "{}",
            message
        );
    }

    #[test]
    fn test_invalid_value_caught_by_validate() {
        let data = TempData::new("invalid_front_bias");
        data.replace(
            "cars/baseline.toml",
            "front_bias = 0.575",
            "front_bias = 1.5",
        );

        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.ends_with("brakes.front_bias must be in 0.0..=1.0, got 1.5"),
            "{}",
            message
        );
        assert!(
            message.contains(&data.run().display().to_string()),
            "{}",
            message
        );
    }
}
