//! Loads a simulation config from data files: a run file that points to a car, tyre,
//! environment and track file (data/README.md).
//!
//! Paths inside a file are relative to that file's own folder, so a run works from any
//! working directory.

use crate::physics::aero::Aero;
use crate::physics::braking::Brakes;
use crate::physics::car::{Car, Mass};
use crate::physics::chassis::{ChassisGeometry, Frame};
use crate::physics::driver::Driver;
use crate::physics::environment::Environment;
use crate::physics::powertrain::Powertrain;
use crate::physics::simulation::{EndCondition, SimulationConfig};
use crate::physics::states::CarState;
use crate::physics::suspension::{StaticCarValues, Suspension};
use crate::physics::track::Track;
use crate::physics::tyres::Tyre;
use serde::Deserialize;
use serde::de::DeserializeOwned;
use std::fs;
use std::path::{Path, PathBuf};

/// The only data file format version this build can read
const FORMAT_VERSION: i64 = 1;

/// Header row of a track CSV with braking zones
const TRACK_HEADER: [&str; 2] = ["start_m", "end_m"];

/// Header row of a track CSV with the XY points of the line (docs/15 section 15.2)
const TRACK_POINTS_HEADER: [&str; 2] = ["x_m", "y_m"];

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
    /// When the run ends: `{ time_s }` or `{ laps, max_time_s }` (docs/15 section 15.8)
    end_condition: EndCondition,
    /// Length the track curvature is averaged over in meters, m, 0 = no smoothing (docs/15
    /// section 15.5). Only used by a track with XY points. If missing, 0 is used.
    #[serde(rename = "curvature_smoothing_window_m")]
    curvature_smoothing_window: Option<f64>,
    /// Time step of the simulation in seconds, s
    #[serde(rename = "time_step_s")]
    time_step: f64,
    /// How the driver plans the lap (docs/17). If missing, the driver has no grip margin.
    driver: Option<Driver>,
    /// Starting conditions of the car at the start time. The calculated fields of `CarState`
    /// are not in the file, `SimulationConfig::prepare` fills them.
    initial_state: CarState,
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
    /// Springs, dampers, end stops, anti-roll bars, roll centres and body inertias
    suspension: Suspension,
    /// Frame torsional stiffness (docs/19 section 19.8). Missing = rigid frame.
    frame: Option<Frame>,
}

/// Just the version key of a TOML data file. The other keys are ignored here, they are
/// checked when the whole file is read.
#[derive(Deserialize)]
struct FormatVersion {
    /// Data file format version, must be `FORMAT_VERSION`
    format_version: i64,
}

/// Loads a run file and every file it points to, then runs `SimulationConfig::prepare`:
/// it validates the config and calculates everything that follows from it (the shift tables
/// and the calculated part of the starting car state). The returned config is always valid
/// and complete.
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
    let track = load_track(
        &run_folder.join(&run.track),
        run.curvature_smoothing_window.unwrap_or(0.0),
    )?;

    let mut simulation_config = SimulationConfig {
        initial_car_state: run.initial_state,
        car: Car {
            mass: car.mass,
            powertrain: car.powertrain,
            tyre,
            aero: car.aero,
            geometry: car.geometry,
            brakes: car.brakes,
            suspension: car.suspension,
            frame: car.frame,
        },
        environment,
        track,
        driver: run.driver.unwrap_or_default(),
        start_time: run.start_time,
        end_condition: run.end_condition,
        time_step: run.time_step,
        // Filled by prepare below
        static_values: StaticCarValues::default(),
        corner_speed_limits: Vec::new(),
        target_speeds: Vec::new(),
        braking_points: Vec::new(),
    };

    simulation_config
        .prepare()
        .map_err(|message| format!("{}: invalid config: {}", path.display(), message))?;

    Ok(simulation_config)
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

/// Reads a whole data file as text
fn read_file(path: &Path) -> Result<String, String> {
    fs::read_to_string(path)
        .map_err(|error| format!("{}: cannot read file: {}", path.display(), error))
}

/// Folder a data file is in, which its relative paths start from
fn folder_of(path: &Path) -> &Path {
    path.parent().unwrap_or(Path::new(""))
}

/// Reads a track CSV. The header row decides what the rows are:
/// * `start_m,end_m`: one braking zone per row in meters, m,
/// * `x_m,y_m`: one point of the line the car drives per row in meters, m (docs/15). The
///   curvature is smoothed over `curvature_smoothing_window` meters.
///
/// Lines starting with `#` are comments.
fn load_track(path: &Path, curvature_smoothing_window: f64) -> Result<Track, String> {
    let text = read_file(path)?;
    let mut reader = csv::ReaderBuilder::new()
        .comment(Some(b'#'))
        .trim(csv::Trim::All)
        .from_reader(text.as_bytes());

    let header = reader
        .headers()
        .map_err(|error| format!("{}: {}", path.display(), error))?;
    let columns = if header.iter().eq(TRACK_HEADER) {
        TRACK_HEADER
    } else if header.iter().eq(TRACK_POINTS_HEADER) {
        TRACK_POINTS_HEADER
    } else {
        return Err(format!(
            "{}: line {}: header must be `{}` or `{}`, got `{}`",
            path.display(),
            header.position().map_or(1, |position| position.line()),
            TRACK_HEADER.join(","),
            TRACK_POINTS_HEADER.join(","),
            header.iter().collect::<Vec<_>>().join(",")
        ));
    };

    let mut rows = Vec::new();
    for record in reader.records() {
        let record = record.map_err(|error| csv_error_message(path, &error))?;
        let line = record.position().map_or(0, |position| position.line());

        let first = parse_csv_number(path, line, columns[0], &record[0])?;
        let second = parse_csv_number(path, line, columns[1], &record[1])?;
        rows.push((first, second));
    }

    if columns == TRACK_POINTS_HEADER {
        Track::from_points(rows, curvature_smoothing_window)
            .map_err(|message| format!("{}: {}", path.display(), message))
    } else {
        Ok(Track {
            braking_zones: rows,
            points: Vec::new(),
            distances: Vec::new(),
            curvatures: Vec::new(),
            lap_length: 0.0,
        })
    }
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

    /// The config main.rs built before data files existed, prepared like `load_run` does
    /// (start_simulation always prepared it before the first step)
    fn expected_baseline() -> SimulationConfig {
        let mut simulation_config = SimulationConfig {
            end_condition: EndCondition::Time { end_time: 100000.0 },
            ..SimulationConfig::default()
        };
        simulation_config.prepare().unwrap();
        simulation_config
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

        // The starting conditions in the run file are exactly CarState::default()
        let default_car_state = CarState::default();
        assert_eq!(loaded.initial_car_state.speed, default_car_state.speed);
        assert_eq!(
            loaded.initial_car_state.distance,
            default_car_state.distance
        );
        assert_eq!(loaded.initial_car_state.gear, default_car_state.gear);
        assert_eq!(
            loaded.initial_car_state.shift_time_left,
            default_car_state.shift_time_left
        );
        assert_eq!(
            loaded.initial_car_state.suspension,
            default_car_state.suspension
        );

        // Every field, exact float equality: the files must hold the exact Default values
        assert_eq!(loaded.initial_car_state, expected.initial_car_state);
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
        assert_eq!(loaded.end_condition, expected.end_condition);
        assert_eq!(loaded.time_step, expected.time_step);
        // And the whole struct, so a field added later can't be missed
        assert_eq!(loaded, expected);
    }

    #[test]
    fn test_baseline_gives_same_results() {
        let mut loaded = load_run(&baseline_run()).unwrap();
        let mut expected = expected_baseline();
        // Long enough to pass the braking zone at 900 m to 1000 m and shift up several times
        loaded.end_condition = EndCondition::Time { end_time: 30.0 };
        expected.end_condition = EndCondition::Time { end_time: 30.0 };

        let loaded_state = start_simulation(&mut loaded, None, false);
        let expected_state = start_simulation(&mut expected, None, false);

        assert_eq!(loaded_state, expected_state);
        // The shift tables computed from the loaded data are the same too
        assert_eq!(loaded, expected);
    }

    #[test]
    fn test_tyre_override() {
        let data = TempData::new("tyre_override");
        data.write(
            "tyres/grippy.toml",
            "format_version = 1\nmu_x_ref = 1.2\nmu_y_ref = 1.1\nreference_load_N = 700.0\nload_sensitivity = 0.2\nmu_rolling = 0.03\nmu_breakaway = 0.035\nwheel_radius_m = 0.33\n",
        );

        // The run file's tyres win over the car's default_tyres
        data.replace(
            "runs/baseline.toml",
            "tyres = \"../tyres/baseline.toml\"",
            "tyres = \"../tyres/grippy.toml\"",
        );
        let tyre = load_run(&data.run()).unwrap().car.tyre;
        assert_eq!(tyre.mu_x_ref, 1.2);
        assert_eq!(tyre.mu_y_ref, 1.1);
        assert_eq!(tyre.reference_load, 700.0);
        assert_eq!(tyre.load_sensitivity, 0.2);
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
        let run = "runs/baseline.toml";
        data.replace(run, "speed_m_per_s = 10.0", "speed_m_per_s = 30.0");
        // Inside the 900 m to 1000 m braking zone
        data.replace(run, "distance_m = 100.0", "distance_m = 950.0");
        data.replace(run, "gear = 1", "gear = 2");
        data.replace(run, "shift_time_left_s = 0.0", "shift_time_left_s = 0.003");
        data.replace(run, "pitch_rad = 0.0", "pitch_rad = 0.001");

        let simulation_config = load_run(&data.run()).unwrap();
        let initial_car_state = simulation_config.initial_car_state;

        // Starting conditions come from the file
        assert_eq!(initial_car_state.speed, 30.0);
        assert_eq!(initial_car_state.distance, 950.0);
        assert_eq!(initial_car_state.gear, 2);
        assert_eq!(initial_car_state.shift_time_left, 0.003);
        assert_eq!(initial_car_state.suspension.pitch, 0.001);
        assert_eq!(initial_car_state.suspension.heave, 0.0);

        // The rest is calculated from them, not left at the defaults
        assert_eq!(initial_car_state.active_braking_zone, 0);
        assert!(initial_car_state.braking);
        // n = v * i_2 * i_f / (2 * pi * r) = 30 * 4.02 * 3.0 / (2 * pi * 0.36) = 159.95 Hz
        assert!(
            (initial_car_state.revs_hz - 159.9507).abs() < 1e-3,
            "{}",
            initial_car_state.revs_hz
        );
        // Braking, so the car decelerates
        assert!(
            initial_car_state.acceleration < 0.0,
            "{}",
            initial_car_state.acceleration
        );
    }

    #[test]
    fn test_calculated_initial_state_not_in_file() {
        // Calculated fields are not starting conditions, so writing one is an error
        let data = TempData::new("calculated_in_file");
        data.replace("runs/baseline.toml", "gear = 1", "gear = 1\nrevs_hz = 72.0");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("unknown field `revs_hz`"), "{}", message);
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
        data.replace("tyres/baseline.toml", "mu_x_ref", "mu_peak");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("unknown field `mu_peak`"), "{}", message);
        assert!(message.contains("../tyres/baseline.toml"), "{}", message);

        let data = TempData::new("unknown_key_run");
        data.replace("runs/baseline.toml", "time_step_s", "time_step");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("unknown field `time_step`"), "{}", message);
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
    fn test_frame_and_optional_roll_keys() {
        // docs/19 section 19.12: the baseline car has a rigid frame (no [frame] table)
        let baseline = load_run(&baseline_run()).unwrap();
        assert_eq!(baseline.car.frame, None);

        let data = TempData::new("frame_table");
        // Line by line, so the test works with either line ending
        data.replace("cars/baseline.toml", "# [frame]", "[frame]");
        data.replace(
            "cars/baseline.toml",
            "# torsional_stiffness_N_m_per_rad",
            "torsional_stiffness_N_m_per_rad",
        );
        let config = load_run(&data.run()).unwrap();
        assert_eq!(
            config.car.frame,
            Some(Frame {
                torsional_stiffness: 100_000.0
            })
        );

        // Roll centres, bars and the starting roll are optional, 0 when missing
        let data = TempData::new("optional_roll_keys");
        data.replace("cars/baseline.toml", "front_roll_centre_height_m = 0.0", "");
        data.replace(
            "cars/baseline.toml",
            "front_anti_roll_bar_stiffness_N_m_per_rad = 0.0",
            "",
        );
        data.replace("runs/baseline.toml", "roll_front_rad = 0.0", "");
        data.replace("runs/baseline.toml", "roll_rear_rad = 0.0", "");
        let config = load_run(&data.run()).unwrap();
        assert_eq!(config.car.suspension.front_roll_centre_height, 0.0);
        assert_eq!(config.car.suspension.front_anti_roll_bar_stiffness, 0.0);
        assert_eq!(config.initial_car_state.suspension.roll_front, 0.0);

        // The track widths are required
        let data = TempData::new("missing_track_width");
        data.replace("cars/baseline.toml", "front_track_width_m = 1.6", "");
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("missing field `front_track_width_m`"),
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

    #[test]
    fn test_oval_run_loads() {
        // data/runs/oval_3_laps.toml: XY track, run ends on laps
        let run = data_folder().join("runs").join("oval_3_laps.toml");
        let loaded = load_run(&run).unwrap();

        assert_eq!(
            loaded.end_condition,
            EndCondition::Laps {
                laps: 3,
                max_time: 600.0
            }
        );
        assert!(loaded.track.braking_zones.is_empty());
        assert_eq!(loaded.track.points.len(), 244);
        // docs/15 section 15.9: 2 * 75 + 2 * pi * 15 = 244.248 m, a little less with straight segments
        assert!((loaded.track.lap_length - 244.248).abs() < 0.02);
        assert_eq!(loaded.track.curvatures[37], 0.0);
        assert!((loaded.track.curvatures[98] * 15.0 - 1.0).abs() < 1e-3);
        assert_eq!(loaded.initial_car_state.distance, 0.0);
    }

    #[test]
    fn test_end_condition_parses() {
        let time = "end_condition = { time_s = 100000.0 }";

        // Laps need an XY track
        let data = TempData::new("end_condition_laps_no_line");
        data.replace(
            "runs/baseline.toml",
            time,
            "end_condition = { laps = 2, max_time_s = 60.0 }",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("needs a track with XY points"),
            "{}",
            message
        );

        // Neither one nor the other
        for (name, bad) in [
            (
                "both",
                "end_condition = { time_s = 30.0, laps = 2, max_time_s = 60.0 }",
            ),
            ("laps_only", "end_condition = { laps = 2 }"),
            ("empty", "end_condition = { }"),
        ] {
            let data = TempData::new(&format!("end_condition_{}", name));
            data.replace("runs/baseline.toml", time, bad);
            let message = load_run(&data.run()).unwrap_err();
            assert!(
                message.contains(
                    "must be either { time_s = ... } or { laps = ..., max_time_s = ... }"
                ),
                "{}",
                message
            );
        }

        // Unknown key, and the old key
        let data = TempData::new("end_condition_unknown");
        data.replace(
            "runs/baseline.toml",
            time,
            "end_condition = { time = 30.0 }",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("unknown field `time`"), "{}", message);

        let data = TempData::new("end_condition_old_key");
        data.replace("runs/baseline.toml", time, "end_time_s = 30.0");
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("unknown field `end_time_s`"),
            "{}",
            message
        );
    }

    #[test]
    fn test_points_track_loads_from_run() {
        let track = "tracks/straight_1000m.csv";
        let square = "x_m,y_m\n0.0,0.0\n10.0,0.0\n10.0,10.0\n0.0,10.0\n";

        let data = TempData::new("points_track");
        data.write(track, square);
        let loaded = load_run(&data.run()).unwrap();
        assert_eq!(loaded.track.lap_length, 40.0);
        assert!(loaded.track.braking_zones.is_empty());
        // Started 100 m in: on lap 3 (k = 2), 20 m after the line, at point 2
        assert_eq!(loaded.initial_car_state.lap, 2);
        assert_eq!(
            (loaded.initial_car_state.x, loaded.initial_car_state.y),
            (10.0, 10.0)
        );

        // Smoothing window from the run file: 25 m covers 3 of the 4 corners
        let data = TempData::new("points_track_window");
        data.write(track, square);
        data.replace(
            "runs/baseline.toml",
            "time_step_s = 0.001",
            "time_step_s = 0.001\ncurvature_smoothing_window_m = -1.0",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("straight_1000m.csv"), "{}", message);
        assert!(
            message.contains("curvature_smoothing_window must be >= 0 m"),
            "{}",
            message
        );

        // Too few points names the file
        let data = TempData::new("points_track_two_points");
        data.write(track, "x_m,y_m\n0.0,0.0\n10.0,0.0\n");
        let message = load_run(&data.run()).unwrap_err();
        assert!(message.contains("straight_1000m.csv"), "{}", message);
        assert!(message.contains("at least 3 points"), "{}", message);
    }

    #[test]
    fn test_skidpad_run_loads() {
        // data/runs/skidpad_5_laps.toml (docs/16 section 16.9)
        let run = data_folder().join("runs").join("skidpad_5_laps.toml");
        let loaded = load_run(&run).unwrap();

        assert_eq!(loaded.track.points.len(), 57);
        assert!((loaded.track.lap_length - 2.0 * std::f64::consts::PI * 9.125).abs() < 0.05);
        for curvature in &loaded.track.curvatures {
            assert!((curvature * 9.125 - 1.0).abs() < 1e-3, "{}", curvature);
        }
        // The corner speed limits are filled by prepare: 9.02 m/s everywhere
        assert_eq!(loaded.corner_speed_limits.len(), 57);
        for limit in &loaded.corner_speed_limits {
            assert!((limit - 9.02).abs() < 0.005, "{}", limit);
        }
        assert!((loaded.initial_car_state.corner_speed_limit - 9.02).abs() < 0.005);
    }

    #[test]
    fn test_driver_table_parses() {
        // No [driver] table in the baseline run: no grip margin
        let loaded = load_run(&baseline_run()).unwrap();
        assert_eq!(loaded.driver, Driver::default());
        assert_eq!(loaded.driver.grip_margin, 0.0);

        // The oval run plans with a 5 % margin (docs/17 section 17.2)
        let oval = load_run(&data_folder().join("runs").join("oval_3_laps.toml")).unwrap();
        assert_eq!(oval.driver.grip_margin, 0.05);
        assert_eq!(oval.target_speeds.len(), 244);
        assert!((oval.corner_speed_limits[98] - 11.31).abs() < 0.005);

        // Out of range and unknown keys are errors
        let table = "[initial_state]";
        let data = TempData::new("driver_margin_range");
        data.replace(
            "runs/baseline.toml",
            table,
            "[driver]\ngrip_margin = 1.0\n\n[initial_state]",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("driver.grip_margin must be in 0.0..1.0"),
            "{}",
            message
        );

        let data = TempData::new("driver_unknown_key");
        data.replace(
            "runs/baseline.toml",
            table,
            "[driver]\ngrip_margin = 0.05\nlook_ahead_m = 50.0\n\n[initial_state]",
        );
        let message = load_run(&data.run()).unwrap_err();
        assert!(
            message.contains("unknown field `look_ahead_m`"),
            "{}",
            message
        );

        // docs/18: braking_point_trials alone, without grip_margin (which is then 0)
        let data = TempData::new("driver_trials");
        data.replace(
            "runs/baseline.toml",
            table,
            "[driver]\nbraking_point_trials = true\n\n[initial_state]",
        );
        let loaded = load_run(&data.run()).unwrap();
        assert!(loaded.driver.braking_point_trials);
        assert_eq!(loaded.driver.grip_margin, 0.0);
        // A track with braking zones has no corners to correct
        assert!(loaded.braking_points.is_empty());
    }
}
