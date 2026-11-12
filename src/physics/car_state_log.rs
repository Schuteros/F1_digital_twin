//! Writes the car state to a CSV file while the simulation runs (`--log` in main.rs)

use crate::physics::simulation::SimulationState;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

/// Column names of the log, in the order `write_row` writes them
const HEADER: [&str; 21] = [
    "time_s",
    "distance_m",
    "speed_m_per_s",
    "acceleration_m_per_s2",
    "gear",
    "revs_Hz",
    "braking",
    "shift_time_left_s",
    "active_braking_zone",
    "heave_m",
    "heave_velocity_m_per_s",
    "pitch_rad",
    "pitch_rate_rad_per_s",
    "lap",
    "x_m",
    "y_m",
    "curvature_1_per_m",
    "corner_speed_limit_m_per_s",
    "target_speed_m_per_s",
    "lateral_acceleration_m_per_s2",
    "over_grip_limit",
];

/// CSV log of the car state: one row at the start, then one every `every_steps` steps, and
/// always the last step of the run.
pub struct CarStateLog {
    /// File the log is written to
    path: PathBuf,
    /// Buffered writer, so each row doesn't go to the disk on its own
    writer: BufWriter<File>,
    /// A row is written every this many steps, at least 1
    every_steps: u64,
    /// First write error, reported by `finish`. Later rows are skipped after an error.
    error: Option<String>,
}

impl CarStateLog {
    /// Creates (or overwrites) the log file and writes the header row
    pub fn create(path: &Path, every_steps: u64) -> Result<Self, String> {
        if every_steps == 0 {
            return Err("log interval must be at least 1 step".to_string());
        }

        let file = File::create(path)
            .map_err(|error| format!("{}: cannot create log file: {}", path.display(), error))?;
        let mut log = Self {
            path: path.to_path_buf(),
            writer: BufWriter::new(file),
            every_steps,
            error: None,
        };
        log.write_line(&HEADER.join(","));

        Ok(log)
    }

    /// Writes the state after `step` steps if it is a logged step (`step` 0 is the start).
    /// `last` forces a row for the final step of the run.
    pub(crate) fn record(&mut self, step: u64, state: &SimulationState, last: bool) {
        if step.is_multiple_of(self.every_steps) || last {
            self.write_row(state);
        }
    }

    /// Flushes the file. Returns the first error that happened while writing.
    pub fn finish(mut self) -> Result<(), String> {
        if self.error.is_none()
            && let Err(error) = self.writer.flush()
        {
            self.error = Some(format!(
                "{}: cannot write log: {}",
                self.path.display(),
                error
            ));
        }

        match self.error {
            Some(message) => Err(message),
            None => Ok(()),
        }
    }

    /// Writes one row with the columns of `HEADER`
    fn write_row(&mut self, state: &SimulationState) {
        let car = &state.car;
        let suspension = &car.suspension;
        let row = format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            state.time,
            car.distance,
            car.speed,
            car.acceleration,
            car.gear,
            car.revs_hz,
            car.braking,
            car.shift_time_left,
            car.active_braking_zone,
            suspension.heave,
            suspension.heave_velocity,
            suspension.pitch,
            suspension.pitch_rate,
            car.lap,
            car.x,
            car.y,
            car.curvature,
            car.corner_speed_limit,
            car.target_speed,
            car.lateral_acceleration,
            car.over_grip_limit,
        );
        self.write_line(&row);
    }

    /// Writes one line, remembering the first error instead of stopping the simulation
    fn write_line(&mut self, line: &str) {
        if self.error.is_some() {
            return;
        }

        if let Err(error) = writeln!(self.writer, "{}", line) {
            self.error = Some(format!(
                "{}: cannot write log: {}",
                self.path.display(),
                error
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::physics::car_state_log::{CarStateLog, HEADER};
    use crate::physics::simulation::{EndCondition, SimulationConfig, start_simulation};
    use std::fs;

    #[test]
    fn test_log_rows() {
        // 10 steps of 1 ms, a row every 4 steps: steps 0, 4, 8 and the last step 10
        let path = std::env::temp_dir().join(format!("car_state_log_{}.csv", std::process::id()));
        let mut simulation_config = SimulationConfig {
            end_condition: EndCondition::Time { end_time: 0.01 },
            ..SimulationConfig::default()
        };

        let mut log = CarStateLog::create(&path, 4).unwrap();
        let final_state = start_simulation(&mut simulation_config, Some(&mut log), false);
        log.finish().unwrap();

        let text = fs::read_to_string(&path).unwrap();
        fs::remove_file(&path).unwrap();
        let lines: Vec<&str> = text.lines().collect();

        assert_eq!(lines[0], HEADER.join(","));
        assert_eq!(lines.len(), 1 + 4, "Header + 4 rows: {:?}", lines);

        let times: Vec<f64> = lines[1..]
            .iter()
            .map(|line| line.split(',').next().unwrap().parse().unwrap())
            .collect();
        let expected = [0.0, 0.004, 0.008, 0.01];
        for (time, expected_time) in times.iter().zip(expected) {
            assert!((time - expected_time).abs() < 1e-12, "Times: {:?}", times);
        }

        // The last row is the final state
        let last: Vec<&str> = lines[4].split(',').collect();
        assert_eq!(last[2].parse::<f64>().unwrap(), final_state.car.speed);
        assert_eq!(last.len(), HEADER.len());
    }

    #[test]
    fn test_log_interval_zero_rejected() {
        let path = std::env::temp_dir().join("car_state_log_never_created.csv");
        assert!(CarStateLog::create(&path, 0).is_err());
    }
}
