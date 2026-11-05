pub mod physics;

use physics::car_state_log::CarStateLog;
use physics::config::load_run;
use physics::simulation::start_simulation;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Instant;

/// How to start the program, printed when the arguments are wrong
const USAGE: &str = "Usage: cargo run -- <run file> [--log <csv file>] [--log-every <steps>]
  e.g. cargo run -- data/runs/baseline.toml
       cargo run --release -- data/runs/baseline.toml --log out/baseline.csv --log-every 10
  --log <csv file>     write the car state to a CSV file (overwritten if it exists)
  --log-every <steps>  write a row every this many steps (default 1); the first and the last
                       step are always written";

/// Command line arguments
struct Arguments {
    /// Run file to simulate
    run_path: PathBuf,
    /// CSV file to log the car state to, if any
    log_path: Option<PathBuf>,
    /// A log row every this many steps
    log_every_steps: u64,
}

fn main() -> ExitCode {
    // 1. Read the arguments and load the config from the run file
    let arguments = match parse_arguments(std::env::args().skip(1)) {
        Ok(arguments) => arguments,
        Err(message) => {
            eprintln!("Error: {}\n{}", message, USAGE);
            return ExitCode::FAILURE;
        }
    };

    let mut config = match load_run(&arguments.run_path) {
        Ok(config) => config,
        Err(message) => {
            eprintln!("Error: {}", message);
            return ExitCode::FAILURE;
        }
    };

    let mut log = match &arguments.log_path {
        Some(path) => match CarStateLog::create(path, arguments.log_every_steps) {
            Ok(log) => Some(log),
            Err(message) => {
                eprintln!("Error: {}", message);
                return ExitCode::FAILURE;
            }
        },
        None => None,
    };

    println!("Launching vehicle dynamics simulation...");

    // 2. Run the simulation end-to-end
    let start = Instant::now();
    let final_state = start_simulation(&mut config, log.as_mut(), false);
    let duration = start.elapsed();

    println!("Simulation finished in: {:?}", duration);

    if let Some(log) = log {
        if let Err(message) = log.finish() {
            eprintln!("Error: {}", message);
            return ExitCode::FAILURE;
        }
        if let Some(path) = &arguments.log_path {
            println!("Car state log written to {}", path.display());
        }
    }

    // 3. Inspect the final output
    println!("\nSimulation Complete!");
    println!("Final Time    : {:.3} s", final_state.time);
    println!("Final Speed   : {:.2} m/s", final_state.car.speed);
    println!("Final Distance: {:.2} m", final_state.car.distance);

    ExitCode::SUCCESS
}

/// Reads the run file path and the optional `--log` and `--log-every` flags
fn parse_arguments(mut arguments: impl Iterator<Item = String>) -> Result<Arguments, String> {
    let mut run_path = None;
    let mut log_path = None;
    let mut log_every_steps = 1;

    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--log" => {
                let path = arguments.next().ok_or("--log needs a file name")?;
                log_path = Some(PathBuf::from(path));
            }
            "--log-every" => {
                let steps = arguments
                    .next()
                    .ok_or("--log-every needs a number of steps")?;
                log_every_steps = steps
                    .parse()
                    .ok()
                    .filter(|&steps| steps > 0)
                    .ok_or(format!(
                        "--log-every must be a whole number >= 1, got `{}`",
                        steps
                    ))?;
            }
            _ if argument.starts_with("--") => {
                return Err(format!("unknown option `{}`", argument));
            }
            _ if run_path.is_none() => run_path = Some(Path::new(&argument).to_path_buf()),
            _ => return Err(format!("only one run file allowed, got `{}` too", argument)),
        }
    }

    Ok(Arguments {
        run_path: run_path.ok_or("no run file given")?,
        log_path,
        log_every_steps,
    })
}

#[cfg(test)]
mod tests {
    use super::parse_arguments;
    use std::path::PathBuf;

    fn parse(arguments: &[&str]) -> Result<super::Arguments, String> {
        parse_arguments(arguments.iter().map(|argument| argument.to_string()))
    }

    #[test]
    fn test_parse_arguments() {
        let arguments = parse(&["run.toml"]).unwrap();
        assert_eq!(arguments.run_path, PathBuf::from("run.toml"));
        assert!(arguments.log_path.is_none());
        assert_eq!(arguments.log_every_steps, 1);

        // Flags may come before or after the run file
        let arguments = parse(&["--log", "out.csv", "run.toml", "--log-every", "10"]).unwrap();
        assert_eq!(arguments.run_path, PathBuf::from("run.toml"));
        assert_eq!(arguments.log_path, Some(PathBuf::from("out.csv")));
        assert_eq!(arguments.log_every_steps, 10);

        for bad in [
            &[][..],
            &["run.toml", "--log"],
            &["run.toml", "--log-every", "0"],
            &["run.toml", "--log-every", "ten"],
            &["run.toml", "--verbose"],
            &["run.toml", "other.toml"],
        ] {
            assert!(parse(bad).is_err(), "{:?} accepted", bad);
        }
    }
}
