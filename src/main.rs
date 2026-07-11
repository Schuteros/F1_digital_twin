
/// Physics module folder contains all the needed physics functions in modules: [physics::integrators].
pub mod physics;


pub fn calculate_twin_speed(acceleration: f64, t_start: f64, t_end: f64, t_step: f64, start_speed: f64) -> f64 {
    let mut speed = start_speed;
    let n_steps = ((t_end - t_start) / t_step).round() as usize;

    for _ in 0..n_steps {
        speed += acceleration * t_step;
    }
    speed
}

pub fn calculate_actual_speed(acceleration: f64, t_start: f64, t_end: f64, start_speed: f64) -> f64 {
    start_speed + acceleration * (t_end - t_start)
}

fn main() {
    // initial variables needed:
    let x: f64 = 0.0; // Starting position in meters, m
    let initial_speed: f64 = 0.0;
    let mass: f64 = 10.0; // Mass in kilograms, kg of the F1 car
    let force: f64 = 1.0; // Force in Newtons, N with which the car is propelled forward
    let t_start: f64 = 0.0; // Initial time in seconds, s
    let t_end:f64 = 100.0; // End time in seconds, s
    let t_step_for_calculate_twin_speed:f64 = 0.1; // Step for Euler integration for calculate_twin_speed

    let acceleration: f64  = force / mass;

    let twin_final_speed: f64 = calculate_twin_speed(acceleration, t_start, t_end, t_step_for_calculate_twin_speed, initial_speed);
    let actual_final_speed: f64 = calculate_actual_speed(acceleration, t_start, t_end, initial_speed);

    println!("Actual speed: {}, Twin speed {}", actual_final_speed,  twin_final_speed);
}
