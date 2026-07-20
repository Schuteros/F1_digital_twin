

/// Calculates rolling friction force from tires knowing tire model
fn calculate_force_tyre_friction_rolling(normal_force: f64, mu_rolling_friction: f64) -> f64 {
    let force_rolling_friction: f64 = normal_force * mu_rolling_friction; // Newtons (N)

    force_rolling_friction
}


/// Calculates breakaway friction force from tires knowing tire model
fn calculate_force_tyre_friction_breakaway(normal_force: f64, mu_breakaway_friction: f64) -> f64 {
    let force_breakaway_friction: f64 = normal_force * mu_breakaway_friction; // Newtons (N)

    force_breakaway_friction
}


/// Calculates tire static friction which correlates to accessible traction
fn calculate_force_static_friction(normal_force: f64, mu_static_friction: f64) -> f64 {
    let force_static_friction: f64 = normal_force * mu_static_friction; // Newtons (N)

    force_static_friction
}