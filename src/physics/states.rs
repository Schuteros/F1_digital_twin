#[derive(Debug, Clone, Copy)]
pub struct CarState {
    pub speed: f64,
    pub distance: f64,
    pub current_gear: u8,
    pub current_revs: f64,
    pub active_braking_zone: usize
}
