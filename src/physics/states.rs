#[derive(Debug, Clone, Copy)]
pub struct CarState {
    pub speed: f64,
    pub distance: f64,
    pub current_gear: u8,
    pub current_revs: f64,
    pub active_braking_zone: usize,
    pub braking: bool,
}

impl Default for CarState {
    fn default() -> Self {
        Self {
            speed: 10.0,
            distance: 100.0,
            current_gear: 1,
            current_revs: 0.5,
            active_braking_zone: 0,
            braking: false,
        }
    }
}
