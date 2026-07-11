#[derive(Debug, Clone)]
pub struct CarState {
    pub(crate) force: f64,
    pub(crate) acceleration: f64,
    pub(crate) velocity: f64,
    pub(crate) displacement: f64
}