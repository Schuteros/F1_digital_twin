#[derive(Debug, Clone)]
pub(crate) struct CarState {
    pub(crate) force: f64,
    pub(crate) acceleration: f64,
    pub(crate) velocity: f64,
    pub(crate) displacement: f64
}