/// Struct that holds information about the track on which the car is tested
pub struct Track {
    /// Track length in meters, m
    track_length: f64, // m
    /// Braking zone starts and ends in meters, m
    braking_zones: Vec<(f64, f64)>, // start-end in meters, m
}


/// Function checks if there are braking zones or not
pub(crate) fn is_braking_zone(position_on_track: f64, braking_zones: &Vec<(f64, f64)>) -> bool {
    for braking_zone in braking_zones.iter() {
        if braking_zone.0 <= position_on_track && position_on_track <= braking_zone.1 {
            return true;
        }
    }
    false
}


#[cfg(test)]
mod tests {
    use super::*;


    #[test]
    fn test_is_braking_zone() {
        let track = Track {
            track_length: 1000.0, // m
            braking_zones: vec![(200.0, 300.0), (300.0, 400.0), (450.0, 500.0)], // [(m, m)],
        };

        let position_on_track: f64 = 0.0;

        for _ in 0..20 {
            if position_on_track >= 200.0 && position_on_track <= 300.0 {
                assert!(is_braking_zone(position_on_track, &track.braking_zones), "Did not register braking zone");
            } else if position_on_track >= 300.0 && position_on_track <= 400.0 {
                assert!(is_braking_zone(position_on_track, &track.braking_zones), "Did not register braking zone");
            } else if position_on_track >= 450.0 && position_on_track <= 500.0 {
                assert!(is_braking_zone(position_on_track, &track.braking_zones), "Did not register braking zone");
            } else {
                assert_eq!(is_braking_zone(position_on_track, &track.braking_zones), false, "Register braking zone which doesnt exist");
            }
        }
    }

}

