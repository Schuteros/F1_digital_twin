/// Struct that holds information about the track on which the car is tested
pub struct Track {
    /// Track length in meters, m
    track_length: f64, // m
    /// Braking zone starts and ends in meters, m
    braking_zones: Vec<(f64, f64)>, // start-end in meters, m
}


/// Function checks if there are braking zones or not
pub(crate) fn is_braking_zone(braking_zones: &[(f64, f64)], position: f64, active_zone: usize) -> bool {
    if let Some(&(start, end)) = braking_zones.get(active_zone) {
        position >= start && position <= end
    } else {
        false
    }
}


/// Function checks in which active braking zone are we
pub(crate) fn find_active_braking_zone(distance: f64, braking_zones: &[(f64, f64)], active_braking_zone: usize) -> usize {
    if let Some(&(_, end)) = braking_zones.get(active_braking_zone) {
        if distance > end && active_braking_zone + 1 < braking_zones.len() {
            return active_braking_zone + 1;
        }
    }
    active_braking_zone
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

        let mut position_on_track: f64 = 0.0;
        let mut active_zone: usize = 0;

        for _ in 0..20 {
            if track.braking_zones[active_zone].1 < position_on_track && track.braking_zones.len() > active_zone + 1 {
                active_zone += 1;
            }

            if position_on_track >= 200.0 && position_on_track <= 300.0 {
                assert!(is_braking_zone(&track.braking_zones, position_on_track, active_zone), "Did not register braking zone");
            } else if position_on_track >= 300.0 && position_on_track <= 400.0 {
                assert!(is_braking_zone(&track.braking_zones, position_on_track, active_zone), "Did not register braking zone");
            } else if position_on_track >= 450.0 && position_on_track <= 500.0 {
                assert!(is_braking_zone(&track.braking_zones, position_on_track, active_zone), "Did not register braking zone");
            } else {
                assert!(!is_braking_zone(&track.braking_zones, position_on_track, active_zone), "Registered non existing braking zone");
            }

            position_on_track += 50.0;
        }
    }

    #[test]
    fn test_find_active_braking_zone() {
        let braking_zones = [(51.0, 89.0), (102.0, 202.0), (403.0, 505.0), (703.0, 804.0)];

        let mut distance = 0.0;

        let mut active_braking_zone: usize = 0;

        for _ in 0..20 {
            active_braking_zone = find_active_braking_zone(distance, &braking_zones, active_braking_zone);

            if distance <= braking_zones[0].1 {
                assert_eq!(active_braking_zone, 0, "Did not find active braking zone");
            }

            for i in 1..braking_zones.len() {
                if braking_zones[i-1].1 <= distance && braking_zones[i].1 >= distance {
                    assert_eq!(active_braking_zone, i, "Did not resolve active braking zone");
                    break
                }
            }

            distance += 50.0;
        }
    }

}

