pub use fmu_from_struct::prelude::*;

#[derive(Debug, Default, Clone, Fmu)]
#[fmi_version = 3]
pub struct PropulsionControlSystem {
    #[input]
    pub minimum_consumption_kgps: f64, // kg/s
    pub setpoint_consumption_kgps: f64, // kg/s
    pub setpoint_speed_mps: f64,        // m/s
    pub actual_consumption_kgps: f64,   // kg/s
    pub actual_speed_mps: f64,          // m/s

    #[output]
    pub mode: i32, // 0=minimum consumption mode, 1=consumption mode, 2=speed mode
    pub lever_order: f64,                                      // 0..1
    pub actual_consumption_relative_minimum_consumption: f64,  // 0..1
    pub actual_speed_relative_setpoint_speed: f64,             // 0..1
    pub actual_consumption_relative_setpoint_consumption: f64, // 0..1
}

fn calculate_relative_value(actual: f64, setpoint: f64) -> f64 {
    if actual == 0.0 {
        0.0
    } else {
        (setpoint - actual) / actual
    }
}

impl FmuFunctions for PropulsionControlSystem {
    fn do_step(&mut self, _current_time: f64, _time_step: f64) {
        // Calculate relative values
        self.actual_consumption_relative_minimum_consumption =
            calculate_relative_value(self.actual_consumption_kgps, self.minimum_consumption_kgps);
        self.actual_consumption_relative_setpoint_consumption =
            calculate_relative_value(self.actual_consumption_kgps, self.setpoint_consumption_kgps);
        self.actual_speed_relative_setpoint_speed =
            calculate_relative_value(self.actual_speed_mps, self.setpoint_speed_mps);

        if self.actual_consumption_relative_minimum_consumption
            > f64::min(
                self.actual_consumption_relative_setpoint_consumption,
                self.actual_speed_relative_setpoint_speed,
            )
        {
            self.lever_order =
                self.lever_order + self.actual_consumption_relative_minimum_consumption;
            self.mode = 0;
        } else if self.actual_consumption_relative_setpoint_consumption
            < self.actual_speed_relative_setpoint_speed
        {
            self.lever_order =
                self.lever_order + self.actual_consumption_relative_setpoint_consumption;
            self.mode = 1;
        } else {
            self.lever_order = self.lever_order + self.actual_speed_relative_setpoint_speed;
            self.mode = 2;
        }

        self.lever_order = f64::max(0.0, f64::min(1.0, self.lever_order));
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_mode_0_minimum_consumption() {
        let mut sys = PropulsionControlSystem {
            minimum_consumption_kgps: 10.0,
            setpoint_consumption_kgps: 20.0,
            setpoint_speed_mps: 10.0,
            actual_consumption_kgps: 5.0,
            actual_speed_mps: 10.0,
            lever_order: 0.5,
            ..Default::default()
        };

        sys.do_step(0.0, 1.0);
        assert_eq!(sys.mode, 0); // Minimum consumption mode
        assert!(sys.lever_order > 0.5);
    }

    #[test]
    fn test_mode_1_consumption() {
        let mut sys = PropulsionControlSystem {
            minimum_consumption_kgps: 10.0,
            setpoint_consumption_kgps: 20.0,
            setpoint_speed_mps: 10.0,
            actual_consumption_kgps: 22.0,
            actual_speed_mps: 10.0,
            lever_order: 0.5,
            ..Default::default()
        };

        sys.do_step(0.0, 1.0);
        assert_eq!(sys.mode, 1); // Consumption mode
        assert!(sys.lever_order < 0.5);
    }

    #[test]
    fn test_mode_2_speed() {
        let mut sys = PropulsionControlSystem {
            minimum_consumption_kgps: 10.0,
            setpoint_consumption_kgps: 20.0,
            setpoint_speed_mps: 10.0,
            actual_consumption_kgps: 18.0,
            actual_speed_mps: 11.0,
            lever_order: 0.5,
            ..Default::default()
        };

        sys.do_step(0.0, 1.0);
        assert_eq!(sys.mode, 2); // Speed mode
        assert!(sys.lever_order < 0.5);
    }

    #[test]
    fn test_mode_0_minimum_consumption_no_change() {
        let mut sys = PropulsionControlSystem {
            minimum_consumption_kgps: 10.0,
            setpoint_consumption_kgps: 20.0,
            setpoint_speed_mps: 10.0,
            actual_consumption_kgps: 10.0,
            actual_speed_mps: 10.0,
            lever_order: 0.5,
            ..Default::default()
        };

        sys.do_step(0.0, 1.0);
        assert_eq!(sys.mode, 2); // This will go into speed mode since comparisons are made  not including  equalities
        assert!(sys.lever_order == 0.5);
    }

    #[test]
    fn test_mode_1_consumption_no_change() {
        let mut sys = PropulsionControlSystem {
            minimum_consumption_kgps: 10.0,
            setpoint_consumption_kgps: 20.0,
            setpoint_speed_mps: 10.0,
            actual_consumption_kgps: 20.0,
            actual_speed_mps: 10.0,
            lever_order: 0.5,
            ..Default::default()
        };

        sys.do_step(0.0, 1.0);
        assert_eq!(sys.mode, 2); // This will go into speed mode since comparisons are made  not including  equalities
        assert!(sys.lever_order == 0.5);
    }

    #[test]
    fn test_mode_priority() {
        let mut sys = PropulsionControlSystem {
            minimum_consumption_kgps: 5.0,
            setpoint_consumption_kgps: 10.0,
            setpoint_speed_mps: 10.0,
            actual_consumption_kgps: 9.0,
            actual_speed_mps: 11.0,
            lever_order: 0.5,
            ..Default::default()
        };

        sys.do_step(0.0, 1.0);
        assert_eq!(sys.mode, 2);
        assert!(sys.lever_order < 0.5);
    }

    #[test]
    fn test_infinite_speed_setpoint_results_in_consumption_mode() {
        let mut sys = PropulsionControlSystem {
            minimum_consumption_kgps: 5.0,
            setpoint_consumption_kgps: 10.0,
            setpoint_speed_mps: f64::INFINITY,
            actual_consumption_kgps: 9.0,
            actual_speed_mps: 11.0,
            lever_order: 0.5,
            ..Default::default()
        };

        sys.do_step(0.0, 1.0);
        assert_eq!(sys.mode, 1);
        assert!(sys.lever_order > 0.5);
    }
}
