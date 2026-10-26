pub mod actuator;
pub mod blackbox;
pub mod flight_loop;
pub mod receiver;
#[cfg(any(feature = "baro", feature = "compass"))]
pub mod sensors_ext;
pub mod telemetry;

pub use actuator::actuator_pwm_task;
pub use blackbox::{blackbox_task, usb_task};
#[cfg(not(feature = "dshot"))]
pub use flight_loop::calibrate_esc_if_requested;
pub use flight_loop::{calibrate_imu_and_gyro, FlightLoopState};
pub use receiver::receiver_task;
#[cfg(any(feature = "baro", feature = "compass"))]
pub use sensors_ext::{external_sensors_task, ExternalSensorSnapshot};
#[cfg(feature = "crsf")]
pub use telemetry::{crsf_telemetry_task, FlightTelemetrySnapshot};
pub use telemetry::{telemetry_led_task, VisualStatus};

