pub mod flight_loop;
pub mod receiver;
#[cfg(any(feature = "baro", feature = "compass"))]
pub mod sensors_ext;
pub mod telemetry;

pub use flight_loop::FlightLoopState;
pub use receiver::receiver_task;
#[cfg(any(feature = "baro", feature = "compass"))]
pub use sensors_ext::{external_sensors_task, ExternalSensorSnapshot};
#[cfg(feature = "crsf")]
pub use telemetry::{crsf_telemetry_task, FlightTelemetrySnapshot};
pub use telemetry::{telemetry_led_task, VisualStatus};
