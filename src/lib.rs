use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use std::time::Instant;

use micro_rdk::DoCommand;
use micro_rdk::common::config::ConfigType;
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::common::registry::{get_board_from_dependencies, ComponentRegistry, RegistryError, Dependency};

use micro_rdk::common::sensor::{Sensor, SensorType, Readings, SensorError, GenericReadingsResult, SensorResult, TypedReadingsResult, SensorT};
use micro_rdk::common::board::{BoardType};

pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("my_sensor", &MySensor::from_config)
}

unsafe impl Send for MySensor {}

#[derive(DoCommand)]
pub struct MySensor {
    board_handle: BoardType
}

impl MySensor {
    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType,SensorError> {
        let board_handle = get_board_from_dependencies(deps)
            .expect("failed to get board from dependencies");
        Ok(Arc::new(Mutex::new(Self {board_handle})))
    }
}

impl Sensor for MySensor {}

impl SensorT<f64> for MySensor {
    fn get_readings(&self) ->  Result<TypedReadingsResult<f64>, SensorError> {
        let mut map = HashMap::new();

        // Set pin 23 to high
        //self.board_handle.lock().unwrap().set_gpio_pin_level(23, true);

        let start = Instant::now();

        // Get pin 22 level
        let gpio_level_22 = self.board_handle.lock().unwrap().get_gpio_level(22);
        let gpio_level_22_f64 = match gpio_level_22 {
            Ok(true) => 1.0,
            Ok(false) => 0.0,
            Err(e) => return Err(SensorError::SensorBoardError(e))
        };

        // Get pin 23 level
        let gpio_level_23 = self.board_handle.lock().unwrap().get_gpio_level(23);
        let gpio_level_23_f64 = match gpio_level_23 {
            Ok(true) => 1.0,
            Ok(false) => 0.0,
            Err(e) => return Err(SensorError::SensorBoardError(e))
        };

        // Get pin 25 level
        let gpio_level_25 = self.board_handle.lock().unwrap().get_gpio_level(25);
        let gpio_level_25_f64 = match gpio_level_25 {
            Ok(true) => 1.0,
            Ok(false) => 0.0,
            Err(e) => return Err(SensorError::SensorBoardError(e))
        };

        let duration = start.elapsed();
        //let duration_in_millis_f64 = duration.as_millis() as f64;
        let duration_in_nanos_f64 = duration.as_nanos() as f64;

        // Insert items into the HashMap
        map.insert("pin_22_level".to_string(), gpio_level_22_f64);
        map.insert("pin_23_level".to_string(), gpio_level_23_f64);
        map.insert("pin_25_level".to_string(), gpio_level_25_f64);
        map.insert("duration".to_string(), duration_in_nanos_f64);

        // TypedReadingsResult<f64> is a type alias for HashMap<String, f64>
        Ok(map)
    }
}

impl Readings for MySensor {
    fn get_generic_readings(&mut self) -> Result<GenericReadingsResult, SensorError> {
        Ok(self
            .get_readings()?
            .into_iter()
            .map(|v| (v.0, SensorResult::<f64> { value: v.1 }.into()))
            .collect())        

//        Ok(std::collections::HashMap::new())
    }
}

impl Status for MySensor {
    fn get_status(&self) -> Result<Option<micro_rdk::google::protobuf::Struct>, StatusError> {
        Ok(Some(micro_rdk::google::protobuf::Struct {
            fields: HashMap::new(),
        }))
    }
}

