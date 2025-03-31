use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use micro_rdk::DoCommand;
use micro_rdk::common::config::ConfigType;
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::common::registry::{ComponentRegistry, RegistryError, Dependency};

use micro_rdk::common::sensor::{Sensor, SensorType, Readings, SensorError};


pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("my_sensor", &MySensor::from_config)
    log::info!("my_sensor sensor registration ok")
}


#[derive(DoCommand)]
pub struct MySensor {}

impl MySensor {
    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType,SensorError> {
        Ok(Arc::new(Mutex::new(MySensor {})))
    }
}

impl Status for MySensor {
    fn get_status(&self) -> Result<Option<micro_rdk::google::protobuf::Struct>, StatusError> {
        Ok(Some(micro_rdk::google::protobuf::Struct {
            fields: HashMap::new(),
        }))
    }
}

