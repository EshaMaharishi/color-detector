use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use micro_rdk::DoCommand;
use micro_rdk::common::config::ConfigType;
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::common::registry::{ComponentRegistry, RegistryError, Dependency};

use micro_rdk::common::sensor::{Sensor, SensorType, Readings, SensorError, GenericReadingsResult, SensorResult, TypedReadingsResult, SensorT};


pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("my_sensor", &MySensor::from_config)
}


#[derive(DoCommand)]
pub struct MySensor {}

impl MySensor {
    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType,SensorError> {
        Ok(Arc::new(Mutex::new(MySensor {})))
    }
}

impl Sensor for MySensor {}

impl SensorT<f64> for MySensor {
    fn get_readings(&self) ->  Result<TypedReadingsResult<f64>, SensorError> {
       let mut map = HashMap::new();

        // Insert items into the HashMap
        map.insert("key1".to_string(), 0.0);
        map.insert("key2".to_string(), 1.0);
        map.insert("key3".to_string(), 2.0);                

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

