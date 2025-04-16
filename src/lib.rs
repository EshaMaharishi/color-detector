use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use std::time::Instant;
use std::thread;
use std::time::Duration;

use micro_rdk::DoCommand;
use micro_rdk::common::config::ConfigType;
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::common::registry::{get_board_from_dependencies, ComponentRegistry, RegistryError, Dependency};

use micro_rdk::common::sensor::{Sensor, SensorType, Readings, SensorError, GenericReadingsResult, SensorResult, TypedReadingsResult, SensorT};
use micro_rdk::common::board::{BoardType};

use micro_rdk::esp32::esp_idf_svc::hal::{
    delay::TickType,
    gpio::{
        enable_isr_service, init_isr_alloc_flags, AnyIOPin, Input, InterruptType, Output,
        PinDriver, Pull,
    },
    task::notification::{Notification, Notifier},
};
use std::sync::atomic::{AtomicU8, Ordering};
use std::cell::RefCell;

use micro_rdk::esp32::esp_idf_svc::sys::{esp, gpio_isr_handler_add, gpio_isr_handler_remove};

const NUM_CYCLES: u8 = 10;
struct ImpulseCounter {
    impulse_count: AtomicU8,
    notifier: Arc<Notifier>,
    pin_num: i32
}

pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("my_sensor", &MySensor::from_config)
}

unsafe impl Send for MySensor {}

#[derive(DoCommand)]
pub struct MySensor {
    board_handle: BoardType,
    impulse_counter: Arc<ImpulseCounter>,
    interrupt_notification: Notification,
    interrupt_pin: RefCell<PinDriver<'static, AnyIOPin, Input>>
}

impl MySensor {
    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType,SensorError> {

        init_isr_alloc_flags(micro_rdk::esp32::esp_idf_svc::hal::interrupt::InterruptType::Iram.into());
        log::info!("my_sensor sensor from_config 1");

        enable_isr_service().map_err(|err| SensorError::SensorCodeError(err.code()))?;
        log::info!("my_sensor sensor from_config 2");

        let board_handle = get_board_from_dependencies(deps)
            .expect("failed to get board from dependencies");
        log::info!("my_sensor sensor from_config 3");

        let interrupt_pin_num = cfg
            .get_attribute::<i32>("interrupt_pin")
            .map_err(|_| SensorError::ConfigError("missing `interrupt_pin`"))?;  
        log::info!("my_sensor sensor from_config 4");


        let interrupt_pin = RefCell::new(PinDriver::input(unsafe { AnyIOPin::new(interrupt_pin_num) }).map_err(|err| SensorError::SensorCodeError(err.code()))?);
        log::info!("my_sensor sensor from_config 5");

        let interrupt_notification = Notification::new();
        let notifier = interrupt_notification.notifier();
        let impulse_counter = Arc::new(ImpulseCounter{impulse_count: AtomicU8::new(0), notifier, pin_num: interrupt_pin_num});
        log::info!("my_sensor sensor from_config 6");


        let sensor = Self {board_handle, impulse_counter, interrupt_notification, interrupt_pin};

        sensor.interrupt_pin.borrow_mut().set_pull(Pull::Up).map_err(|err| SensorError::SensorCodeError(err.code()))?;
        sensor.interrupt_pin.borrow_mut().set_interrupt_type(InterruptType::NegEdge);
        log::info!("my_sensor sensor from_config 7");

        Ok(Arc::new(Mutex::new(sensor)))
    }

    #[inline(always)]
    #[link_section = ".iram1.intr_srv"]
    unsafe extern "C" fn subscription_interrupt(arg: *mut core::ffi::c_void) {
        let arg: &mut ImpulseCounter = &mut *(arg as *mut _);
        let count = arg.impulse_count.fetch_add(1, Ordering::SeqCst);

        if count == 10 {
//            arg.impulse_count.store(0, Ordering::SeqCst);
            micro_rdk::esp32::esp_idf_svc::sys::esp_rom_printf(b"impulse count reset!\n\0".as_ptr() as *const i8);
            arg.notifier.notify_and_yield(std::num::NonZeroU32::new(30).unwrap());
            let pin = arg.pin_num;
            if let Err(error) = unsafe { esp!(gpio_isr_handler_remove(pin)) } {
                micro_rdk::esp32::esp_idf_svc::sys::esp_rom_printf(b"failed to remove the handler!\n\0".as_ptr() as *const i8);
            }
       }

//        micro_rdk::esp32::esp_idf_svc::sys::esp_rom_printf(b"impulse count incremented!\n\0".as_ptr() as *const i8);
   }
}

/*
impl Drop for MySensor {
    fn drop(&mut self) {
        let pin = self.interrupt_pin.borrow_mut().pin();
        if let Err(error) = unsafe { esp!(gpio_isr_handler_remove(pin)) } {
            log::warn!(
                "failed to remove interrupt handler for pin {}: {}",
                pin,
                error
            )
        }
    } 
}
*/

impl Sensor for MySensor {}

impl SensorT<f64> for MySensor {
    fn get_readings(&self) ->  Result<TypedReadingsResult<f64>, SensorError> {
        log::info!("my_sensor GetReadings called");
        let mut map = HashMap::new();

        // Set pins 22 and 23 to low to get 'red' signal
        self.board_handle.lock().unwrap().set_gpio_pin_level(22, false);
        self.board_handle.lock().unwrap().set_gpio_pin_level(23, false);
        log::info!("my_sensor readings 0");

        thread::sleep(Duration::from_millis(300));

        let start_red = Instant::now();
        unsafe {
            esp!(gpio_isr_handler_add(
                self.interrupt_pin.borrow().pin(),
                Some(Self::subscription_interrupt),
                Arc::as_ptr(&self.impulse_counter) as *mut _,
            ))
            .map_err(|err| SensorError::SensorCodeError(err.code()))?;
        }
        self.interrupt_notification.wait(TickType::from(Duration::from_secs(1)).as_millis_u32());
        let duration_red = start_red.elapsed();
        self.impulse_counter.impulse_count.store(0, Ordering::SeqCst);
        let duration_in_nanos_f64_red = duration_red.as_nanos() as f64;



        // Set pins 22 to low and 23 to high to get 'blue' signal
        self.board_handle.lock().unwrap().set_gpio_pin_level(22, false);
        self.board_handle.lock().unwrap().set_gpio_pin_level(23, true);
        log::info!("my_sensor readings 0");

        thread::sleep(Duration::from_millis(300));

        let start_blue = Instant::now();
        unsafe {
            esp!(gpio_isr_handler_add(
                self.interrupt_pin.borrow().pin(),
                Some(Self::subscription_interrupt),
                Arc::as_ptr(&self.impulse_counter) as *mut _,
            ))
            .map_err(|err| SensorError::SensorCodeError(err.code()))?;
        }

        self.interrupt_notification.wait(TickType::from(Duration::from_secs(1)).as_millis_u32());
        let duration_blue = start_blue.elapsed();
        self.impulse_counter.impulse_count.store(0, Ordering::SeqCst);
        let duration_in_nanos_f64_blue = duration_blue.as_nanos() as f64;



        // Set pins 22 and 23 to high to get 'green' signal
        self.board_handle.lock().unwrap().set_gpio_pin_level(22, true);
        self.board_handle.lock().unwrap().set_gpio_pin_level(23, true);
        log::info!("my_sensor readings 0");

        thread::sleep(Duration::from_millis(300));

        let start_green = Instant::now();
        unsafe {
            esp!(gpio_isr_handler_add(
                self.interrupt_pin.borrow().pin(),
                Some(Self::subscription_interrupt),
                Arc::as_ptr(&self.impulse_counter) as *mut _,
            ))
            .map_err(|err| SensorError::SensorCodeError(err.code()))?;
        }
        self.interrupt_notification.wait(TickType::from(Duration::from_secs(1)).as_millis_u32());
        let duration_green = start_green.elapsed();
        self.impulse_counter.impulse_count.store(0, Ordering::SeqCst);
        let duration_in_nanos_f64_green = duration_green.as_nanos() as f64;

        // Insert items into the HashMap
        // TODO: Instead, return a single key "color" with value "red", "green", or "blue"
        if (duration_in_nanos_f64_red > 600000.0 && duration_in_nanos_f64_red < 1000000.0) {
            map.insert("red".to_string(), 1.0);
        } else if (duration_in_nanos_f64_blue > 400000.0 && duration_in_nanos_f64_blue < 500000.0) {
            map.insert("blue".to_string(), 1.0);
        } else if (duration_in_nanos_f64_green > 1000000.0 && duration_in_nanos_f64_green < 3000000.0) {
            map.insert("green".to_string(), 1.0);
        }

        log::info!("duration_red: {}", duration_in_nanos_f64_red);
        log::info!("duration_blue: {}", duration_in_nanos_f64_blue);
        log::info!("duration_green: {}", duration_in_nanos_f64_green);

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

