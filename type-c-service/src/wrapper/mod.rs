//! This module contains the `Controller` trait. Any types that implement this trait can be used with the `ControllerWrapper` struct
//! which provides a bridge between various service messages and the actual controller functions.
use core::array::from_fn;
use core::cell::{Cell, RefCell};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
//use core::borrow::BorrowMut;
use embassy_futures::select::{select3, select_array, Either3};
use embedded_services::power::policy::device::StateKind;
use embedded_services::power::policy::{self, action};
use embedded_services::type_c::controller::{self, Controller, PortStatus};
use embedded_services::type_c::event::{PortEventFlags, PortEventKind};
use embedded_services::{error, info, intrusive_list, trace, warn};
use embedded_usb_pd::{type_c::Current as TypecCurrent, Error, PdError, PortId as LocalPortId};
//use subsystem_soc::ssh::set_debug_card_status;
mod pd;
mod power;

static DBG_CARD_STS: Mutex<ThreadModeRawMutex, RefCell<RecordDbgCard>> = Mutex::new(RefCell::new(RecordDbgCard::new()));
#[derive(Debug, Clone)]
pub struct RecordDbgCard {
    pub debug_card_connect: u8,
    pub dedicate_port: Debug_Card_Port,
    pub initial: bool,
}

impl RecordDbgCard {
    const fn new() -> Self {
        Self {
            debug_card_connect: 0,
            dedicate_port: Debug_Card_Port::global_port_0,
            initial: false,
        }
    }
}

pub async fn set_debug_card_port(select_port: Debug_Card_Port) {
    let dbg_temp = DBG_CARD_STS.lock().await;
    //let mut assign_port = dbg_temp.borrow_mut();
    //assign_port.dedicate_port = select_port;
    let mut assign_port = dbg_temp.borrow_mut();

    assign_port.dedicate_port = select_port;
    assign_port.initial = true;
}

pub async fn set_debug_card_status(status_update: u8, which_Port: u8) {
    let dbg_temp = DBG_CARD_STS.lock().await;
    let mut assign_status = dbg_temp.borrow_mut();
    if assign_status.initial == true {
        if assign_status.dedicate_port as u8 == which_Port {
            assign_status.debug_card_connect = status_update;
        } else {
            error!("Inserting Debug card in incorrect port!!");
        }
    } else {
        error!("Never assign Debug port");
    }
}

pub async fn get_debug_card_status() -> u8 {
    let dbg_temp = DBG_CARD_STS.lock().await;
    //let report_status = dbg_temp.borrow_mut();
    //let sts = report_status.debug_card_connect;
    //return sts;
    let report = dbg_temp.borrow().debug_card_connect;
    return report;
}
#[derive(Debug, Clone, Copy)]
pub enum Debug_Card_Port {
    global_port_0,
    global_port_1,
    global_port_2,
}

//static BATTERY_INFO: Mutex<ThreadModeRawMutex, RefCell<BattInfo>> =
//    Mutex::new(RefCell::new(BattInfo::new()));

//pub static dbg_card_sts: Record_dbg_card = Record_dbg_card{debug_card_connect: false, dedicate_port:0, initial:false};
/*
pub async fn dbg_card_detect_init(select_port: u8) {
    dbg_card_sts.borrow_mut().dedicate_port = select_port;
    dbg_card_sts.initial = true;
}

fn update_debug_card_status(dbg_sts:bool, whichPort: u8)
{
    if dbg_card_sts.dedicate_port == whichPort{
        dbg_card_sts.debug_card_connect = dbg_sts;
    }
}
pub fn get_debug_card_status() -> bool{
    let dbg_connect = dbg_card_sts.debug_card_connect;
    return dbg_connect;
}
*/

//fn Update_Debug_Card_Status(&mut self, status: bool, port: u8){
//if self.whichPort == port //Only for Port 0
//{
//    self.debug_card_connect = status;
//    self.State_updated = true;
//}

//pub async fn Get_Debug_Card_Status(&mut self) -> bool{
//    if self.State_updated == true
//    {
//        return self.debug_card_connect;
//    }
//    else
//    {
//        return false;
//    }
//}

//impl Record_dbg_card {
//    fn new()-> Self{
//        Self{debug_card_connect, State_updated, whichPort}
//    }
//    async fn init (&mut self){
//        self.debug_card_connect = false;
//        self.whichPort = 0;
//        self.State_updated = false;
//    }
//
//    async
//    }

//}

//static mut debug_card_connect: bool = false;
//static mut State_updated: bool = false;
//static mut whichPort: u8 = 0;

//pub fn init_detect_debug_card() {
//    unsafe {
//        debug_card_connect = false;
//        whichPort = 0;
//        State_updated = false;
//    }
//}

//pub fn Update_Debug_Card_Status(status: bool, port: u8) {
//    unsafe {
//        whichPort = port;
//
//        if (port == 0)
//        {
//            debug_card_connect = status;
//            State_updated = true;
//        }
//
//    }
//}

//pub fn Get_Debug_Card_Status() -> bool {
//    unsafe {
//        if State_updated {
//                debug_card_connect
//        } else {
//            false
//        }
//    }
//}

/// Default current to source
const DEFAULT_SOURCE_CURRENT: TypecCurrent = TypecCurrent::Current1A5;
/// Threshold power capability before we'll attempt to sink from a dual-role supply
/// This ensures we don't try to sink from something like a phone
const DUAL_ROLE_CONSUMER_THRESHOLD_MW: u32 = 15000;

/// Takes an implementation of the `Controller` trait and wraps it with logic to handle
/// message passing and power-policy integration.
pub struct ControllerWrapper<'a, const N: usize, C: Controller> {
    /// PD controller to interface with PD service
    pd_controller: controller::Device<'a>,
    /// Power policy devices to interface with power policy service
    power: [policy::device::Device; N],
    controller: RefCell<C>,
    active_events: [Cell<PortEventKind>; N],
}

impl<'a, const N: usize, C: Controller> ControllerWrapper<'a, N, C> {
    /// Create a new controller wrapper
    pub fn new(pd_controller: controller::Device<'a>, power: [policy::device::Device; N], controller: C) -> Self {
        Self {
            pd_controller,
            power,
            controller: RefCell::new(controller),
            active_events: [const { Cell::new(PortEventKind::none()) }; N],
        }
    }
    /// Handle a plug event
    async fn process_plug_event(
        &self,
        controller: &mut C,
        power: &policy::device::Device,
        port: LocalPortId,
        status: &PortStatus,
    ) -> Result<(), Error<C::BusError>> {
        if port.0 > N as u8 {
            error!("Invalid port {}", port.0);
            return PdError::InvalidPort.into();
        }

        info!("Plug event");
        if status.is_connected() {
            info!("Plug inserted");
            //dbg_card_detect_init(0u8);
            // Recover if we're not in the correct state
            if power.state().await.kind() != StateKind::Detached {
                warn!("Power device not in detached state, recovering");
                if let Err(e) = power.detach().await {
                    error!("Error detaching power device: {:?}", e);
                    return PdError::Failed.into();
                }
            }

            if let Ok(state) = power.try_device_action::<action::Detached>().await {
                if let Err(e) = state.attach().await {
                    error!("Error attaching power device: {:?}", e);
                    return PdError::Failed.into();
                }
            } else {
                // This should never happen
                error!("Power device not in detached state");
                return PdError::InvalidMode.into();
            }
        } else {
            info!("Plug removed");

            // Reset source enable to default
            if controller.set_sourcing(port, true).await.is_err() {
                error!("Error setting source enable to default");
                return PdError::Failed.into();
            }

            // Don't signal since we're disconnected and just resetting to our default value
            if controller
                .set_source_current(port, DEFAULT_SOURCE_CURRENT, false)
                .await
                .is_err()
            {
                error!("Error setting source current to default");
                return PdError::Failed.into();
            }

            if let Err(e) = power.detach().await {
                error!("Error detaching power device: {:?}", e);
                return PdError::Failed.into();
            };
        }

        Ok(())
    }

    /// Process port events
    /// None of the event processing functions return errors to allow processing to continue for other ports on a controller
    async fn process_event(&self, controller: &mut C) {
        let mut port_events = PortEventFlags::none();

        for port in 0..N {
            let local_port_id = LocalPortId(port as u8);
            let global_port_id = match self.pd_controller.lookup_global_port(local_port_id) {
                Ok(port) => port,
                Err(_) => {
                    error!("Invalid local port {}", local_port_id.0);
                    continue;
                }
            };

            let event = match controller.clear_port_events(local_port_id).await {
                Ok(event) => event,
                Err(_) => {
                    error!("Error clearing port events",);
                    continue;
                }
            };

            if event == PortEventKind::none() {
                self.active_events[port].set(PortEventKind::none());
                continue;
            }

            port_events.pend_port(global_port_id);

            let status = match controller.get_port_status(local_port_id).await {
                Ok(status) => status,
                Err(_) => {
                    error!("Port{}: Error getting port status", global_port_id.0);
                    continue;
                }
            };
            trace!("Port{} status: {:#?}", port, status);

            let mut debug_card_detect: u8;
            if status.is_connected() && status.is_debug_accessory() {
                //if status.is_debug_accessory() {
                debug_card_detect = 1;
            } else {
                debug_card_detect = 0;
            }
            set_debug_card_status(debug_card_detect, global_port_id.0).await;

            // if status.is_connected() {
            //     if global_port_id.0 == 0 {
            //         if status.is_debug_accessory() {
            //             debug_card_detect = true;
            //         } else {
            //             debug_card_detect = false;
            //         }
            //     } else {
            //         debug_card_detect = false;
            //     }
            // } else {
            //     debug_card_detect = false;
            // }

            // Update_Debug_Card_Status(debug_card_detect, global_port_id.0);
            //debug_card_status.Update_Debug_Card_Status(debug_card_detect, global_port_id.0);

            let power = match self.get_power_device(local_port_id) {
                Ok(power) => power,
                Err(_) => {
                    error!("Port{}: Error getting power device", global_port_id.0);
                    continue;
                }
            };

            trace!("Port{} Interrupt: {:#?}", global_port_id.0, event);
            if event.plug_inserted_or_removed()
                && self
                    .process_plug_event(controller, power, local_port_id, &status)
                    .await
                    .is_err()
            {
                error!("Port{}: Error processing plug event", global_port_id.0);
                continue;
            }

            if event.new_power_contract_as_consumer()
                && self
                    .process_new_consumer_contract(controller, power, local_port_id, &status)
                    .await
                    .is_err()
            {
                error!("Port{}: Error processing new consumer contract", global_port_id.0);
                continue;
            }

            if event.new_power_contract_as_provider()
                && self
                    .process_new_provider_contract(global_port_id, power, &status)
                    .await
                    .is_err()
            {
                error!("Port{}: Error processing new provider contract", global_port_id.0);
                continue;
            }

            self.active_events[port].set(event);
        }

        self.pd_controller.notify_ports(port_events).await;
    }

    /// Top-level processing function
    #[allow(clippy::await_holding_refcell_ref)]
    pub async fn process(&self) {
        let mut controller = self.controller.borrow_mut();
        match select3(
            controller.wait_port_event(),
            self.wait_power_command(),
            self.pd_controller.wait_command(),
        )
        .await
        {
            Either3::First(r) => match r {
                Ok(_) => self.process_event(&mut controller).await,
                Err(_) => error!("Error waiting for port event"),
            },
            Either3::Second((command, port)) => self.process_power_command(&mut controller, port, command).await,
            Either3::Third(command) => self.process_pd_command(&mut controller, command).await,
        }
    }

    /// Register all devices with their respective services
    pub async fn register(&'static self) -> Result<(), intrusive_list::Error> {
        for device in &self.power {
            policy::register_device(device).await?
        }

        controller::register_controller(&self.pd_controller).await
    }
}
