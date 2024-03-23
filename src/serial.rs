//! # Serial
//!
//! Asynchronous serial communication using the interal USART peripherals
//!
//! The serial modules implement the [`Read`] and [`Write`] traits.
//!
//! [`Read`]: embedded_hal::serial::Read
//! [`Write`]: embedded_hal::serial::Write

use core::{
    convert::{Infallible, TryFrom},
    fmt,
    ops::Deref,
};

use crate::{
    gpio::{gpioa, gpiob, gpioc, AF7},
    hal::{blocking, serial, serial::Write},
    pac::{
        rcc::cfgr3::USART1SW_A,
        usart1::{cr1::M_A, cr1::PCE_A, cr1::PS_A, RegisterBlock},
        Interrupt, USART1, USART2, USART3,
    },
    rcc::{self, Clocks},
    time::fixed_point::FixedPoint,
    time::rate::{Baud, Hertz},
    Switch,
};

#[allow(unused_imports)]
use crate::pac::RCC;

use cfg_if::cfg_if;
#[cfg(feature = "enumset")]
use enumset::{EnumSet, EnumSetType};

use crate::dma;

/// Interrupt and status events for the USART transmitter (TX)
///
/// All events can be cleared by [`Serial::clear_tx_event`] or [`Serial::clear_events`].
/// Some events are also cleared on other conditions.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "enumset", derive(EnumSetType))]
#[cfg_attr(not(feature = "enumset"), derive(Copy, Clone, PartialEq, Eq))]
#[non_exhaustive]
pub enum TxEvent {
    /// Transmit data register empty / new data can be sent.
    ///
    /// This event is set by hardware when the content of the TDR register has been transferred
    /// into the shift register. It is cleared by [`Serial`]s [`serial::Write::write()`]
    /// implementation to the TDR register.
    #[doc(alias = "TXE")]
    TransmitDataRegisterEmtpy,
    /// CTS (Clear to Send) event.
    ///
    /// This event is set by hardware when the CTS input toggles, if the CTSE bit is set.
    #[doc(alias = "CTSIF")]
    CtsInterrupt,
    /// Transmission complete
    ///
    /// This event is set by hardware if the transmission of a frame containing data is complete and
    /// if TXE is set.
    /// It is cleared by [`Serial`]s [`serial::Write::write()`] implementaiton to the USART_TDR register.
    #[doc(alias = "TC")]
    TransmissionComplete,
    // Framing error detected in smartcard mode.
    // TODO SmartCard mode currently not implemented.
    // This event is set by hardware when a de-synchronization, excessive noise or a break character
    // is detected.
    // #[doc(alias = "FE")]
    // FramingError,
}

/// Interrupt and status events for the USART receiver (RX)
///
/// All events can be cleared by [`Serial::clear_rx_event`] or [`Serial::clear_events`].
/// Some events are also cleared on other conditions.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "enumset", derive(EnumSetType))]
#[cfg_attr(not(feature = "enumset"), derive(Copy, Clone, PartialEq, Eq))]
#[non_exhaustive]
pub enum RxEvent {
    /// Read data register not empty / new data has been received.
    ///
    /// This event is set by hardware when the content of the RDR shift register has been
    /// transferred to the RDR register.
    /// It is cleared by [`Serial`]s [`serial::Read::read()`] to the USART_RDR register.
    #[doc(alias = "RXNE")]
    ReceiveDataRegisterNotEmpty,
    /// Overrun Error detected.
    ///
    /// This event is set by hardware when the data currently being received in the shift register
    /// is ready to be transferred into the RDR register while
    /// [`RxEvent::ReceiveDataRegisterNotEmpty`] is set.
    ///
    /// See [`Error::Overrun`] for a more detailed description.
    #[doc(alias = "ORE")]
    OverrunError,
    /// Idle line state detected.
    ///
    /// This event is set by hardware when an Idle Line is detected.
    #[doc(alias = "IDLE")]
    Idle,
    /// Parity error detected.
    ///
    /// This event is set by hardware when a parity error occurs in receiver mode.
    ///
    /// Parity can be configured by using [`config::Parity`] to create a [`config::Config`].
    #[doc(alias = "PE")]
    ParityError,
    /// Noise error detected.
    ///
    /// This event is set by hardware when noise is detected on a received frame.
    #[doc(alias = "NF")]
    NoiseError,
    /// Framing error detected
    ///
    /// This event is set by hardware when a de-synchronization, excessive noise or a break character
    /// is detected.
    #[doc(alias = "FE")]
    FramingError,
    /// LIN break
    ///
    /// This bit is set by hardware when the LIN break is detected.
    #[doc(alias = "LBDF")]
    LinBreak,
    /// The received character matched the configured character.
    ///
    /// The matching character can be configured with [`Serial::match_character()`]
    #[doc(alias = "CMF")]
    CharacterMatch,
    /// Nothing was received since the last received character for
    /// [`Serial::receiver_timeout()`] amount of time.
    ///
    /// # Note
    ///
    /// Never set for UART peripheral, which does not have [`ReceiverTimeoutExt`]
    /// implemented.
    #[doc(alias = "RTOF")]
    ReceiverTimeout,
    // TODO(Sh3Rm4n): SmartCard Mode not implemented, no use as of now.
    // EndOfBlock,
    // TODO(Sh3Rm4n): The wakeup from stop mode is alittle bit more complicated:
    // - UESM has to be enabled so that it works (RM0316 29.8.1)
    // - Only works with LSI and HSI (which are not configurable yet)
    // - ...
    // /// The peripheral was woken up from "Stop Mode".
    // ///
    // /// This event is set by hardware, when a wakeup event is detected.
    // ///
    // /// The condition, when it does wake up can be configured via
    // /// [`Serial::set_wakeup_from_stopmode_reason()`]
    // #[doc(alias = "WUF")]
    // WakeupFromStopMode,
}

/// Wrapper enumeration for both transmitter and receiver events.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Event {
    /// USART receiver (RX) events
    Rx(RxEvent),
    /// USART transmitter (TX) events
    Tx(TxEvent),
}

impl From<RxEvent> for Event {
    fn from(rx: RxEvent) -> Self {
        Self::Rx(rx)
    }
}

impl From<TxEvent> for Event {
    fn from(tx: TxEvent) -> Self {
        Self::Tx(tx)
    }
}

/// Check if a receiver (RX) interrupt event happend.
#[inline]
pub fn is_rx_event_triggered(uart: &impl Instance, event: RxEvent) -> bool {
    let isr = uart.isr.read();
    match event {
        RxEvent::ReceiveDataRegisterNotEmpty => isr.rxne().bit(),
        RxEvent::OverrunError => isr.ore().bit(),
        RxEvent::Idle => isr.idle().bit(),
        RxEvent::ParityError => isr.pe().bit(),
        RxEvent::LinBreak => isr.lbdf().bit(),
        RxEvent::NoiseError => isr.nf().bit(),
        RxEvent::FramingError => isr.fe().bit(),
        RxEvent::CharacterMatch => isr.cmf().bit(),
        RxEvent::ReceiverTimeout => isr.rtof().bit(),
        // Event::EndOfBlock => isr.eobf().bit(),
        // Event::WakeupFromStopMode => isr.wuf().bit(),
    }
}

/// Check if a transmitter (TX) interrupt event happend.
#[inline]
pub fn is_tx_event_triggered(uart: &impl Instance, event: TxEvent) -> bool {
    let isr = uart.isr.read();
    match event {
        TxEvent::TransmitDataRegisterEmtpy => isr.txe().bit(),
        TxEvent::CtsInterrupt => isr.ctsif().bit(),
        TxEvent::TransmissionComplete => isr.tc().bit(),
    }
}

/// Check if an interrupt event happend.
#[inline]
pub fn is_event_triggered(uart: &impl Instance, event: Event) -> bool {
    match event {
        Event::Rx(rx_event) => is_rx_event_triggered(uart, rx_event),
        Event::Tx(tx_event) => is_tx_event_triggered(uart, tx_event),
    }
}

/// Clear the given interrupt event flag for a given [`Event`] and USART instance
#[inline]
pub fn clear_event(usart: &mut impl Instance, event: Event) {
    match event {
        Event::Tx(tx_event) => clear_tx_event(usart, tx_event),
        Event::Rx(rx_event) => clear_rx_event(usart, rx_event),
    }
}

/// Clear a transmitter event.
#[inline]
pub fn clear_tx_event(usart: &mut impl Instance, event: TxEvent) {
    usart.icr.write(|w| match event {
        TxEvent::CtsInterrupt => w.ctscf().clear(),
        TxEvent::TransmissionComplete => w.tccf().clear(),
        // Do nothing with this event (only useful for Smartcard, which is not
        // supported right now)
        TxEvent::TransmitDataRegisterEmtpy => w,
    });
}

/// Clear a receiver event.
pub fn clear_rx_event(usart: &mut impl Instance, event: RxEvent) {
    usart.icr.write(|w| match event {
        RxEvent::OverrunError => w.orecf().clear(),
        RxEvent::Idle => w.idlecf().clear(),
        RxEvent::ParityError => w.pecf().clear(),
        RxEvent::LinBreak => w.lbdcf().clear(),
        RxEvent::NoiseError => w.ncf().clear(),
        RxEvent::FramingError => w.fecf().clear(),
        RxEvent::CharacterMatch => w.cmcf().clear(),
        RxEvent::ReceiverTimeout => w.rtocf().clear(),
        // Event::EndOfBlock => w.eobcf().clear(),
        // Event::WakeupFromStopMode => w.wucf().clear(),
        RxEvent::ReceiveDataRegisterNotEmpty => {
            // Flush the register data queue, so that this even will not be thrown again.
            usart.rqr.write(|w| w.rxfrq().set_bit());
            w
        }
    });
}

/// Enables or disables an interrupt for a given event and USART instance.
#[inline]
pub fn configure_interrupt(usart: &mut impl Instance, event: Event, enable: impl Into<Toggle>) {
    match event {
        Event::Tx(tx_event) => configure_tx_interrupt(usart, tx_event, enable),
        Event::Rx(rx_event) => configure_rx_interrupt(usart, rx_event, enable),
    }
}

/// Enable or disable the interrupt for the specified [`TxEvent`].
#[inline]
pub fn configure_tx_interrupt(
    usart: &mut impl Instance,
    event: TxEvent,
    enable: impl Into<Toggle>,
) {
    // Do a round way trip to be convert Into<Toggle> -> bool
    let enable: Toggle = enable.into();
    let enable: bool = enable.into();
    match event {
        TxEvent::TransmitDataRegisterEmtpy => usart.cr1.modify(|_, w| w.txeie().bit(enable)),
        TxEvent::CtsInterrupt => usart.cr3.modify(|_, w| w.ctsie().bit(enable)),
        TxEvent::TransmissionComplete => usart.cr1.modify(|_, w| w.tcie().bit(enable)),
    }
}

/// Enable or disable the interrupt for the specified [`RxEvent`].
#[inline]
pub fn configure_rx_interrupt(
    usart: &mut impl Instance,
    event: RxEvent,
    enable: impl Into<Toggle>,
) {
    // Do a round way trip to be convert Into<Toggle> -> bool
    let enable: Toggle = enable.into();
    let enable: bool = enable.into();

    match event {
        RxEvent::ReceiveDataRegisterNotEmpty => usart.cr1.modify(|_, w| w.rxneie().bit(enable)),
        RxEvent::ParityError => usart.cr1.modify(|_, w| w.peie().bit(enable)),
        RxEvent::LinBreak => usart.cr2.modify(|_, w| w.lbdie().bit(enable)),
        RxEvent::NoiseError | RxEvent::OverrunError | RxEvent::FramingError => {
            usart.cr3.modify(|_, w| w.eie().bit(enable))
        }
        RxEvent::Idle => usart.cr1.modify(|_, w| w.idleie().bit(enable)),
        RxEvent::CharacterMatch => usart.cr1.modify(|_, w| w.cmie().bit(enable)),
        RxEvent::ReceiverTimeout => usart.cr1.modify(|_, w| w.rtoie().bit(enable)),
        // Event::EndOfBlock => self.usart.cr1.modify(|_, w| w.eobie().bit(enable)),
        // Event::WakeupFromStopMode => self.usart.cr3.modify(|_, w| w.wufie().bit(enable)),
    };
}

/// Check whether a transmitter interrupt is enabled.
#[inline]
pub fn is_tx_interrupt_configured(usart: &impl Instance, event: TxEvent) -> bool {
    match event {
        TxEvent::TransmitDataRegisterEmtpy => usart.cr1.read().txeie().is_enabled(),
        TxEvent::CtsInterrupt => usart.cr3.read().ctsie().is_enabled(),
        TxEvent::TransmissionComplete => usart.cr1.read().tcie().is_enabled(),
    }
}

/// Check whether a receiver interrupt was enabled.
#[inline]
pub fn is_rx_interrupt_configured(usart: &impl Instance, event: RxEvent) -> bool {
    match event {
        RxEvent::ReceiveDataRegisterNotEmpty => usart.cr1.read().rxneie().is_enabled(),
        RxEvent::ParityError => usart.cr1.read().peie().is_enabled(),
        RxEvent::LinBreak => usart.cr2.read().lbdie().is_enabled(),
        RxEvent::NoiseError | RxEvent::OverrunError | RxEvent::FramingError => {
            usart.cr3.read().eie().is_enabled()
        }
        RxEvent::Idle => usart.cr1.read().idleie().is_enabled(),
        RxEvent::CharacterMatch => usart.cr1.read().cmie().is_enabled(),
        RxEvent::ReceiverTimeout => usart.cr1.read().rtoie().is_enabled(),
        // Event::EndOfBlock => self.usart.cr1.read().eobie().is_enabled(),
        // Event::WakeupFromStopMode => self.usart.cr3.read().wufie().is_enabled(),
    }
}

/// Check if an interrupt is configured for the [`Event`]
#[inline]
pub fn is_interrupt_configured(usart: &impl Instance, event: Event) -> bool {
    match event {
        Event::Rx(rx_event) => is_rx_interrupt_configured(usart, rx_event),
        Event::Tx(tx_event) => is_tx_interrupt_configured(usart, tx_event),
    }
}

/// Enable or disable interrupt for the specified [`TxEvent`]s.
///
/// Like [`Serial::configure_interrupt`], but instead using an enumset. The corresponding
/// interrupt for every [`TxEvent`] in the set will be enabled, every other interrupt will be
/// **disabled**.
#[cfg(feature = "enumset")]
#[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
pub fn configure_tx_interrupts(usart: &mut impl Instance, events: EnumSet<TxEvent>) {
    for event in events.complement().iter() {
        configure_tx_interrupt(usart, event, false);
    }
    for event in events.iter() {
        configure_tx_interrupt(usart, event, true);
    }
}

/// Enable or disable interrupt for the specified [`RxEvent`]s.
///
/// Like [`Serial::configure_interrupt`], but instead using an enumset. The corresponding
/// interrupt for every [`RxEvent`] in the set will be enabled, every other interrupt will be
/// **disabled**.
#[cfg(feature = "enumset")]
#[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
pub fn configure_rx_interrupts(usart: &mut impl Instance, events: EnumSet<RxEvent>) {
    for event in events.complement().iter() {
        configure_rx_interrupt(usart, event, false);
    }
    for event in events.iter() {
        configure_rx_interrupt(usart, event, true);
    }
}

/// Check which interrupts are enabled for all [`TxEvent`]s
#[cfg(feature = "enumset")]
#[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
#[inline]
pub fn configured_tx_interrupts(usart: &impl Instance) -> EnumSet<TxEvent> {
    let mut tx_events = EnumSet::new();

    for event in EnumSet::<TxEvent>::all().iter() {
        if is_tx_interrupt_configured(usart, event) {
            tx_events |= event;
        }
    }

    tx_events
}

/// Check which interrupts are enabled for all [`RxEvent`]s
#[cfg(feature = "enumset")]
#[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
#[inline]
pub fn configured_rx_interrupts(usart: &impl Instance) -> EnumSet<RxEvent> {
    let mut rx_events = EnumSet::new();

    for event in EnumSet::<RxEvent>::all().iter() {
        if is_rx_interrupt_configured(usart, event) {
            rx_events |= event;
        }
    }

    rx_events
}

/// Get an [`EnumSet`] of all fired [`RxEvent`]s
#[cfg(feature = "enumset")]
#[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
pub fn triggered_rx_events(usart: &impl Instance) -> EnumSet<RxEvent> {
    let mut rx_events = EnumSet::new();

    for event in EnumSet::<RxEvent>::all().iter() {
        if is_rx_event_triggered(usart, event) {
            rx_events |= event;
        }
    }

    rx_events
}

/// Get an [`EnumSet`] of all fired [`TxEvent`]s
#[cfg(feature = "enumset")]
#[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
pub fn triggered_tx_events(usart: &impl Instance) -> EnumSet<TxEvent> {
    let mut tx_events = EnumSet::new();

    for event in EnumSet::<TxEvent>::all().iter() {
        if is_tx_event_triggered(usart, event) {
            tx_events |= event;
        }
    }

    tx_events
}

/// Serial error
///
/// As these are status events, they can be converted to [`Event`]s, via [`Into`].
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    ///
    /// This error is thrown by hardware when a de-synchronization, excessive noise or a break
    /// character is detected.
    Framing,
    /// Noise error
    ///
    /// This error is thrown by hardware when noise is detected on a received frame.
    Noise,
    /// RX buffer overrun
    ///
    /// # Cause
    ///
    /// An overrun error occurs when a character is received when RXNE has not been reset. Data can
    /// not be transferred from the shift register to the RDR register until the RXNE bit is
    /// cleared. The RXNE flag is set after every byte received. An overrun error occurs if RXNE
    /// flag is set when the next data is received or the previous DMA request has not been
    /// serviced.
    ///
    /// # Behavior
    ///
    /// - The RDR content will not be lost. The previous data is available when a read to USART_RDR
    ///   is performed.
    /// - The shift register will be overwritten. After that point, any data received
    ///   during overrun is lost
    Overrun,
    /// Parity check error
    ///
    /// This error is thrown by hardware when a parity error occurs in receiver mode.
    Parity,
}

impl From<Error> for RxEvent {
    fn from(error: Error) -> Self {
        match error {
            Error::Framing => RxEvent::FramingError,
            Error::Overrun => RxEvent::OverrunError,
            Error::Noise => RxEvent::NoiseError,
            Error::Parity => RxEvent::ParityError,
        }
    }
}

/// The error type returned when a [`Event`] to [`Error`] conversion failed.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TryFromEventError(pub(crate) ());

impl TryFrom<RxEvent> for Error {
    type Error = TryFromEventError;
    fn try_from(event: RxEvent) -> Result<Self, Self::Error> {
        Ok(match event {
            RxEvent::FramingError => Error::Framing,
            RxEvent::OverrunError => Error::Overrun,
            RxEvent::NoiseError => Error::Noise,
            RxEvent::ParityError => Error::Parity,
            _ => return Err(TryFromEventError(())),
        })
    }
}

/// An convinicnce enum for the most typical baud rates
#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[allow(missing_docs)]
pub enum BaudTable {
    Bd1200,
    Bd9600,
    Bd19200,
    Bd38400,
    Bd57600,
    Bd115200,
    Bd230400,
    Bd460800,
}

impl From<BaudTable> for Baud {
    fn from(baud: BaudTable) -> Self {
        match baud {
            BaudTable::Bd1200 => Baud(1_200),
            BaudTable::Bd9600 => Baud(9_600),
            BaudTable::Bd19200 => Baud(19_200),
            BaudTable::Bd38400 => Baud(38_400),
            BaudTable::Bd57600 => Baud(57_600),
            BaudTable::Bd115200 => Baud(115_200),
            BaudTable::Bd230400 => Baud(230_400),
            BaudTable::Bd460800 => Baud(460_800),
        }
    }
}

/// The error type returned when a [`Baud`] to [`BaudTable`] conversion failed.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TryFromBaudError(pub(crate) ());

impl TryFrom<Baud> for BaudTable {
    type Error = TryFromBaudError;
    fn try_from(baud: Baud) -> Result<Self, Self::Error> {
        Ok(match baud {
            Baud(1_200) => BaudTable::Bd1200,
            Baud(9_600) => BaudTable::Bd9600,
            Baud(19_200) => BaudTable::Bd19200,
            Baud(38_400) => BaudTable::Bd38400,
            Baud(57_600) => BaudTable::Bd57600,
            Baud(115_200) => BaudTable::Bd115200,
            Baud(230_400) => BaudTable::Bd230400,
            Baud(460_800) => BaudTable::Bd460800,
            _ => return Err(TryFromBaudError(())),
        })
    }
}

/// TX pin
pub trait TxPin<Usart>: crate::private::Sealed {}

/// RX pin
pub trait RxPin<Usart>: crate::private::Sealed {}

impl<Otype> TxPin<USART1> for gpioa::PA9<AF7<Otype>> {}
impl<Otype> TxPin<USART1> for gpiob::PB6<AF7<Otype>> {}
impl<Otype> TxPin<USART1> for gpioc::PC4<AF7<Otype>> {}
impl<Otype> RxPin<USART1> for gpioa::PA10<AF7<Otype>> {}
impl<Otype> RxPin<USART1> for gpiob::PB7<AF7<Otype>> {}
impl<Otype> RxPin<USART1> for gpioc::PC5<AF7<Otype>> {}

impl<Otype> TxPin<USART2> for gpioa::PA2<AF7<Otype>> {}
impl<Otype> TxPin<USART2> for gpiob::PB3<AF7<Otype>> {}
impl<Otype> RxPin<USART2> for gpioa::PA3<AF7<Otype>> {}
impl<Otype> RxPin<USART2> for gpiob::PB4<AF7<Otype>> {}

impl<Otype> TxPin<USART3> for gpiob::PB10<AF7<Otype>> {}
impl<Otype> TxPin<USART3> for gpioc::PC10<AF7<Otype>> {}
impl<Otype> RxPin<USART3> for gpioc::PC11<AF7<Otype>> {}

cfg_if! {
    if #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e", feature = "gpio-f373"))] {
        use crate::gpio::{gpiod, gpioe};

        impl<Otype> TxPin<USART1> for gpioe::PE0<AF7<Otype>> {}
        impl<Otype> RxPin<USART1> for gpioe::PE1<AF7<Otype>> {}

        impl<Otype> TxPin<USART2> for gpiod::PD5<AF7<Otype>> {}
        impl<Otype> RxPin<USART2> for gpiod::PD6<AF7<Otype>> {}

        impl<Otype> TxPin<USART3> for gpiod::PD8<AF7<Otype>> {}
        impl<Otype> RxPin<USART3> for gpiod::PD9<AF7<Otype>> {}
        impl<Otype> RxPin<USART3> for gpioe::PE15<AF7<Otype>> {}
    }
}

cfg_if! {
    if #[cfg(not(feature = "gpio-f373"))] {
        impl<Otype> TxPin<USART2> for gpioa::PA14<AF7<Otype>> {}
        impl<Otype> RxPin<USART2> for gpioa::PA15<AF7<Otype>> {}

        impl<Otype> RxPin<USART3> for gpiob::PB11<AF7<Otype>> {}
    }
}

cfg_if! {
    if #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e",))] {
        use crate::pac::{UART4, UART5};
        use crate::gpio::AF5;

        impl<Otype> TxPin<UART4> for gpioc::PC10<AF5<Otype>> {}
        impl<Otype> RxPin<UART4> for gpioc::PC11<AF5<Otype>> {}
        impl<Otype> TxPin<UART5> for gpioc::PC12<AF5<Otype>> {}
        impl<Otype> RxPin<UART5> for gpiod::PD2<AF5<Otype>> {}
    }
}

pub mod config;

/// Serial abstraction
///
/// This is an abstraction of the UART peripheral intended to be
/// used for standard duplex serial communication.
pub struct Serial<Usart, Pins> {
    usart: Usart,
    pins: Pins,
}

mod split {
    use super::*;
    /// Serial receiver
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Rx<Usart, Pin> {
        usart: Usart,
        pub(crate) pin: Pin,
    }

    /// Serial transmitter
    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Tx<Usart, Pin> {
        usart: Usart,
        pub(crate) pin: Pin,
    }

    impl<Usart, Pin> Tx<Usart, Pin>
    where
        Usart: Instance,
        Pin: super::TxPin<Usart>,
    {
        pub(crate) fn new(usart: Usart, pin: Pin) -> Self {
            Tx { usart, pin }
        }

        /// Destruct [`Tx`] to regain access to underlying USART and pin.
        pub(crate) fn free(self) -> (Usart, Pin) {
            (self.usart, self.pin)
        }
    }

    impl<Usart, Pin> Tx<Usart, Pin>
    where
        Usart: Instance,
    {
        /// Get a reference to internal usart peripheral
        ///
        /// # Safety
        ///
        /// This is unsafe, because the creation of this struct
        /// is only possible by splitting the the USART peripheral
        /// into Tx and Rx, which are internally both pointing
        /// to the same peripheral.
        ///
        /// Therefor, if getting a mutuable reference to the peripheral
        /// or changing any of it's configuration, the exclusivity
        /// is no longer guaranteed by the type system.
        ///
        /// Ensure that the Tx and Rx implemtation only do things with
        /// the peripheral, which do not effect the other.
        pub(crate) unsafe fn usart(&self) -> &Usart {
            &self.usart
        }

        /// Get a reference to internal usart peripheral
        ///
        /// # Saftey
        ///
        /// Same as in [`Self::usart()`].
        #[allow(dead_code)]
        pub(crate) unsafe fn usart_mut(&mut self) -> &mut Usart {
            &mut self.usart
        }

        /// Check if an interrupt event happend.
        #[inline]
        pub fn is_event_triggered(&self, event: TxEvent) -> bool {
            // Safety: Only reads TX specific events, should not be influenced
            // by RX half
            is_tx_event_triggered(unsafe { self.usart() }, event)
        }

        /// Enables or disables an interrupt for a given [`TxEvent`].
        #[inline]
        pub fn configure_interrupt(&mut self, event: TxEvent, enable: impl Into<crate::Toggle>) {
            // Safety: Only configures TX specific interrupts, which should not interfere
            // with the RX half
            configure_tx_interrupt(unsafe { self.usart_mut() }, event, enable)
        }

        /// Checks whether a transmitter interrupt is enabled.
        #[inline]
        pub fn is_interrupt_configured(&self, event: TxEvent) -> bool {
            // Safety: Only reads TX specific interrupts, should not be influenced
            // by RX half
            is_tx_interrupt_configured(unsafe { self.usart() }, event)
        }

        /// Clears the given interrupt.
        #[inline]
        pub fn clear_event(&mut self, event: TxEvent) {
            // Safety: Only clears TX specific interrupts, which should not interfere
            // with the RX half
            clear_tx_event(unsafe { self.usart_mut() }, event)
        }

        /// Check which interrupts are enabled for all [`TxEvent`]s
        #[cfg(feature = "enumset")]
        #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
        #[inline]
        pub fn configured_interrupts(&self) -> EnumSet<TxEvent> {
            // Safety: Only reads TX specific interrupts, should not be influenced
            // by RX half
            configured_tx_interrupts(unsafe { self.usart() })
        }

        /// Get a tuple of [`EnumSet`]s of all fired interrupt events.
        #[cfg(feature = "enumset")]
        #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
        pub fn triggered_events(&self) -> EnumSet<TxEvent> {
            triggered_tx_events(unsafe { self.usart() })
        }
    }

    impl<Usart, Pin> Rx<Usart, Pin>
    where
        Usart: Instance,
        Pin: super::RxPin<Usart>,
    {
        pub(crate) fn new(usart: Usart, pin: Pin) -> Self {
            Rx { usart, pin }
        }

        /// Destruct [`Rx`] to regain access to the underlying pin.
        ///
        /// The USART is omitted, as it is returnend from Tx already to avoid
        /// beeing able to crate a duplicate reference to the same underlying
        /// peripheral.
        pub(crate) fn free(self) -> Pin {
            self.pin
        }
    }

    impl<Usart, Pin> Rx<Usart, Pin>
    where
        Usart: Instance,
    {
        /// Get a reference to internal usart peripheral
        ///
        /// # Safety
        ///
        /// This is unsafe, because the creation of this struct
        /// is only possible by splitting the the USART peripheral
        /// into Tx and Rx, which are internally both pointing
        /// to the same peripheral.
        ///
        /// Therefor, if getting a mutuable reference to the peripheral
        /// or changing any of it's configuration, the exclusivity
        /// is no longer guaranteed by the type system.
        ///
        /// Ensure that the Tx and Rx implemtation only do things with
        /// the peripheral, which do not effect the other.
        pub(crate) unsafe fn usart(&self) -> &Usart {
            &self.usart
        }

        /// Get a reference to internal usart peripheral
        ///
        /// # Saftey
        ///
        /// Same as in [`Self::usart()`].
        pub(crate) unsafe fn usart_mut(&mut self) -> &mut Usart {
            &mut self.usart
        }

        /// Check if an interrupt event happend.
        #[inline]
        pub fn is_event_triggered(&self, event: RxEvent) -> bool {
            // Safety: Only reads RX specific events, should not be influenced
            // by TX half
            is_rx_event_triggered(unsafe { self.usart() }, event)
        }

        /// Enables or disables an interrupt for a given [`RxEvent`].
        #[inline]
        pub fn configure_interrupt(&mut self, event: RxEvent, enable: impl Into<crate::Toggle>) {
            // Safety: Only configures RX specific interrupts, which should not interfere
            // with the TX half
            configure_rx_interrupt(unsafe { self.usart_mut() }, event, enable)
        }

        /// Checks whether a transmitter interrupt is enabled.
        #[inline]
        pub fn is_interrupt_configured(&self, event: RxEvent) -> bool {
            // Safety: Only reads RX specific interrupts, should not be influenced
            // by TX half
            is_rx_interrupt_configured(unsafe { self.usart() }, event)
        }

        /// Clears the given interrupt.
        #[inline]
        pub fn clear_event(&mut self, event: RxEvent) {
            // Safety: Only clears RX specific interrupts, which should not interfere
            // with the TX half
            clear_rx_event(unsafe { self.usart_mut() }, event)
        }

        /// Check which interrupts are enabled for all [`RxEvent`]s
        #[cfg(feature = "enumset")]
        #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
        #[inline]
        pub fn configured_interrupts(&self) -> EnumSet<RxEvent> {
            // Safety: Only reads RX specific interrupts, should not be influenced
            // by TX half
            configured_rx_interrupts(unsafe { self.usart() })
        }

        /// Get a tuple of [`EnumSet`]s of all fired interrupt events.
        #[cfg(feature = "enumset")]
        #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
        pub fn triggered_events(&self) -> EnumSet<RxEvent> {
            triggered_rx_events(unsafe { self.usart() })
        }
    }
}

pub use split::{Rx, Tx};

impl<Usart, Tx, Rx> Serial<Usart, (Tx, Rx)>
where
    Usart: Instance,
{
    /// Configures a USART peripheral to provide serial communication
    ///
    /// # Panics
    ///
    /// Panics if the configured baud rate is impossible for the hardware to setup.
    pub fn new<Config>(
        usart: Usart,
        pins: (Tx, Rx),
        config: Config,
        clocks: Clocks,
        apb: &mut <Usart as rcc::RccBus>::Bus,
    ) -> Self
    where
        Usart: Instance,
        Tx: TxPin<Usart>,
        Rx: RxPin<Usart>,
        Config: Into<config::Config>,
    {
        use config::Parity;

        let config = config.into();

        // Enable USART peripheral for any further interaction.
        Usart::enable(apb);
        Usart::reset(apb);
        // Disable USART because some configuration bits could only be written
        // in this state.
        usart.cr1.modify(|_, w| w.ue().disabled());

        let brr = Usart::clock(&clocks).integer() / config.baudrate.integer();
        crate::assert!(brr >= 16, "impossible baud rate");
        usart.brr.write(|w| {
            w.brr().bits(
                // SAFETY: safe because of assert before
                unsafe { u16::try_from(brr).unwrap_unchecked() },
            )
        });

        // We currently support only eight data bits as supporting a full-blown
        // configuration gets complicated pretty fast. The USART counts data
        // and partiy bits together so the actual amount depends on the parity
        // selection.
        let (m0, ps, pce) = match config.parity {
            Parity::None => (M_A::Bit8, PS_A::Even, PCE_A::Disabled),
            Parity::Even => (M_A::Bit9, PS_A::Even, PCE_A::Enabled),
            Parity::Odd => (M_A::Bit9, PS_A::Odd, PCE_A::Enabled),
        };

        usart
            .cr2
            .modify(|_, w| w.stop().variant(config.stopbits.into()));
        usart.cr1.modify(|_, w| {
            w.ps().variant(ps); // set parity mode
            w.pce().variant(pce); // enable parity checking/generation
            w.m().variant(m0); // set data bits
            w.re().enabled(); // enable receiver
            w.te().enabled() // enable transmitter
        });

        // Finally enable the configured UART.
        usart.cr1.modify(|_, w| w.ue().enabled());

        Self { usart, pins }
    }

    /// Get access to the underlying register block.
    ///
    /// # Safety
    ///
    /// This function is not _memory_ unsafe per se, but does not guarantee
    /// anything about assumptions of invariants made in this implementation.
    ///
    /// Changing specific options can lead to un-expected behavior and nothing
    /// is guaranteed.
    pub unsafe fn peripheral(&mut self) -> &mut Usart {
        &mut self.usart
    }

    /// Releases the USART peripheral and associated pins
    pub fn free(self) -> (Usart, (Tx, Rx)) {
        self.usart
            .cr1
            .modify(|_, w| w.ue().disabled().re().disabled().te().disabled());
        (self.usart, self.pins)
    }
}

impl<Usart, Pins> Serial<Usart, Pins>
where
    Usart: Instance,
{
    /// Serial read out of the read register
    ///
    /// No error handling and no additional side-effects, besides the implied
    /// side-effects when reading out the RDR register.
    /// Handling errors has to be done manually. This can be done, by checking
    /// the triggered events via [`Serial::triggered_events`].
    ///
    /// Returns `None` if the hardware is busy.
    ///
    /// ## Embedded HAL
    ///
    /// To have a more managed way to read from the serial use the [`embedded_hal::serial::Read`]
    /// trait implementation.
    #[doc(alias = "RDR")]
    pub fn read_data_register(&self) -> Option<u8> {
        if self.usart.isr.read().busy().bit_is_set() {
            return None;
        }
        #[allow(clippy::cast_possible_truncation)]
        Some(self.usart.rdr.read().rdr().bits() as u8)
    }

    /// Check if the USART peripheral is busy.
    ///
    /// This can be useful to block on to synchronize between peripheral and CPU
    /// because of the asynchronous nature of the peripheral.
    pub fn is_busy(&mut self) -> bool {
        self.usart.isr.read().busy().bit_is_set()
    }

    /// Obtain the associated interrupt number for the serial peripheral.
    ///
    /// Used to unmask / enable the interrupt with [`cortex_m::peripheral::NVIC::unmask()`].
    /// This is useful for all `cortex_m::peripheral::INTERRUPT` functions.
    ///
    /// # Note
    ///
    /// This is the easier alternative to obatain the interrupt for:
    ///
    /// ```
    /// use cortex_m::peripheral::INTERRUPT;
    /// use stm32f3xx_hal::pac::USART1;
    /// use stm32f3xx_hal::interrupt::InterruptNumber;
    ///
    /// const INTERRUPT: Interrupt = <USART1 as InterruptNumber>::INTERRUPT;
    /// ```
    ///
    /// though this function can not be used in a const context.
    #[doc(alias = "unmask")]
    pub fn interrupt(&self) -> <Usart as crate::interrupts::InterruptNumber>::Interrupt {
        <Usart as crate::interrupts::InterruptNumber>::INTERRUPT
    }

    /// Enable the interrupt for the specified [`Event`].
    #[inline]
    pub fn enable_interrupt(&mut self, event: Event) {
        self.configure_interrupt(event, Switch::On);
    }

    /// Disable the interrupt for the specified [`Event`].
    #[inline]
    pub fn disable_interrupt(&mut self, event: Event) {
        self.configure_interrupt(event, Switch::Off);
    }

    /// Enable or disable the interrupt for the specified [`Event`].
    #[inline]
    pub fn configure_interrupt(&mut self, event: Event, enable: impl Into<Toggle>) {
        configure_interrupt(&mut self.usart, event, enable)
    }

    /// Enable or disable the interrupt for the specified [`TxEvent`].
    #[inline]
    pub fn configure_tx_interrupt(&mut self, event: TxEvent, enable: impl Into<Toggle>) {
        configure_tx_interrupt(&mut self.usart, event, enable)
    }

    /// Enable or disable the interrupt for the specified [`RxEvent`].
    #[inline]
    pub fn configure_rx_interrupt(&mut self, event: RxEvent, enable: impl Into<Toggle>) {
        configure_rx_interrupt(&mut self.usart, event, enable)
    }

    /// Enable or disable interrupt for the specified [`TxEvent`]s.
    ///
    /// Like [`Serial::configure_interrupt`], but instead using an enumset. The corresponding
    /// interrupt for every [`TxEvent`] in the set will be enabled, every other interrupt will be
    /// **disabled**.
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn configure_tx_interrupts(&mut self, events: EnumSet<TxEvent>) {
        configure_tx_interrupts(&mut self.usart, events)
    }

    /// Enable or disable interrupt for the specified [`RxEvent`]s.
    ///
    /// Like [`Serial::configure_interrupt`], but instead using an enumset. The corresponding
    /// interrupt for every [`RxEvent`] in the set will be enabled, every other interrupt will be
    /// **disabled**.
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn configure_rx_interrupts(&mut self, events: EnumSet<RxEvent>) {
        configure_rx_interrupts(&mut self.usart, events)
    }

    /// Check whether a transmitter interrupt was enabled.
    #[inline]
    pub fn is_tx_interrupt_configured(&self, event: TxEvent) -> bool {
        is_tx_interrupt_configured(&self.usart, event)
    }

    /// Check whether a receiver interrupt was enabled.
    #[inline]
    pub fn is_rx_interrupt_configured(&self, event: RxEvent) -> bool {
        is_rx_interrupt_configured(&self.usart, event)
    }

    /// Check if an interrupt is configured for the [`Event`]
    #[inline]
    pub fn is_interrupt_configured(&self, event: Event) -> bool {
        is_interrupt_configured(&self.usart, event)
    }

    /// Check which interrupts are enabled for all [`Event`]s
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    #[inline]
    pub fn configured_interrupts(&mut self) -> (EnumSet<TxEvent>, EnumSet<RxEvent>) {
        (
            self.configured_tx_interrupts(),
            self.configured_rx_interrupts(),
        )
    }

    /// Check which interrupts are enabled for all [`TxEvent`]s
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    #[inline]
    pub fn configured_tx_interrupts(&self) -> EnumSet<TxEvent> {
        configured_tx_interrupts(&self.usart)
    }

    /// Check which interrupts are enabled for all [`RxEvent`]s
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    #[inline]
    pub fn configured_rx_interrupts(&mut self) -> EnumSet<RxEvent> {
        configured_rx_interrupts(&self.usart)
    }

    /// Check if an interrupt event happend.
    #[inline]
    pub fn is_event_triggered(&self, event: Event) -> bool {
        is_event_triggered(&self.usart, event)
    }

    /// Check if a reception (RX) interrupt event was triggered.
    #[inline]
    pub fn is_rx_event_triggered(&self, event: RxEvent) -> bool {
        is_rx_event_triggered(&self.usart, event)
    }

    /// Check if a transmission (TX) interrupt event was triggered.
    #[inline]
    pub fn is_tx_event_triggered(&self, event: TxEvent) -> bool {
        is_tx_event_triggered(&self.usart, event)
    }

    /// Get a tuple of [`EnumSet`]s of all fired interrupt events.
    ///
    /// # Examples
    ///
    /// This allows disabling all fired event at once, via the enum set abstraction, like so
    ///
    /// ```rust
    /// let (tx_events, rx_events) = serial.triggered_events();
    /// for event in tx_events {
    ///     serial.listen(event, false);
    /// }
    /// for event in rx_events {
    ///     serial.listen(event, false);
    /// }
    /// ```
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn triggered_events(&self) -> (EnumSet<TxEvent>, EnumSet<RxEvent>) {
        (self.triggered_tx_events(), self.triggered_rx_events())
    }

    /// Get an [`EnumSet`] of all fired [`RxEvent`]s
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn triggered_rx_events(&self) -> EnumSet<RxEvent> {
        triggered_rx_events(&self.usart)
    }

    /// Get an [`EnumSet`] of all fired [`TxEvent`]s
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn triggered_tx_events(&self) -> EnumSet<TxEvent> {
        triggered_tx_events(&self.usart)
    }

    /// Clear an event.
    #[inline]
    pub fn clear_event(&mut self, event: Event) {
        match event {
            Event::Tx(tx_event) => self.clear_tx_event(tx_event),
            Event::Rx(rx_event) => self.clear_rx_event(rx_event),
        }
    }
    /// Clear a transmitter event.
    #[inline]
    pub fn clear_tx_event(&mut self, event: TxEvent) {
        clear_tx_event(&mut self.usart, event)
    }

    /// Clear a receiver event.
    pub fn clear_rx_event(&mut self, event: RxEvent) {
        clear_rx_event(&mut self.usart, event)
    }

    /// Clear **all** interrupt events.
    #[inline]
    pub fn clear_events(&mut self) {
        // SAFETY: This atomic write clears all flags and ignores the reserverd bit fields.
        self.usart.icr.write(|w| unsafe { w.bits(u32::MAX) });
    }

    /// Enable or disable overrun detection
    ///
    /// When overrun detection is disabled and new data is received while the
    /// [`RxEvent::ReceiveDataRegisterNotEmpty`] flag is still set,
    /// the [`RxEvent::OverrunError`] flag is not set and the new received data overwrites the
    /// previous content of the RDR register.
    #[doc(alias = "OVRDIS")]
    #[inline]
    pub fn detect_overrun(&mut self, enable: bool) {
        let uart_enabled = self.usart.cr1.read().ue().bit();
        self.usart.cr1.modify(|_, w| w.ue().disabled());
        self.usart.cr3.modify(|_, w| w.ovrdis().bit(!enable));
        self.usart.cr1.modify(|_, w| w.ue().bit(uart_enabled));
    }

    /// Configuring the UART to match each received character,
    /// with the configured one.
    ///
    /// If the character is matched [`Event::CharacterMatch`] is generated,
    /// which can fire an interrupt, if enabled via [`Serial::configure_interrupt()`]
    pub fn set_match_character(&mut self, char: u8) {
        // Note: This bit field can only be written when reception is disabled (RE = 0) or the
        // USART is disabled
        let enabled = self.usart.cr1.read().ue().bit_is_set();
        self.usart.cr1.modify(|_, w| w.ue().disabled());
        self.usart.cr2.modify(|_, w| w.add().bits(char));
        self.usart.cr1.modify(|_, w| w.ue().bit(enabled));
    }

    /// Read out the configured match character.
    pub fn match_character(&self) -> u8 {
        self.usart.cr2.read().add().bits()
    }
}

impl<Usart, Tx, Rx> Serial<Usart, (Tx, Rx)>
where
    Usart: Instance + ReceiverTimeoutExt,
{
    /// Set the receiver timeout value.
    ///
    /// The RTOF flag ([`RxEvent::ReceiverTimeout`]) is set if, after the last received character,
    /// no new start bit is detected for more than the receiver timeout value, where the value
    /// is being a counter, which is decreased by the configured baud rate.
    ///
    /// A simple calculation might be `time_per_counter_value = 1 / configured_baud_rate`
    ///
    ///
    /// ## Note
    ///
    /// - If the value is None, the receiver timeout feature is disabled.
    /// - This value must only be programmed once per received character.
    /// - Can be written on the fly. If the new value is lower than or equal to the counter,
    ///   the RTOF flag is set.
    /// - Values higher than 24 bits are truncated to 24 bit max (`16_777_216`).
    pub fn set_receiver_timeout(&mut self, value: Option<u32>) {
        if let Some(value) = value {
            self.usart.cr2.modify(|_, w| w.rtoen().enabled());
            self.usart.rtor.modify(|_, w| w.rto().bits(value));
        } else {
            self.usart.cr2.modify(|_, w| w.rtoen().disabled());
        }
    }

    /// Read out the currently set timeout value
    ///
    /// The relationship between the unit value and time is described in
    /// [`Serial::receiver_timeout`].
    ///
    /// - If the value is None, the receiver timeout feature is disabled.
    pub fn receiver_timeout(&self) -> Option<u32> {
        if self.usart.cr2.read().rtoen().is_enabled() {
            Some(self.usart.rtor.read().rto().bits())
        } else {
            None
        }
    }
}

/// Implementation of the [`embedded_hal::serial::Read`] trait
/// shared between [`Rx::read()`] and [`Serial::read()`]
fn eh_read<Usart>(usart: &mut Usart) -> nb::Result<u8, Error>
where
    Usart: Instance,
{
    let isr = usart.isr.read();

    Err(if isr.pe().bit_is_set() {
        usart.icr.write(|w| w.pecf().clear());
        nb::Error::Other(Error::Parity)
    } else if isr.fe().bit_is_set() {
        usart.icr.write(|w| w.fecf().clear());
        nb::Error::Other(Error::Framing)
    } else if isr.nf().bit_is_set() {
        usart.icr.write(|w| w.ncf().clear());
        nb::Error::Other(Error::Noise)
    } else if isr.ore().bit_is_set() {
        usart.icr.write(|w| w.orecf().clear());
        // Flush the receive data
        //
        // Imagine a case of an overrun, where 2 or more bytes have been received by the hardware
        // but haven't been read out yet: An overrun is signaled!
        //
        // The current state is: One byte is in the RDR (read data register) one one byte is still
        // in the hardware pipeline (shift register).
        //
        // With this implementation, the overrun flag would be cleared but the data would not be
        // read out, so there are still to bytes waiting in the pipeline.
        //
        // In case the flush wasn't called: The next read would then be successful, as the RDR is
        // cleared, but the read after that would again report an overrun error, because the byte
        // still in the hardware shift register would signal it.
        //
        // This means, that the overrun error is not completely handled by this read()
        // implementation and leads to surprising behavior, if one would explicitly check for
        // Error::Overrun and think, that this error would than be handled, which would not be the
        // case.
        //
        // This is because, with this function signature, the data can not be returned
        // simultainously with the error.
        //
        // To mitigate this and have an implementation without these surprises flush the RDR
        // register. This leads to loosing a theoretically still receivable data byte! But at least
        // no cleanup is needed, after an overrun is called.
        usart.rqr.write(|w| w.rxfrq().set_bit());
        nb::Error::Other(Error::Overrun)
    } else if isr.rxne().bit_is_set() {
        #[allow(clippy::cast_possible_truncation)]
        return Ok(usart.rdr.read().bits() as u8);
    } else {
        nb::Error::WouldBlock
    })
}

// TODO: Check if u16 for WORD is feasiable / possible
impl<Usart, Tx, Rx> serial::Read<u8> for Serial<Usart, (Tx, Rx)>
where
    Usart: Instance,
{
    type Error = Error;

    /// Getting back an error means that the error is defined as "handled":
    ///
    /// This implementation has the side effect for error handling, that the [`Event`] flag of the returned
    /// [`Error`] is cleared.
    ///
    /// This might be a problem, because if an interrupt is enabled for this particular flag, the
    /// interrupt handler might not have the chance to find out from which flag the interrupt
    /// originated.
    ///
    /// So this function is only intended to be used for direct error handling and not leaving it
    /// up to the interrupt handler.
    ///
    /// To read out the content of the read register without internal error handling, use
    /// [`embedded_hal::serial::Read`].
    /// ...
    // -> According to this API it should be skipped.
    fn read(&mut self) -> nb::Result<u8, Error> {
        eh_read(&mut self.usart)
    }
}

impl<Usart, Pins> serial::Write<u8> for Serial<Usart, Pins>
where
    Usart: Instance,
{
    // NOTE(Infallible) See section "29.7 USART interrupts"; the only possible errors during
    // transmission are: clear to send (which is disabled in this case) errors and
    // framing errors (which only occur in SmartCard mode); neither of these apply to
    // our hardware configuration
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Infallible> {
        if self.usart.isr.read().tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
        if self.usart.isr.read().txe().bit_is_set() {
            self.usart.tdr.write(|w| w.tdr().bits(u16::from(byte)));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<Usart, Pins> fmt::Write for Serial<Usart, Pins>
where
    Serial<Usart, Pins>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| nb::block!(self.write(c)))
            .map_err(|_| fmt::Error)
    }
}

impl<USART, TX, RX> blocking::serial::write::Default<u8> for Serial<USART, (TX, RX)> where
    USART: Instance
{
}

impl<Usart, Pin> serial::Write<u8> for Tx<Usart, Pin>
where
    Usart: Instance,
    Pin: TxPin<Usart>,
{
    // NOTE(Infallible) See section "29.7 USART interrupts"; the only possible errors during
    // transmission are: clear to send (which is disabled in this case) errors and
    // framing errors (which only occur in SmartCard mode); neither of these apply to
    // our hardware configuration
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Infallible> {
        let isr = unsafe { self.usart().isr.read() };

        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { self.usart().isr.read() };

        if isr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            unsafe { self.usart().tdr.write(|w| w.tdr().bits(u16::from(byte))) };
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<Usart, Pin> fmt::Write for Tx<Usart, Pin>
where
    Tx<Usart, Pin>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| nb::block!(self.write(c)))
            .map_err(|_| fmt::Error)
    }
}

impl<Usart, Pin> Rx<Usart, Pin>
where
    Usart: Instance + Dma,
{
    /// Fill the buffer with received data using DMA.
    pub fn read_exact<B, C>(self, mut buffer: B, mut channel: C) -> SerialDmaRx<B, C, Self>
    where
        Self: dma::OnChannel<C>,
        B: dma::WriteBuffer<Word = u8> + 'static,
        C: dma::Channel,
    {
        // NOTE(unsafe) usage of a valid peripheral address
        unsafe {
            channel.set_peripheral_address(
                &self.usart().rdr as *const _ as u32,
                dma::Increment::Disable,
            )
        };

        // Safety: It is okay to call [`write_buffer'] multiple times as specified
        // in its safety note.
        let transfer_len = unsafe { buffer.write_buffer().1 } as u16;
        SerialDmaRx {
            transfer: dma::Transfer::start_write(buffer, channel, self),
            transfer_len,
        }
    }
}

impl<Usart, Pin> blocking::serial::write::Default<u8> for Tx<Usart, Pin>
where
    Usart: Instance,
    Pin: TxPin<Usart>,
{
}

/// Thin wrapper struct over the DMA transfer struct.
///
/// This wrapper mostly exposes the [`dma::Transfer`] API but also also exposes
/// an API to check for other USART ISR events during on-going transfers.
pub struct SerialDmaRx<B: dma::WriteBuffer<Word = u8> + 'static, C: dma::Channel, T: dma::Target> {
    transfer: dma::Transfer<B, C, T>,
    transfer_len: u16,
}

impl<B, C, T> SerialDmaRx<B, C, T>
where
    B: dma::WriteBuffer<Word = u8>,
    C: dma::Channel,
    T: dma::Target,
{
    /// Call [`dma::Transfer::stop`].
    pub fn stop(self) -> (B, C, T) {
        self.transfer.stop()
    }

    /// Call [`dma::Transfer::is_complete`].
    pub fn is_complete(&self) -> bool {
        self.transfer.is_complete()
    }

    /// Call [`dma::Transfer::wait`].
    pub fn wait(self) -> (B, C, T) {
        self.transfer.wait()
    }

    /// Returns the current transfer length
    pub fn transfer_len(&self) -> u16 {
        self.transfer_len
    }

    /// Check whether a DMA event was triggered.
    pub fn is_dma_event_triggered(&self, event: dma::Event) -> bool {
        self.transfer.is_event_triggered(event)
    }

    /// Stop the DMA transfer while also returning the number of received bytes.
    /// For a completed RX transfer, this should be the full transfer length.
    /// For incomplete transfers, this can be used to retrieve the number of
    /// received bytes.
    pub fn stop_and_return_received_bytes(self) -> (B, C, T, u16) {
        let transfer_len = self.transfer_len;
        // Stop the DMA before reading the remaining transfer length because otherwise,
        // the hardware might arbitrarily decrement the value.
        let (b, c, t) = self.stop();
        // The DMA API and the decrement logic used by the hardware
        // enforces that the remaining transfer length is always equal or
        // smaller than the specified transfer length.
        let remaining_transfer_len = c.get_remaining_transfer_len();
        (b, c, t, transfer_len - remaining_transfer_len)
    }
}

impl<B, C, Usart, Pin> SerialDmaRx<B, C, Rx<Usart, Pin>>
where
    B: dma::WriteBuffer<Word = u8>,
    C: dma::Channel,
    Usart: Instance + Dma,
    Pin: RxPin<Usart>,
{
    /// Check if an interrupt event happened.
    pub fn is_event_triggered(&self, event: RxEvent) -> bool {
        self.transfer.target().is_event_triggered(event)
    }
}

impl<B, C, Usart, Pins> SerialDmaRx<B, C, Serial<Usart, Pins>>
where
    B: dma::WriteBuffer<Word = u8>,
    C: dma::Channel,
    Usart: Instance + Dma,
{
    /// Check if an interrupt event happened.
    pub fn is_event_triggered(&self, event: Event) -> bool {
        self.transfer.target().is_event_triggered(event)
    }
}

/// Thin wrapper struct over the DMA transfer struct.
///
/// This wrapper mostly exposes the [`dma::Transfer`] API but also implements
/// some additional checks because the conditions for DMA transfer completion
/// require that the USART TC ISR flag is set as well. It also exposes an API
/// to check for other USART ISR events during ongoing transfers.
pub struct SerialDmaTx<B: dma::ReadBuffer<Word = u8> + 'static, C: dma::Channel, T: dma::Target> {
    transfer: dma::Transfer<B, C, T>,
}

impl<B, C, T> SerialDmaTx<B, C, T>
where
    B: dma::ReadBuffer<Word = u8>,
    C: dma::Channel,
    T: dma::Target,
{
    /// Calls [`dma::Transfer::stop`].
    pub fn stop(self) -> (B, C, T) {
        self.transfer.stop()
    }
}

impl<B, C, Usart, Pin> SerialDmaTx<B, C, Tx<Usart, Pin>>
where
    Usart: Instance + Dma,
    C: dma::Channel,
    B: dma::ReadBuffer<Word = u8>,
    Pin: TxPin<Usart>,
{
    /// Wrapper function which can be used to check transfer completion.
    /// In addition to checking the transfer completion of the DMA, it also
    /// checks that the USART Transmission Complete flag was set by the hardware.
    /// This is required to avoid corrupting the last transmission before
    /// disabling the USART or entering stop mode.
    pub fn is_complete(&self) -> bool {
        let target = self.transfer.target();
        self.transfer.is_complete() && target.is_event_triggered(TxEvent::TransmissionComplete)
    }

    /// Block until the transfer is complete. This function also uses
    /// [`SerialDmaTx::is_complete`] to check that the USART TC flag was set by
    /// the hardware.
    pub fn wait(self) -> (B, C, Tx<Usart, Pin>) {
        while !self.is_complete() {}
        self.stop()
    }

    /// Check whether a DMA event was triggered.
    pub fn is_dma_event_triggered(&self, event: dma::Event) -> bool {
        self.transfer.is_event_triggered(event)
    }

    /// Check if an interrupt event happened.
    pub fn is_event_triggered(&self, event: TxEvent) -> bool {
        self.transfer.target().is_event_triggered(event)
    }
}

impl<B, C, Usart, Pins> SerialDmaTx<B, C, Serial<Usart, Pins>>
where
    Usart: Instance + Dma,
    C: dma::Channel,
    B: dma::ReadBuffer<Word = u8>,
{
    /// Wrapper function which can be used to check transfer completion.
    /// In addition to checking the transfer completion of the DMA, it also
    /// checks that the Transmission Complete flag was set by the hardware.
    /// This is required to avoid corrupting the last transmission before
    /// disabling the USART or entering stop mode.
    pub fn is_complete(&self) -> bool {
        let target = self.transfer.target();
        self.transfer.is_complete() && target.is_tx_event_triggered(TxEvent::TransmissionComplete)
    }

    /// Block until the transfer is complete. This function also uses
    /// [`SerialDmaTx::is_complete`] to check that the USART TC flag was set by
    /// the hardware.
    pub fn wait(self) -> (B, C, Serial<Usart, Pins>) {
        while !self.is_complete() {}
        self.stop()
    }
}

impl<Usart, Pin> Tx<Usart, Pin>
where
    Usart: Instance + Dma,
    Pin: TxPin<Usart>,
{
    /// Transmit all data in the buffer using DMA.
    pub fn write_all<B, C>(self, buffer: B, mut channel: C) -> SerialDmaTx<B, C, Self>
    where
        Self: dma::OnChannel<C>,
        B: dma::ReadBuffer<Word = u8> + 'static,
        C: dma::Channel,
    {
        // NOTE(unsafe) usage of a valid peripheral address
        unsafe {
            channel.set_peripheral_address(
                &self.usart().tdr as *const _ as u32,
                dma::Increment::Disable,
            )
        };

        SerialDmaTx {
            transfer: dma::Transfer::start_read(buffer, channel, self),
        }
    }
}

impl<Usart, Pin> dma::Target for Rx<Usart, Pin>
where
    Usart: Instance + Dma,
{
    fn enable_dma(&mut self) {
        // NOTE(unsafe) critical section prevents races
        cortex_m::interrupt::free(|_| unsafe {
            self.usart().cr3.modify(|_, w| w.dmar().enabled());
        });
    }

    fn disable_dma(&mut self) {
        // NOTE(unsafe) critical section prevents races
        cortex_m::interrupt::free(|_| unsafe {
            self.usart().cr3.modify(|_, w| w.dmar().disabled());
        });
    }
}

impl<Usart, Pin> dma::Target for Tx<Usart, Pin>
where
    Usart: Instance + Dma,
    Pin: TxPin<Usart>,
{
    fn enable_dma(&mut self) {
        // NOTE(unsafe) critical section prevents races
        cortex_m::interrupt::free(|_| unsafe {
            self.usart().cr3.modify(|_, w| w.dmat().enabled());
        });
    }

    fn disable_dma(&mut self) {
        // NOTE(unsafe) critical section prevents races
        cortex_m::interrupt::free(|_| unsafe {
            self.usart().cr3.modify(|_, w| w.dmat().disabled());
        });
    }
}

impl<Usart, Pins> Serial<Usart, Pins>
where
    Usart: Instance + Dma,
{
    /// Fill the buffer with received data using DMA.
    pub fn read_exact<B, C>(self, mut buffer: B, mut channel: C) -> SerialDmaRx<B, C, Self>
    where
        Self: dma::OnChannel<C>,
        B: dma::WriteBuffer<Word = u8> + 'static,
        C: dma::Channel,
    {
        // SAFETY: RDR is valid peripheral address, safe to dereference and pass to the DMA
        unsafe {
            channel.set_peripheral_address(
                core::ptr::addr_of!(self.usart.rdr) as u32,
                dma::Increment::Disable,
            );
        };

        // Safety: It is okay to call [`write_buffer'] multiple times as specified
        // in its safety note.
        let transfer_len = unsafe { buffer.write_buffer().1 } as u16;
        SerialDmaRx {
            transfer: dma::Transfer::start_write(buffer, channel, self),
            transfer_len,
        }
    }

    /// Transmit all data in the buffer using DMA.
    pub fn write_all<B, C>(self, buffer: B, mut channel: C) -> SerialDmaTx<B, C, Self>
    where
        Self: dma::OnChannel<C>,
        B: dma::ReadBuffer<Word = u8> + 'static,
        C: dma::Channel,
    {
        // SAFETY: TDR is valid peripheral address, safe to dereference and pass to the DMA
        unsafe {
            channel.set_peripheral_address(
                core::ptr::addr_of!(self.usart.tdr) as u32,
                dma::Increment::Disable,
            );
        };

        SerialDmaTx {
            transfer: dma::Transfer::start_read(buffer, channel, self),
        }
    }
}

impl<Usart, Pins> dma::Target for Serial<Usart, Pins>
where
    Usart: Instance + Dma,
{
    fn enable_dma(&mut self) {
        self.usart
            .cr3
            .modify(|_, w| w.dmar().enabled().dmat().enabled());
    }

    fn disable_dma(&mut self) {
        self.usart
            .cr3
            .modify(|_, w| w.dmar().disabled().dmat().disabled());
    }
}

/// Marker trait for DMA capable UART implementations.
pub trait Dma: crate::private::Sealed {}

impl Dma for USART1 {}
impl Dma for USART2 {}
impl Dma for USART3 {}

/// Marker trait for Receiver Timeout capable UART implementations.
pub trait ReceiverTimeoutExt: crate::private::Sealed {}

impl ReceiverTimeoutExt for USART1 {}
#[cfg(not(any(feature = "gpio-f333")))]
impl ReceiverTimeoutExt for USART2 {}
#[cfg(not(any(feature = "gpio-f333")))]
impl ReceiverTimeoutExt for USART3 {}

/// UART instance
pub trait Instance:
    Deref<Target = RegisterBlock>
    + crate::interrupts::InterruptNumber
    + crate::private::Sealed
    + rcc::Enable
    + rcc::Reset
{
    #[doc(hidden)]
    fn clock(clocks: &Clocks) -> Hertz;
}

macro_rules! usart {
    (
        $(
            $USARTX:ident: ($INTERRUPT:path),
        )+
    ) => {
        $(
            impl crate::interrupts::InterruptNumber for $USARTX {
                type Interrupt = Interrupt;
                const INTERRUPT: Interrupt = $INTERRUPT;
            }

            #[cfg(feature = "defmt")]
            impl<Pins> defmt::Format for Serial<$USARTX, Pins> {
                fn format(&self, f: defmt::Formatter) {
                    // Omitting pins makes it:
                    // 1. Easier.
                    // 2. Not to specialized to use it ergonimically for users
                    //    even in a generic context.
                    // 3. Not require specialization.
                    defmt::write!(
                        f,
                        "Serial {{ usart: {}, pins: ? }}",
                        stringify!($USARTX),
                    );
                }
            }

            impl<Pins> fmt::Debug for Serial<$USARTX, Pins> {
                fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                    f.debug_struct(stringify!(Serial))
                        .field("usart", &stringify!($USARTX))
                        .field("pins", &"?")
                        .finish()
                }
            }
        )+
    };

    ([ $(($X:literal, $INTERRUPT:path)),+ ]) => {
        paste::paste! {
            usart!(
                $(
                    [<USART $X>]: ($INTERRUPT),
                )+
            );
        }
    };
}

/// Generates a clock function for UART Peripherals, where
/// the only clock source can be the peripheral clock
#[allow(unused_macros)]
macro_rules! usart_static_clock {
    ($($USARTX:ident),+) => {
        $(
            impl Instance for $USARTX {
                fn clock(clocks: &Clocks) -> Hertz {
                    <$USARTX as rcc::BusClock>::clock(clocks)
                }
            }
        )+
    };
    ($($X:literal),+) => {
        paste::paste! {
            usart_static_clock!(
                $([<USART $X>]),+
            );
        }
    };
}

/// Generates a clock function for UART Peripherals, where
/// the clock source can vary.
macro_rules! usart_var_clock {
    ($($USARTX:ident, $usartXsw:ident),+) => {
        $(
            impl Instance for $USARTX {
                fn clock(clocks: &Clocks) -> Hertz {
                    // SAFETY: The read instruction of the RCC.cfgr3 register should be atomic
                    match unsafe {(*RCC::ptr()).cfgr3.read().$usartXsw().variant()} {
                        USART1SW_A::Pclk => <$USARTX as rcc::BusClock>::clock(clocks),
                        USART1SW_A::Hsi => crate::rcc::HSI,
                        USART1SW_A::Sysclk => clocks.sysclk(),
                        USART1SW_A::Lse => crate::rcc::LSE,
                    }
                }
            }
        )+
    };
    ($($X:literal),+) => {
        paste::paste! {
            usart_var_clock!(
                $([<USART $X>], [<usart $X sw>]),+
            );
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "stm32f301x6",
        feature = "stm32f301x8",
        feature = "stm32f318x8",
        feature = "stm32f302x6",
        feature = "stm32f302x8",
        feature = "stm32f303x6",
        feature = "stm32f303x8",
        feature = "stm32f328x8",
        feature = "stm32f334x4",
        feature = "stm32f334x6",
        feature = "stm32f334x8",
    ))] {
        // USART1 is accessed through APB2,
        // but USART1SW_A::PCLK will connect its phy to PCLK1.
        usart_var_clock!(1);
        // These are uart peripherals, where the only clock source
        // is the PCLK (peripheral clock).
        usart_static_clock!(2, 3);
    } else {
        usart_var_clock!(1, 2, 3);
    }
}

#[cfg(not(feature = "svd-f373"))]
usart!([
    (1, Interrupt::USART1_EXTI25),
    (2, Interrupt::USART2_EXTI26),
    (3, Interrupt::USART3_EXTI28)
]);
#[cfg(feature = "svd-f373")]
usart!([
    (1, Interrupt::USART1),
    (2, Interrupt::USART2),
    (3, Interrupt::USART3)
]);

cfg_if::cfg_if! {
    // See table 29.4 RM0316
    if #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e"))] {

        macro_rules! uart {
            ([ $(($X:literal, $INTERRUPT:path)),+ ]) => {
                paste::paste! {
                    usart!(
                        $(
                            [<UART $X>]: ($INTERRUPT),
                        )+
                    );
                }
            };
        }

        macro_rules! uart_var_clock {
            ($($X:literal),+) => {
                paste::paste! {
                    usart_var_clock!(
                        $([<UART $X>], [<uart $X sw>]),+
                    );
                }
            };
        }

        uart_var_clock!(4, 5);
        uart!([(4, Interrupt::UART4_EXTI34), (5, Interrupt::UART5_EXTI35)]);

        impl Dma for UART4 {}

        impl ReceiverTimeoutExt for UART4 {}
        impl ReceiverTimeoutExt for UART5 {}
    }
}
