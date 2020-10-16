//! # nRF52 802.15.4 Radio
//!
//! Adhering to the IEEE 802.15.4-2006 standard.
//!
//! ## Interframe spacing
//!
//! Inter-frame spacing in 802.15.4 depends on frames type and the payload
//! length. Short frames has a short interframe spacing (SIFS) frames longer
//! than 18 octets shall have a long interframe spacing (LIFS). Acknowledge
//! frames shall have a special acknowledge interframe spacing (AIFS).
//!
//! * SIFS is aMinSIFSPeriod symbols which is 12 for our PHY
//! * LIFS is aMinLIFSPeriod symbols which is 40 for our PHY
//! * AIFS is aTurnaroundTime to aTurnaroundTime + aUnitBackoffPeriod symbols which is
//!   12 to 12 + 20 → 32 symbols
//!
//! The symbol rate for O-QPSK is,
//! > 62.5 ksymbol/s when operating in the 2.4 GHz band
//!
//! Each symbol is 16 μs.
//!
//! * SIFS: 12 × 16 μs → 192 μs
//! * LIFS: 40 × 16 μs → 640 μs
//! * AIFS: 32 × 16 μs → 612 μs
//!

use core::sync::atomic::{compiler_fence, Ordering};

use crate::pac::{generic::Variant, radio, RADIO};

/// RX-TX turn-around time in symbols
const TURNAROUND_TIME_SYMBOLS: u32 = 12;

/// Back-off time period in symbols
const BACKOFF_PERIOD_SYMBOLS: u32 = 20;

/// Number of short interframe spacing (SIFS) symbols
const SIFS_SYMBOLS: u32 = 12;
/// Number of acknowledge interframe spacing (AIFS) symbols
const AIFS_SYMBOLS: u32 = TURNAROUND_TIME_SYMBOLS;
/// Number of long interframe spacing (LIFS) symbols
const LIFS_SYMBOLS: u32 = 40;

/// Microseconds (μs) per symbol
const MICROSECONDS_PER_SYMBOL: u32 = 16;

/// Backoff period in microseconds
const BACKOFF_PERIOD_MICROSECONDS: u32 = MICROSECONDS_PER_SYMBOL * BACKOFF_PERIOD_SYMBOLS;
/// Acknowledge interframe spacing (AIFS) in microseconds
const AIFS_MICROSECONDS: u32 = MICROSECONDS_PER_SYMBOL * AIFS_SYMBOLS;
/// Short interframe spacing (SIFS) in microseconds
const SIFS_MICROSECONDS: u32 = MICROSECONDS_PER_SYMBOL * SIFS_SYMBOLS;
/// Long interframe spacing (LIFS) in microseconds
const LIFS_MICROSECONDS: u32 = MICROSECONDS_PER_SYMBOL * LIFS_SYMBOLS;

/// Maximum length of a 802.15.4 package
const MAX_PACKET_LENGHT_REG: u8 = 129;

/// Max packet length, 127 bytes according to the standard
/// Here the length byte and LQI byte is added
pub const MAX_PACKET_LENGHT: usize = MAX_PACKET_LENGHT_REG as usize;

const CRC_POLYNOMIAL: u32 = 0x0001_1021;
const CCA_ED_THRESHOLD_DEFAULT: u8 = 20;
const CCA_CORR_THRESHOLD_DEFAULT: u8 = 20;
const CCA_CORR_LIMIT_DEFAULT: u8 = 2;
const SFD_DEFAULT: u8 = 0xA7;
const MHMU_MASK: u32 = 0xff0_00700;

/// Byte array capable of holding a 802.15.4 package
pub type PacketBuffer = [u8; MAX_PACKET_LENGHT as usize];

/// Clear all interrupts on the radio
fn clear_interrupts(radio: &mut RADIO) {
    radio.intenclr.write(|w| unsafe { w.bits(0xffff_ffff) });
}

/// Configure interrupts
fn configure_interrupts(radio: &mut RADIO) {
    clear_interrupts(radio);
    // Enable interrupts for READY, DISABLED, CCABUSY and PHYEND
    radio.intenset.write(|w| {
        w.ready()
            .set()
            .disabled()
            .set()
            .ccabusy()
            .set()
            .phyend()
            .set()
            .bcmatch()
            .set()
    });
}

/// State flag for when the radio is transmitting
pub const STATE_SEND: u32 = 1 << 0;

/// Errors returned by Radio
pub enum Error {
    /// Clear channel assesment returned that the channel is busy
    CcaBusy,
}

/// # 802.15.4 PHY layer implementation for nRF Radio
///
/// This is work in progress.
///
pub struct Radio {
    /// The nRF52 radio peripheral
    radio: RADIO,
    /// Internal buffer
    buffer: PacketBuffer,
    /// Internal state
    state: u32,
}

impl Radio {
    /// Initialise the radio in 802.15.4 mode
    pub fn new(mut radio: RADIO) -> Self {
        // Enable 802.15.4 mode
        radio.mode.write(|w| w.mode().ieee802154_250kbit());
        // Configure CRC skip address
        radio
            .crccnf
            .write(|w| w.len().two().skipaddr().ieee802154());
        unsafe {
            // Configure CRC polynominal and init
            radio.crcpoly.write(|w| w.crcpoly().bits(CRC_POLYNOMIAL));
            radio.crcinit.write(|w| w.crcinit().bits(0));
            // Configure packet layout
            // 8-bit on air length
            // S0 length, zero bytes
            // S1 length, zero bytes
            // S1 included in RAM if S1 length > 0, No.
            // Code Indicator length, 0
            // Preamble length 32-bit zero
            // Exclude CRC
            // No TERM field
            radio.pcnf0.write(|w| {
                w.lflen()
                    .bits(8)
                    .s0len()
                    .clear_bit()
                    .s1len()
                    .bits(0)
                    .s1incl()
                    .clear_bit()
                    .cilen()
                    .bits(0)
                    .plen()
                    ._32bit_zero()
                    .crcinc()
                    .set_bit()
            });
            radio.pcnf1.write(|w| {
                w.maxlen()
                    .bits(MAX_PACKET_LENGHT_REG)
                    .statlen()
                    .bits(0)
                    .balen()
                    .bits(0)
                    .endian()
                    .clear_bit()
                    .whiteen()
                    .clear_bit()
            });
            // Configure clear channel assessment to sane default
            radio.ccactrl.write(|w| {
                w.ccamode()
                    .ed_mode()
                    .ccaedthres()
                    .bits(CCA_ED_THRESHOLD_DEFAULT)
                    .ccacorrthres()
                    .bits(CCA_CORR_THRESHOLD_DEFAULT)
                    .ccacorrthres()
                    .bits(CCA_CORR_LIMIT_DEFAULT)
            });
            // Configure MAC header match
            radio.mhrmatchmas.write(|w| w.bits(MHMU_MASK));
            radio.mhrmatchconf.write(|w| w.bits(0));
            // Start of frame delimiter
            radio.sfd.write(|w| w.sfd().bits(SFD_DEFAULT));
            radio.bcc.write(|w| w.bcc().bits(24));
        }
        // Set transmission power to 4dBm
        radio.txpower.write(|w| w.txpower().pos4d_bm());

        // Configure interrupts
        configure_interrupts(&mut radio);

        Self {
            radio,
            buffer: [0u8; MAX_PACKET_LENGHT],
            state: 0,
        }
    }

    fn clear_interrupts(&mut self) {
        clear_interrupts(&mut self.radio);
    }

    fn configure_interrupts(&mut self) {
        configure_interrupts(&mut self.radio);
    }

    /// Configure channel to use
    ///
    /// There are 16 channels, 11 to 26. The channel frequency can be calculated as follows,
    ///
    /// frequency = 2400 MHz + ((channel - 10) * 5 MHz)
    ///
    pub fn set_channel(&mut self, channel: u8) {
        if channel < 11 || channel > 26 {
            panic!("Bad 802.15.4 channel");
        }
        let frequency_offset = (channel - 10) * 5;
        self.radio
            .frequency
            .write(|w| unsafe { w.frequency().bits(frequency_offset).map().default() });
    }

    /// Get the configured channel
    pub fn get_channel(&mut self) -> u8 {
        let frequency_offset = self.radio.frequency.read().frequency().bits();
        (frequency_offset / 5) + 10
    }

    /// Busy sending
    pub fn is_tx_busy(&self) -> bool {
        self.state & STATE_SEND == STATE_SEND
    }

    /// Configure transmission power
    ///
    /// Valid power levels are 8-2,0,-4,-8,-12,-16,-20,-40 dBm
    pub fn set_transmission_power(&mut self, power: i8) {
        match power {
            8 => self.radio.txpower.write(|w| w.txpower().pos8d_bm()),
            7 => self.radio.txpower.write(|w| w.txpower().pos7d_bm()),
            6 => self.radio.txpower.write(|w| w.txpower().pos6d_bm()),
            5 => self.radio.txpower.write(|w| w.txpower().pos5d_bm()),
            4 => self.radio.txpower.write(|w| w.txpower().pos4d_bm()),
            3 => self.radio.txpower.write(|w| w.txpower().pos3d_bm()),
            2 => self.radio.txpower.write(|w| w.txpower().pos2d_bm()),
            0 => self.radio.txpower.write(|w| w.txpower()._0d_bm()),
            -4 => self.radio.txpower.write(|w| w.txpower().neg4d_bm()),
            -8 => self.radio.txpower.write(|w| w.txpower().neg8d_bm()),
            -12 => self.radio.txpower.write(|w| w.txpower().neg12d_bm()),
            -16 => self.radio.txpower.write(|w| w.txpower().neg16d_bm()),
            -20 => self.radio.txpower.write(|w| w.txpower().neg20d_bm()),
            -40 => self.radio.txpower.write(|w| w.txpower().neg40d_bm()),
            _ => panic!("Bad transmission power value"),
        }
    }

    // Enter the disabled state
    fn enter_disabled(&mut self) {
        if self.state() != radio::state::STATE_A::DISABLED {
            self.radio
                .tasks_disable
                .write(|w| w.tasks_disable().set_bit());
            loop {
                if self
                    .radio
                    .events_disabled
                    .read()
                    .events_disabled()
                    .bit_is_set()
                {
                    break;
                }
            }
        }
        self.radio.events_disabled.reset();
    }

    /// Get the radio state
    pub fn state(&mut self) -> radio::state::STATE_A {
        match self.radio.state.read().state().variant() {
            Variant::Val(state) => state,
            Variant::Res(_) => unreachable!(),
        }
    }

    /// Prepare to receive data
    pub fn receive_prepare(&mut self) {
        self.enter_disabled();
        self.radio.shorts.reset();
        self.radio
            .shorts
            .write(|w| w.rxready_start().enabled().phyend_start().enabled());
        self.radio.tasks_rxen.write(|w| w.tasks_rxen().set_bit());
    }

    /// Read received data into buffer
    ///
    /// ```notrust
    /// ------------------------
    /// | size | payload | LQI |
    /// ------------------------
    ///    1        *       1     octets
    /// ```
    ///
    /// The first octet in the buffer is the size of the packet (including size and LQI). Then
    /// comes the payload. Last octet is the link quality indicator (LQI).
    ///
    /// # Return
    ///
    /// Returns the number of bytes received, or zero if no data could be received.
    ///
    pub fn receive(&mut self, buffer: &mut PacketBuffer) -> Result<usize, Error> {
        self.receive_slice(&mut buffer[..])
    }

    /// Read received data into byte slice
    ///
    /// ```notrust
    /// ------------------------
    /// | size | payload | LQI |
    /// ------------------------
    ///    1        *       1     octets
    /// ```
    ///
    /// The first octet in the buffer is the size of the packet (including size and LQI). Then
    /// comes the payload. Last octet is the link quality indicator (LQI).
    ///
    /// # Return
    ///
    /// Returns the number of bytes received, or zero if no data could be received.
    ///
    pub fn receive_slice(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        assert!(buffer.len() >= MAX_PACKET_LENGHT);
        // PHYEND event signal
        let length = if self.radio.events_phyend.read().events_phyend().bit_is_set() {
            // PHR contains length of the packet in the low 7 bits, MSB
            // indicates if this packet is a 802.11.4 packet or not
            // 16-bit CRC has been removed, 1 octet LQI has been added to the end
            let phr = self.buffer[0];
            // Clear PHR so we do not read old data next time
            self.buffer[0] = 0;
            let length = if self.state & STATE_SEND == STATE_SEND {
                0
            } else {
                let length = if (phr & 0x80) == 0 {
                    (phr & 0x7f) as usize
                } else {
                    0
                };
                if length > 0 {
                    buffer[0] = phr & 0x7f;
                    buffer[1..=length].copy_from_slice(&self.buffer[1..=length]);
                }
                length
            };
            // Clear interrupt
            self.radio.events_phyend.reset();
            length
        } else {
            0
        };
        if self
            .radio
            .events_disabled
            .read()
            .events_disabled()
            .bit_is_set()
        {
            // Errata 204: Always use DISABLE when switching from TX to RX.
            if self.state & STATE_SEND == STATE_SEND {
                // Re-enable receive after sending a packet
                self.radio.shorts.reset();
                self.radio
                    .shorts
                    .write(|w| w.rxready_start().enabled().phyend_start().enabled());
                self.radio.tasks_rxen.write(|w| w.tasks_rxen().set_bit());
                self.state = 0;
            }
            // Clear interrupt
            self.radio.events_disabled.reset();
        }
        if self.radio.events_ready.read().events_ready().bit_is_set() {
            self.radio
                .packetptr
                .write(|w| unsafe { w.bits(self.buffer.as_ptr() as u32) });
            // Clear interrupt
            self.radio.events_ready.reset();
        }
        if self
            .radio
            .events_ccabusy
            .read()
            .events_ccabusy()
            .bit_is_set()
        {
            self.receive_prepare();
            // Clear interrupt
            self.radio.events_ccabusy.reset();
            return Err(Error::CcaBusy);
        }
        if self
            .radio
            .events_bcmatch
            .read()
            .events_bcmatch()
            .bit_is_set()
        {
            // Clear interrupt
            self.radio.events_bcmatch.reset();
        }
        Ok(length)
    }

    /// Queue a transmission of the provided data
    ///
    /// `data` should contain the packet payload to be sent without the PHR and FCS.
    ///
    /// If the transmission succeeds the PHYEND event shall signal. The
    /// transmission might fail if the channel is used, then the CCABUSY event
    /// will be signalled.
    ///
    /// # Return
    ///
    /// Returns the number of bytes queued for transmission, or zero if no data could be sent.
    ///
    pub fn queue_transmission(&mut self, data: &[u8]) -> usize {
        self.enter_disabled();
        let data_length = data.len();
        let tx_length = data_length + 2; // The radio will add FCS, two octets
        assert!(tx_length < (MAX_PACKET_LENGHT - 1) as usize);
        self.buffer[0] = tx_length as u8;
        self.buffer[1..(tx_length - 1)].copy_from_slice(data);
        // Configure shortcuts
        //
        // The radio goes through following states when sending a 802.15.4 packet
        //
        // enable RX → ramp up RX → clear channel assesment (CCA) → CCA result
        // CCA idle → enable TX → start TX → TX → end (PHYEND) → disabled
        //
        // CCA might end up in the event CCABUSY in which there will be no transmission
        self.radio.shorts.reset();
        self.radio.shorts.write(|w| {
            w.rxready_ccastart()
                .enabled()
                .ccaidle_txen()
                .enabled()
                .txready_start()
                .enabled()
                .ccabusy_disable()
                .enabled()
                .phyend_disable()
                .enabled()
        });
        compiler_fence(Ordering::Release);
        // Start task
        self.radio.tasks_rxen.write(|w| w.tasks_rxen().set_bit());
        self.state |= STATE_SEND;
        data_length
    }

    /// Start a energy detect query on the current channel
    ///
    /// # Return
    ///
    /// Returns true if the energy detection query could be started.
    ///
    pub fn start_energy_detect(&mut self, count: u32) -> bool {
        if count > 0 && count <= 0x10_0000 {
            self.enter_disabled();
            self.radio.edcnt.write(|w| unsafe { w.bits(count - 1) });
            self.radio.shorts.reset();
            self.radio
                .shorts
                .write(|w| w.ready_edstart().enabled().edend_disable().enabled());
            self.radio
                .events_edend
                .write(|w| w.events_edend().clear_bit());
            self.clear_interrupts();
            // Enable interrupts for EDEND
            self.radio.intenset.write(|w| w.edend().set());
            // Start energy detection
            self.radio.tasks_rxen.write(|w| w.tasks_rxen().set_bit());
            true
        } else {
            false
        }
    }

    /// Energy detect result
    ///
    /// # Return
    ///
    /// Returns the energy level, or None.
    ///
    pub fn report_energy_detect(&mut self) -> Option<u8> {
        if self.radio.events_edend.read().events_edend().bit_is_set() {
            self.radio.events_edend.reset();
            let level = self.radio.edsample.read().edlvl().bits();
            self.radio
                .events_edend
                .write(|w| w.events_edend().clear_bit());
            self.configure_interrupts();
            Some(level)
        } else {
            None
        }
    }
}
