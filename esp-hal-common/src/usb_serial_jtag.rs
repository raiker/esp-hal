use crate::pac::{usb_device::RegisterBlock, USB_DEVICE};

pub struct UsbSerialJtag<T> {
    usb_serial: T,
}

/// Custom USB serial error type
#[derive(Debug)]
pub enum Error {}

impl<T> UsbSerialJtag<T>
where
    T: Instance,
{
    /// Create a new USB serial/JTAG instance with defaults
    pub fn new(usb_serial: T) -> Self {
        let mut dev = Self { usb_serial };
        dev.usb_serial.disable_rx_interrupts();
        dev.usb_serial.disable_tx_interrupts();

        dev
    }

    /// Return the raw interface to the underlying USB serial/JTAG instance
    pub fn free(self) -> T {
        self.usb_serial
    }

    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        let reg_block = self.usb_serial.register_block();

        // TODO: 64 byte chunks max
        for chunk in data.chunks(32) {
            unsafe {
                for &b in chunk {
                    reg_block.ep1.write(|w| w.rdwr_byte().bits(b.into()))
                }
                reg_block.ep1_conf.write(|w| w.wr_done().set_bit());

                while reg_block.ep1_conf.read().bits() & 0b011 == 0b000 {
                    // wait
                }
            }
        }

        Ok(())
    }

    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        let reg_block = self.usb_serial.register_block();

        // Check if there are any bytes to read
        if reg_block
            .ep1_conf
            .read()
            .serial_out_ep_data_avail()
            .bit_is_set()
        {
            let value = reg_block.ep1.read().rdwr_byte().bits();

            Ok(value)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Listen for RX-PACKET-RECV interrupts
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        let reg_block = self.usb_serial.register_block();
        reg_block
            .int_ena
            .modify(|_, w| w.serial_out_recv_pkt_int_ena().set_bit());
    }

    /// Stop listening for RX-PACKET-RECV interrupts
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        let reg_block = self.usb_serial.register_block();
        reg_block
            .int_ena
            .modify(|_, w| w.serial_out_recv_pkt_int_ena().clear_bit());
    }

    /// Checks if RX-PACKET-RECV interrupt is set
    pub fn rx_packet_recv_interrupt_set(&mut self) -> bool {
        let reg_block = unsafe { &*USB_DEVICE::PTR };
        reg_block
            .int_st
            .read()
            .serial_out_recv_pkt_int_st()
            .bit_is_set()
    }

    /// Reset RX-PACKET-RECV interrupt
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        let reg_block = unsafe { &*USB_DEVICE::PTR };

        reg_block
            .int_clr
            .write(|w| w.serial_out_recv_pkt_int_clr().set_bit())
    }
}

/// USB serial/JTAG peripheral instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn disable_tx_interrupts(&mut self) {
        self.register_block()
            .int_ena
            .write(|w| w.serial_in_empty_int_ena().clear_bit());

        self.register_block()
            .int_clr
            .write(|w| w.serial_in_empty_int_clr().set_bit())
    }

    fn disable_rx_interrupts(&mut self) {
        self.register_block()
            .int_ena
            .write(|w| w.serial_out_recv_pkt_int_ena().clear_bit());

        self.register_block()
            .int_clr
            .write(|w| w.serial_out_recv_pkt_int_clr().set_bit())
    }

    fn get_rx_fifo_count(&self) -> u16 {
        let ep0_state = self.register_block().in_ep0_st.read();
        let wr_addr = ep0_state.in_ep0_wr_addr().bits();
        let rd_addr = ep0_state.in_ep0_rd_addr().bits();
        (wr_addr - rd_addr).into()
    }

    fn get_tx_fifo_count(&self) -> u16 {
        let ep1_state = self.register_block().in_ep1_st.read();
        let wr_addr = ep1_state.in_ep1_wr_addr().bits();
        let rd_addr = ep1_state.in_ep1_rd_addr().bits();
        (wr_addr - rd_addr).into()
    }
}

impl Instance for USB_DEVICE {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}

impl<T> core::fmt::Write for UsbSerialJtag<T>
where
    T: Instance,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

impl<T> embedded_hal::serial::Read<u8> for UsbSerialJtag<T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}
