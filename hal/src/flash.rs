//! Flash memory.

// Standard programming
// The Flash memory programming sequence in standard mode is as follows:
// 1. Check that no Flash main memory operation is ongoing by checking BSY in
// FLASH_SR or FLASH_C2SR.
// 2. Check that Flash program and erase operation is allowed by checking PESD in
// FLASH_SR or FLASH_C2SR (these checks are recommended even if status may
// change due to Flash operation requests by the other CPU, to limit the risk of receiving
// a bus error when starting programming).
// 3. Check and clear all error programming flags due to a previous programming. If not,
// PGSERR is set.
// 4. Set PG in FLASH_CR or FLASH_C2CR.
// 5. Perform the data write operation at the desired memory address, inside the main
// memory block or OTP area. Only double-word (64 bits) can be programmed.
// a) Write a first word in an address aligned with double-word
// b) Write the second word (see the note below)
// Note: When the Flash memory interface received a good sequence (a double-word),
// programming is automatically launched and the BSY bit is set. The internal oscillator
// HSI16 (16 MHz) is enabled automatically when the PG bit is set, and disabled
// automatically when the PG bit is cleared, except if the HSI16 is previously enabled
// with HSION in the RCC_CR register.
// If the user needs to program only one word, double-word must be completed with the
// erase value 0xFFFF FFFF to launch automatically the programming.
// ECC is calculated from the double-word to program.
// For correct operation, the firmware must guarantee that the Flash page access
// protection is not changed during the programming sequence. This is between the first
// and second word write.
// 6. Wait until BSY is cleared in FLASH_SR or FLASH_C2SR.
// 7. Check that EOP is set in FLASH_SR or FLASH_C2SR (meaning the programming
// operation succeeded), and clear it by software.
// 8. Clear PG in FLASH_SR or FLASH_C2SR if there no more programming request

use crate::pac;
use core::ptr::write_volatile;

/// Flash errors.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Busy Error.
    ///
    /// A flash programming sequence was started while the previous sequence
    /// was still in-progress.
    Busy,
    /// Program erase suspend error.
    ///
    /// A flash programming sequence was started with a program erase suspend
    /// bit set.
    Suspend,
    // /// A fast programming sequence is interrupted due to an error, such as:
    // ///
    // /// * alignment
    // /// * size
    // /// * write protection
    // /// * data miss
    // ///
    // /// The corresponding status bit
    // // (PGAERR, SIZERR, WRPERR or MISSERR) is set at the same time.
    // Fast,
    /// Fast programming data miss error.
    ///
    /// In Fast programming mode, 32 double-words (256 bytes) must be sent to
    /// the flash memory successively and the new data must be sent to the logic
    /// control before the current data is fully programmed.
    ///
    /// This bit is set by hardware when the new data is not present in time and
    /// cleared by writing 1.
    Miss,
    /// Programming sequence error.
    ///
    /// This bit is set by hardware when a write access to the flash memory is
    /// performed by the code, while PG or FSTPG have not been set previously.
    /// This bit is also set by hardware when PROGERR, SIZERR, PGAERR, WRPERR,
    /// MISSERR or FASTERR is set due to a
    /// previous programming error.
    Seq,
    /// Size error.
    ///
    /// This bit is set by hardware when the size of the access is a byte (`u8`)
    /// or half-word (`u16`) during a program or a fast program sequence.
    /// Only double-word (`u64`) programming is allowed (consequently: word (`u32`) access).
    Size,
    /// Programming alignment error.
    ///
    /// This bit is set by hardware when the data to program cannot be contained in the same
    /// double-word (64 bits) Flash memory in case of standard programming, or if there is a change
    /// of page during fast programming.
    Align,
    /// Write protection error.
    ///
    /// An address to be erased/programmed belongs to a write-protected part
    /// (by WRP, PCROP or RDP level 1) of the flash memory.
    Wp,
    /// Programming error.
    ///
    /// A 64-bit address to be programmed contains a value different from
    /// `0xFFFF_FFFF_FFFF_FFFF` before programming, except if the data to write
    /// is `0x0000_0000_0000_0000`.
    Prog,
    // This bit is set by hardware when a Flash memory operation (program/erase) completes
    // unsuccessfully. This bit is set only if error interrupts are enabled (ERRIE = 1).
    Op,
}

/// Flash driver.
#[derive(Debug)]
pub struct Flash {
    flash: pac::FLASH,
}

impl Flash {
    #[inline]
    pub const fn new(flash: pac::FLASH) -> Self {
        Flash { flash }
    }

    #[inline]
    pub const fn free(self) -> pac::FLASH {
        self.flash
    }

    /// Returns `true` if the `bsy` bit is set in the status register.
    #[inline]
    pub fn bsy() -> bool {
        #[cfg(not(feature = "stm32wl5x_cm0p"))]
        unsafe {
            (*pac::FLASH::PTR).sr.read().bsy().bit_is_set()
        }
        #[cfg(feature = "stm32wl5x_cm0p")]
        unsafe {
            (*pac::FLASH::PTR).c2sr.read().bsy().bit_is_set()
        }
    }

    /// Returns `true` if the `pesd` bit is set in the status register.
    #[inline]
    pub fn pesd() -> bool {
        #[cfg(not(feature = "stm32wl5x_cm0p"))]
        unsafe {
            (*pac::FLASH::PTR).sr.read().pesd().bit_is_set()
        }
        #[cfg(feature = "stm32wl5x_cm0p")]
        unsafe {
            (*pac::FLASH::PTR).c2sr.read().pesd().bit_is_set()
        }
    }

    #[rustfmt::skip]
    fn clear_all_err() {
        #[cfg(not(feature = "stm32wl5x_cm0p"))]
        unsafe {
            (*pac::FLASH::PTR).sr.write(|w| {
                w
                    .rderr().clear()
                    .fasterr().clear()
                    .misserr().clear()
                    .pgserr().clear()
                    .sizerr().clear()
                    .pgaerr().clear()
                    .wrperr().clear()
                    .progerr().clear()
                    .operr().clear()
                    .eop().clear()
            })
        }
        #[cfg(feature = "stm32wl5x_cm0p")]
        unsafe {
            (*pac::FLASH::PTR).c2sr.write(|w| {
                w
                    .rderr().clear()
                    .fasterr().clear()
                    .misserr().clear()
                    .pgserr().clear()
                    .sizerr().clear()
                    .pgaerr().clear()
                    .wrperr().clear()
                    .progerr().clear()
                    .operr().clear()
                    .eop().clear()
            })
        }
    }

    fn wait_for_eop() -> Result<(), Error> {
        warn!("TODO: CHECK FOR ERRORS HERE");

        loop {
            #[cfg(not(feature = "stm32wl5x_cm0p"))]
            let sr: u32 = unsafe { (*pac::FLASH::PTR).sr.read().bits() };
            #[cfg(feature = "stm32wl5x_cm0p")]
            let sr: u32 = unsafe { (*pac::FLASH::PTR).c2sr.read().bits() };

            if sr & 0b1 == 0b1 {
                return Ok(());
            }
        }
    }

    /// Peforms the standard program sequence, writing 64 bits into flash
    /// memory.
    #[allow(unused_unsafe)]
    pub unsafe fn standard_program(&mut self, to: *mut u64, from: *const u64) -> Result<(), Error> {
        if Self::bsy() {
            return Err(Error::Busy);
        }
        if Self::pesd() {
            return Err(Error::Suspend);
        }

        Self::clear_all_err();

        #[cfg(not(feature = "stm32wl5x_cm0p"))]
        self.flash.cr.modify(|_, w| w.pg().set_bit());
        #[cfg(feature = "stm32wl5x_cm0p")]
        self.flash.c2cr.modify(|_, w| w.pg().set_bit());

        unsafe {
            write_volatile(to as *mut u32, (from as *const u32).read());
            write_volatile(
                (to as *mut u32).offset(1) as *mut u32,
                (from as *const u32).offset(1).read(),
            );
        }

        Self::wait_for_eop()?;

        #[cfg(not(feature = "stm32wl5x_cm0p"))]
        self.flash.cr.modify(|_, w| w.pg().clear_bit());
        #[cfg(feature = "stm32wl5x_cm0p")]
        self.flash.c2cr.modify(|_, w| w.pg().clear_bit());

        Ok(())
    }
}
