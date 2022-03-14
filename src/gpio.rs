//! GPIO and pin configuration

use core::marker::PhantomData;

#[cfg(not(feature = "riscv-ulp-hal"))]
use esp_idf_sys::*;

#[cfg(feature = "riscv-ulp-hal")]
use crate::riscv_ulp_hal::sys::*;

use crate::adc;

pub use chip::*;

/// A trait implemented by every pin insance
pub trait Pin: Send {
    type Error;

    fn pin(&self) -> i32;
}

/// A marker trait designating a pin which is capable of
/// operating as an input pin, even if its current mode
/// might be a different one
pub trait InputPin: Pin {}

/// A marker trait designating a pin which is capable of
/// operating as an output pin, even if its current mode
/// might be a different one
pub trait OutputPin: Pin {}

/// Functions available on pins with pull up/down resistors
//
// This is split into a separate trait from OutputPin, because for pins which also connect to
// the RTCIO mux, the pull up/down needs to be set via the RTCIO mux.
pub trait Pull {
    type Error;

    /// Enable internal pull up resistor, disable pull down
    fn set_pull_up(&mut self) -> Result<&mut Self, Self::Error>;

    /// Enable internal pull down resistor, disable pull up
    fn set_pull_down(&mut self) -> Result<&mut Self, Self::Error>;

    /// Enable internal pull up and down resistors
    fn set_pull_up_down(&mut self) -> Result<&mut Self, Self::Error>;

    /// Disable internal pull up and down resistors
    fn set_floating(&mut self) -> Result<&mut Self, Self::Error>;
}

pub trait RTCPin: Pin {
    fn rtc_pin(&self) -> i32;
}

/// A marker trait designating a pin which is capable of
/// operating as an ADC pin, even if its current mode
/// might be a different one
pub trait ADCPin: Pin {
    fn adc_unit(&self) -> adc_unit_t;
    fn adc_channel(&self) -> adc_channel_t;
}

/// A marker trait designating a pin which is capable of
/// operating as a DAC pin, even if its current mode
/// might be a different one
#[cfg(all(not(esp32c3), not(esp32s3)))]
pub trait DACPin: Pin {
    fn dac_channel(&self) -> dac_channel_t;
}

/// A marker trait designating a pin which is capable of
/// operating as a touch pin, even if its current mode
/// might be a different one
#[cfg(not(esp32c3))]
pub trait TouchPin: Pin {
    fn touch_channel(&self) -> touch_pad_t;
}

pub struct Input;

pub struct Output;

pub struct InputOutput;

pub struct Disabled;

pub struct Unknown;

pub struct LedcPwm;
pub struct LedcPwmChannel(ledc_channel_t);

pub struct McPwm;
pub struct McPwmChannel(mcpwm_unit_t, mcpwm_generator_t);

pub type PwmDuty = u32;

pub struct McPwmPin {
    unit: mcpwm_unit_t,
    timer: mcpwm_timer_t,
    generator: mcpwm_generator_t,
    _pin: GpioPin<McPwm>,
}

pub struct LedcPwmPin {
    channel: ledc_channel_t,
    speed_mode: ledc_mode_t,
    _pin: GpioPin<LedcPwm>,
}

pub trait PwmPinWithMicros: embedded_hal::pwm::blocking::PwmPin {
    fn set_duty_micros(&mut self, duty: u32) -> Result<(), EspError>;
}

impl embedded_hal::pwm::blocking::PwmPin for LedcPwmPin {
    type Duty = PwmDuty;
    type Error = EspError;

    fn disable(&mut self) -> Result<(), EspError> {
        Ok(())
    }

    /// Enables a PWM `channel`
    fn enable(&mut self) -> Result<(), EspError> {
        Ok(())
    }

    /// Returns the current duty cycle
    fn get_duty(&self) -> Result<Self::Duty, EspError> {
        Ok(unsafe { ledc_get_duty(self.speed_mode, self.channel) })
    }

    /// Returns the maximum duty cycle value
    fn get_max_duty(&self) -> Result<Self::Duty, EspError> {
        Ok(2_u32.pow(ledc_timer_bit_t_LEDC_TIMER_13_BIT))
    }

    /// Sets a new duty cycle
    #[inline]
    fn set_duty(&mut self, duty: Self::Duty) -> Result<(), EspError> {
        esp!(unsafe { ledc_set_duty(self.speed_mode, self.channel, duty) })?;
        esp!(unsafe { ledc_update_duty(self.speed_mode, self.channel) })
    }
}

impl embedded_hal::pwm::blocking::PwmPin for McPwmPin {
    type Duty = PwmDuty;
    type Error = EspError;

    fn disable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { mcpwm_set_signal_low(self.unit, self.timer, self.generator) })
    }

    /// Enables a PWM `channel`
    fn enable(&mut self) -> Result<(), EspError> {
        esp!(unsafe {
            mcpwm_set_duty_type(
                self.unit,
                self.timer,
                self.generator,
                mcpwm_duty_type_t_MCPWM_DUTY_MODE_0,
            )
        })
    }

    /// Returns the current duty cycle
    fn get_duty(&self) -> Result<Self::Duty, EspError> {
        let duty = unsafe { mcpwm_get_duty(self.unit, self.timer, self.generator) };
        Ok((duty * self.get_max_duty()? as f32).round() as u32)
    }

    /// Returns the maximum duty cycle value
    fn get_max_duty(&self) -> Result<Self::Duty, EspError> {
        Ok(Self::Duty::MAX)
    }

    /// Sets a new duty cycle
    #[inline]
    fn set_duty(&mut self, duty: Self::Duty) -> Result<(), EspError> {
        let duty = duty as f32 / Self::Duty::MAX as f32 * 100.;
        esp!(unsafe { mcpwm_set_duty(self.unit, self.timer, self.generator, duty) })
    }
}

impl PwmPinWithMicros for McPwmPin {
    #[inline]
    fn set_duty_micros(&mut self, duty: u32) -> Result<(), EspError> {
        esp!(unsafe { mcpwm_set_duty_in_us(self.unit, self.timer, self.generator, duty) })
    }
}

/// Drive strength (values are approximates)
#[cfg(not(feature = "riscv-ulp-hal"))]
pub enum DriveStrength {
    I5mA = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl From<DriveStrength> for gpio_drive_cap_t {
    fn from(strength: DriveStrength) -> gpio_drive_cap_t {
        match strength {
            DriveStrength::I5mA => gpio_drive_cap_t_GPIO_DRIVE_CAP_0,
            DriveStrength::I10mA => gpio_drive_cap_t_GPIO_DRIVE_CAP_1,
            DriveStrength::I20mA => gpio_drive_cap_t_GPIO_DRIVE_CAP_2,
            DriveStrength::I40mA => gpio_drive_cap_t_GPIO_DRIVE_CAP_3,
        }
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl From<gpio_drive_cap_t> for DriveStrength {
    #[allow(non_upper_case_globals)]
    fn from(cap: gpio_drive_cap_t) -> DriveStrength {
        match cap {
            gpio_drive_cap_t_GPIO_DRIVE_CAP_0 => DriveStrength::I5mA,
            gpio_drive_cap_t_GPIO_DRIVE_CAP_1 => DriveStrength::I10mA,
            gpio_drive_cap_t_GPIO_DRIVE_CAP_2 => DriveStrength::I20mA,
            gpio_drive_cap_t_GPIO_DRIVE_CAP_3 => DriveStrength::I40mA,
            other => panic!("Unknown GPIO pin drive capability: {}", other),
        }
    }
}

macro_rules! impl_base {
    ($pxi:ident) => {
        #[allow(dead_code)]
        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            fn reset(&mut self) -> Result<(), EspError> {
                #[cfg(not(feature = "riscv-ulp-hal"))]
                let res = esp_result!(unsafe { gpio_reset_pin(self.pin()) }, ());
                #[cfg(feature = "riscv-ulp-hal")]
                let res = Ok(());

                res
            }

            fn get_input_level(&self) -> bool {
                (unsafe { gpio_get_level(self.pin()) } != 0)
            }

            #[cfg(not(feature = "riscv-ulp-hal"))]
            fn get_output_level(&self) -> bool {
                let pin = self.pin() as u32;

                #[cfg(esp32c3)]
                let is_set_high = unsafe { (*(GPIO_OUT_REG as *const u32) >> pin) & 0x01 != 0 };
                #[cfg(not(esp32c3))]
                let is_set_high = if pin <= 31 {
                    // GPIO0 - GPIO31
                    unsafe { (*(GPIO_OUT_REG as *const u32) >> pin) & 0x01 != 0 }
                } else {
                    // GPIO32+
                    unsafe { (*(GPIO_OUT1_REG as *const u32) >> (pin - 32)) & 0x01 != 0 }
                };

                is_set_high
            }

            #[cfg(feature = "riscv-ulp-hal")]
            fn get_output_level(&self) -> bool {
                (unsafe { gpio_get_output_level(self.pin()) } != 0)
            }

            fn set_output_level(&mut self, on: bool) -> Result<(), EspError> {
                esp_result!(unsafe { gpio_set_level(self.pin(), (on as u8).into()) }, ())
            }

            #[cfg(not(feature = "riscv-ulp-hal"))]
            pub fn get_drive_strength(&self) -> Result<DriveStrength, EspError> {
                let mut cap: gpio_drive_cap_t = 0;

                esp!(unsafe { gpio_get_drive_capability(self.pin(), &mut cap as *mut _) })?;

                Ok(cap.into())
            }

            #[cfg(not(feature = "riscv-ulp-hal"))]
            pub fn set_drive_strength(&mut self, strength: DriveStrength) -> Result<(), EspError> {
                esp!(unsafe { gpio_set_drive_capability(self.pin(), strength.into()) })?;

                Ok(())
            }

            fn set_disabled(&mut self) -> Result<(), EspError> {
                esp!(unsafe { gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_DISABLE,) })?;

                Ok(())
            }

            fn set_input(&mut self) -> Result<(), EspError> {
                esp!(unsafe {
                    esp_rom_gpio_pad_select_gpio(self.pin() as u32);
                    gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_INPUT)
                })?;

                Ok(())
            }

            fn set_input_output(&mut self) -> Result<(), EspError> {
                esp!(unsafe {
                    esp_rom_gpio_pad_select_gpio(self.pin() as u32);
                    gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_INPUT_OUTPUT)
                })?;

                Ok(())
            }

            fn set_input_output_od(&mut self) -> Result<(), EspError> {
                esp!(unsafe {
                    esp_rom_gpio_pad_select_gpio(self.pin() as u32);
                    gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD)
                })?;

                Ok(())
            }

            fn set_output(&mut self) -> Result<(), EspError> {
                esp!(unsafe {
                    esp_rom_gpio_pad_select_gpio(self.pin() as u32);
                    gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_OUTPUT)
                })?;

                Ok(())
            }

            fn set_output_od(&mut self) -> Result<(), EspError> {
                esp!(unsafe {
                    esp_rom_gpio_pad_select_gpio(self.pin() as u32);
                    gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_OUTPUT_OD)
                })?;

                Ok(())
            }
        }
    };
}

macro_rules! impl_pull {
    ($pxi:ident: $mode:ident) => {
        impl Pull for $pxi<$mode> {
            type Error = EspError;

            fn set_pull_up(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe { gpio_set_pull_mode(self.pin(), gpio_pull_mode_t_GPIO_PULLUP_ONLY,) },
                    self
                )
            }

            fn set_pull_down(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe { gpio_set_pull_mode(self.pin(), gpio_pull_mode_t_GPIO_PULLDOWN_ONLY,) },
                    self
                )
            }

            fn set_pull_up_down(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe {
                        gpio_set_pull_mode(self.pin(), gpio_pull_mode_t_GPIO_PULLUP_PULLDOWN)
                    },
                    self
                )
            }

            fn set_floating(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe { gpio_set_pull_mode(self.pin(), gpio_pull_mode_t_GPIO_FLOATING,) },
                    self
                )
            }
        }
    };
}

macro_rules! impl_input_base {
    ($pxi:ident: $pin:expr) => {
        pub struct $pxi<MODE> {
            _mode: PhantomData<MODE>,
        }

        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            /// # Safety
            ///
            /// Care should be taken not to instantiate a pin which is already used elsewhere
            pub unsafe fn new() -> $pxi<Unknown> {
                $pxi { _mode: PhantomData }
            }

            pub fn into_unknown(self) -> Result<$pxi<Unknown>, EspError> {
                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_disabled(mut self) -> Result<$pxi<Disabled>, EspError> {
                self.set_disabled()?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_input(mut self) -> Result<$pxi<Input>, EspError> {
                self.set_input()?;

                Ok($pxi { _mode: PhantomData })
            }

            /// Degrades a concrete pin (e.g. [`Gpio1`]) to a generic pin
            /// struct that can also be used with periphals.
            pub fn degrade(self) -> GpioPin<MODE> {
                unsafe { GpioPin::new($pin) }
            }
        }

        impl<MODE> Pin for $pxi<MODE>
        where
            MODE: Send,
        {
            type Error = EspError;

            #[inline(always)]
            fn pin(&self) -> i32 {
                $pin
            }
        }

        impl<MODE> InputPin for $pxi<MODE> where MODE: Send {}

        impl_base!($pxi);
        impl_hal_input_pin!($pxi: Input);
    };
}

#[allow(unused)]
macro_rules! impl_input_only {
    ($pxi:ident: $pin:expr) => {
        impl_input_base!($pxi: $pin);
    };
}

macro_rules! impl_input_output {
    ($pxi:ident: $pin:expr) => {
        impl_input_base!($pxi: $pin);
        impl_pull!($pxi: Input);
        impl_pull!($pxi: InputOutput);
        impl_hal_input_pin!($pxi: InputOutput);

        impl<MODE> OutputPin for $pxi<MODE> where MODE: Send {}

        impl_hal_output_pin!($pxi: InputOutput);
        impl_hal_output_pin!($pxi: Output);

        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            pub fn into_input_output(mut self) -> Result<$pxi<InputOutput>, EspError> {
                self.set_input_output()?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_input_output_od(mut self) -> Result<$pxi<InputOutput>, EspError> {
                self.set_input_output_od()?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_mcpwm(self, channel: McPwmChannel) -> Result<McPwmPin, EspError> {
                // Reference: https://github.com/espressif/esp-idf/blob/733fbd9ecc8ac0780de51b3761a16d1faec63644/examples/peripherals/mcpwm/mcpwm_servo_control/main/mcpwm_servo_control_example_main.c
                // This gives us 4 pwm outputs on the esp32:
                // Unit 0 generator A
                // Unit 0 generator B
                // Unit 1 generator A
                // Unit 1 generator B
                let unit = channel.0;
                let generator = channel.1;
                let gpio_pin = self.pin();
                let duty_mode = mcpwm_duty_type_t_MCPWM_DUTY_MODE_0;
                let timer = mcpwm_timer_t_MCPWM_TIMER_0;
                #[allow(non_upper_case_globals)]
                let pwm_pin = match generator {
                    mcpwm_generator_t_MCPWM_GEN_A => mcpwm_io_signals_t_MCPWM0A,
                    mcpwm_generator_t_MCPWM_GEN_B => mcpwm_io_signals_t_MCPWM0B,
                    _ => panic!("Unexpected generator"),
                };
                let pwm_config = mcpwm_config_t {
                    frequency: 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
                    cmpr_a: 0.,    // duty cycle of PWMxA = 0
                    cmpr_b: 0.,
                    counter_mode: mcpwm_counter_type_t_MCPWM_UP_COUNTER,
                    duty_mode,
                };
                esp!(unsafe { mcpwm_gpio_init(unit, pwm_pin, gpio_pin) })?;
                esp!(unsafe { mcpwm_init(unit, timer, &pwm_config) })?;
                Ok(McPwmPin {
                    unit,
                    timer,
                    generator,
                    _pin: $pxi {
                        _mode: PhantomData::<McPwm>,
                    }
                    .degrade(),
                })
            }

            pub fn into_ledcpwm(self, channel: LedcPwmChannel) -> Result<LedcPwmPin, EspError> {
                // Reference:
                // https://github.com/espressif/esp-idf/blob/6cb6087dd0170cd534d8ad68ee2f8740a33368f9/examples/peripherals/ledc/ledc_basic/main/ledc_basic_example_main.c
                // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-ledc.c
                let channel = channel.0;
                let speed_mode = ledc_mode_t_LEDC_LOW_SPEED_MODE;
                let ledc_timer = ledc_timer_config_t {
                    speed_mode,
                    timer_num: ledc_timer_t_LEDC_TIMER_0,
                    __bindgen_anon_1: ledc_timer_config_t__bindgen_ty_1 {
                        duty_resolution: ledc_timer_bit_t_LEDC_TIMER_13_BIT,
                    },
                    freq_hz: 5000, // Set output frequency at 5 kHz
                    clk_cfg: ledc_clk_cfg_t_LEDC_AUTO_CLK,
                };
                let ledc_channel = ledc_channel_config_t {
                    speed_mode,
                    channel,
                    timer_sel: ledc_timer_t_LEDC_TIMER_0,
                    intr_type: ledc_intr_type_t_LEDC_INTR_DISABLE,
                    gpio_num: self.pin(),
                    duty: 0, // Set duty to 0%
                    hpoint: 0,
                    flags: ledc_channel_config_t__bindgen_ty_1 {
                        ..Default::default()
                    },
                };
                esp!(unsafe { ledc_timer_config(&ledc_timer) })?;
                esp!(unsafe { ledc_channel_config(&ledc_channel) })?;
                Ok(LedcPwmPin {
                    channel,
                    speed_mode,
                    _pin: $pxi {
                        _mode: PhantomData::<LedcPwm>,
                    }
                    .degrade(),
                })
            }

            pub fn into_output(mut self) -> Result<$pxi<Output>, EspError> {
                self.set_output()?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_output_od(mut self) -> Result<$pxi<Output>, EspError> {
                self.set_output_od()?;

                Ok($pxi { _mode: PhantomData })
            }
        }
    };
}

macro_rules! impl_rtc {
    ($pxi:ident: $pin:expr, RTC: $rtc:expr) => {
        impl<MODE> RTCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn rtc_pin(&self) -> i32 {
                $rtc
            }
        }
    };

    ($pxi:ident: $pin:expr, NORTC: $rtc:expr) => {};
}

macro_rules! impl_adc {
    ($pxi:ident: $pin:expr, ADC1: $adc:expr) => {
        #[cfg(not(feature = "riscv-ulp-hal"))]
        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            pub fn into_analog_atten_0db(
                mut self,
            ) -> Result<$pxi<adc::Atten0dB<adc::ADC1>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc1_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_0) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_2p5db(
                mut self,
            ) -> Result<$pxi<adc::Atten2p5dB<adc::ADC1>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc1_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_2_5) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_6db(
                mut self,
            ) -> Result<$pxi<adc::Atten6dB<adc::ADC1>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc1_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_6) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_11db(
                mut self,
            ) -> Result<$pxi<adc::Atten11dB<adc::ADC1>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc1_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_11) })?;

                Ok($pxi { _mode: PhantomData })
            }
        }

        impl<MODE> ADCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn adc_unit(&self) -> adc_unit_t {
                adc_unit_t_ADC_UNIT_1
            }

            fn adc_channel(&self) -> adc_channel_t {
                $adc
            }
        }

        impl<AN> embedded_hal_0_2::adc::Channel<AN> for $pxi<AN>
        where
            AN: adc::Analog<adc::ADC1> + Send,
        {
            type ID = u8;

            fn channel() -> Self::ID {
                $adc
            }
        }

        impl<AN> embedded_hal::adc::nb::Channel<AN> for $pxi<AN>
        where
            AN: adc::Analog<adc::ADC1> + Send,
        {
            type ID = u8;

            fn channel(&self) -> Self::ID {
                $adc
            }
        }
    };

    ($pxi:ident: $pin:expr, ADC2: $adc:expr) => {
        #[cfg(not(feature = "riscv-ulp-hal"))]
        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            pub fn into_analog_atten_0db(
                mut self,
            ) -> Result<$pxi<adc::Atten0dB<adc::ADC2>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc2_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_0) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_2p5db(
                mut self,
            ) -> Result<$pxi<adc::Atten2p5dB<adc::ADC2>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc2_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_2_5) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_6db(
                mut self,
            ) -> Result<$pxi<adc::Atten6dB<adc::ADC2>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc2_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_6) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_11db(
                mut self,
            ) -> Result<$pxi<adc::Atten11dB<adc::ADC2>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc2_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_11) })?;

                Ok($pxi { _mode: PhantomData })
            }
        }

        impl<MODE> ADCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn adc_unit(&self) -> adc_unit_t {
                adc_unit_t_ADC_UNIT_2
            }

            fn adc_channel(&self) -> adc_channel_t {
                $adc
            }
        }

        impl<AN> embedded_hal_0_2::adc::Channel<AN> for $pxi<AN>
        where
            AN: adc::Analog<adc::ADC2> + Send,
        {
            type ID = u8;

            fn channel() -> Self::ID {
                $adc as u8
            }
        }

        impl<AN> embedded_hal::adc::nb::Channel<AN> for $pxi<AN>
        where
            AN: adc::Analog<adc::ADC2> + Send,
        {
            type ID = u8;

            fn channel(&self) -> Self::ID {
                adc_unit_t_ADC_UNIT_2 as u8
            }
        }
    };

    ($pxi:ident: $pin:expr, NOADC: $adc:expr) => {};
}

macro_rules! impl_dac {
    ($pxi:ident: $pin:expr, DAC: $dac:expr) => {
        #[cfg(all(not(esp32c3), not(esp32s3)))]
        impl<MODE> DACPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn dac_channel(&self) -> dac_channel_t {
                $dac
            }
        }
    };

    ($pxi:ident: $pin:expr, NODAC: $dac:expr) => {};
}

macro_rules! impl_touch {
    ($pxi:ident: $pin:expr, TOUCH: $touch:expr) => {
        #[cfg(not(esp32c3))]
        impl<MODE> TouchPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn touch_channel(&self) -> touch_pad_t {
                $touch
            }
        }
    };

    ($pxi:ident: $pin:expr, NOTOUCH: $touch:expr) => {};
}

macro_rules! impl_hal_input_pin {
    ($pxi:ident: $mode:ident) => {
        impl embedded_hal_0_2::digital::v2::InputPin for $pxi<$mode> {
            type Error = EspError;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(self.get_input_level())
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.get_input_level())
            }
        }

        impl embedded_hal::digital::blocking::InputPin for $pxi<$mode> {
            type Error = EspError;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(self.get_input_level())
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.get_input_level())
            }
        }
    };
}

macro_rules! impl_hal_output_pin {
    ($pxi:ident: $mode:ident) => {
        impl embedded_hal_0_2::digital::v2::OutputPin for $pxi<$mode> {
            type Error = EspError;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(true)
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(false)
            }
        }

        impl embedded_hal::digital::blocking::OutputPin for $pxi<$mode> {
            type Error = EspError;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(true)
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(false)
            }
        }

        impl embedded_hal::digital::blocking::StatefulOutputPin for $pxi<$mode> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                Ok(self.get_output_level())
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.get_output_level())
            }
        }

        impl embedded_hal_0_2::digital::v2::StatefulOutputPin for $pxi<$mode> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                Ok(self.get_output_level())
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.get_output_level())
            }
        }

        impl embedded_hal_0_2::digital::v2::ToggleableOutputPin for $pxi<$mode> {
            type Error = EspError;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(!self.get_output_level())
            }
        }

        impl embedded_hal::digital::blocking::ToggleableOutputPin for $pxi<$mode> {
            type Error = EspError;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(!self.get_output_level())
            }
        }
    };
}

macro_rules! pin {
    ($pxi:ident: $pin:expr, Input, $rtc:ident: $rtcno:expr, $adc:ident: $adcno:expr, $dac:ident: $dacno:expr, $touch:ident: $touchno:expr) => {
        impl_input_only!($pxi: $pin);
        impl_rtc!($pxi: $pin, $rtc: $rtcno);
        impl_adc!($pxi: $pin, $adc: $adcno);
        impl_dac!($pxi: $pin, $dac: $dacno);
        impl_touch!($pxi: $pin, $touch: $touchno);
    };

    ($pxi:ident: $pin:expr, IO, $rtc:ident: $rtcno:expr, $adc:ident: $adcno:expr, $dac:ident: $dacno:expr, $touch:ident: $touchno:expr) => {
        impl_input_output!($pxi: $pin);
        impl_rtc!($pxi: $pin, $rtc: $rtcno);
        impl_adc!($pxi: $pin, $adc: $adcno);
        impl_dac!($pxi: $pin, $dac: $dacno);
        impl_touch!($pxi: $pin, $touch: $touchno);
    };
}

/// Generic $GpioX pin
pub struct GpioPin<MODE> {
    pin: i32,
    _mode: PhantomData<MODE>,
}

impl<MODE> GpioPin<MODE>
where
    MODE: Send,
{
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Pin, if it is
    /// already instantiated and used elsewhere, or if it is not set
    /// already in the mode of operation which is being instantiated
    pub unsafe fn new(pin: i32) -> GpioPin<MODE> {
        Self {
            pin,
            _mode: PhantomData,
        }
    }
}

impl<MODE> Pin for GpioPin<MODE>
where
    MODE: Send,
{
    type Error = EspError;

    fn pin(&self) -> i32
    where
        Self: Sized,
    {
        self.pin
    }
}

impl InputPin for GpioPin<Input> {}

impl OutputPin for GpioPin<Output> {}

impl InputPin for GpioPin<InputOutput> {}

impl OutputPin for GpioPin<InputOutput> {}

impl_base!(GpioPin);
impl_hal_input_pin!(GpioPin: Input);
impl_hal_input_pin!(GpioPin: InputOutput);
impl_hal_output_pin!(GpioPin: InputOutput);
impl_hal_output_pin!(GpioPin: Output);

#[cfg(esp32)]
mod chip {
    use core::marker::PhantomData;

    #[cfg(not(feature = "riscv-ulp-hal"))]
    use esp_idf_sys::*;

    use super::*;

    #[cfg(feature = "riscv-ulp-hal")]
    use crate::riscv_ulp_hal::sys::*;

    // NOTE: Gpio26 - Gpio32 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0, IO, RTC:11, ADC2:1, NODAC:0, TOUCH:1);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio1:1, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2, IO, RTC:12, ADC2:2, NODAC:0, TOUCH:2);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio3:3, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4, IO, RTC:10, ADC2:0, NODAC:0, TOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio5:5, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio6:6, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio7:7, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio8:8, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio9:9, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, RTC:15, ADC2:5, NODAC:0, TOUCH:5);
    pin!(Gpio13:13, IO, RTC:14, ADC2:4, NODAC:0, TOUCH:4);
    pin!(Gpio14:14, IO, RTC:16, ADC2:6, NODAC:0, TOUCH:6);
    pin!(Gpio15:15, IO, RTC:13, ADC2:3, NODAC:0, TOUCH:3);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio21:21, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio22:22, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio23:23, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio25:25, IO, RTC:6, ADC2:8, DAC:1, NOTOUCH:0);
    pin!(Gpio26:26, IO, RTC:7, ADC2:9, DAC:2, NOTOUCH:0);
    pin!(Gpio27:27, IO, RTC:17, ADC2:7, NODAC:0, TOUCH:7);
    pin!(Gpio32:32, IO, RTC:9, ADC1:4, NODAC:0, TOUCH:9);
    pin!(Gpio33:33, IO, RTC:8, ADC1:5, NODAC:0, TOUCH:8);
    pin!(Gpio34:34, Input, RTC:4, ADC1:6, NODAC:0, NOTOUCH:0);
    pin!(Gpio35:35, Input, RTC:5, ADC1:7, NODAC:0, NOTOUCH:0);
    pin!(Gpio36:36, Input, RTC:0, ADC1:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio37:37, Input, RTC:1, ADC1:1, NODAC:0, NOTOUCH:0);
    pin!(Gpio38:38, Input, RTC:2, ADC1:2, NODAC:0, NOTOUCH:0);
    pin!(Gpio39:39, Input, RTC:3, ADC1:3, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio1: Gpio1<Unknown>,
        pub gpio2: Gpio2<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio3: Gpio3<Unknown>,
        pub gpio4: Gpio4<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio5: Gpio5<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio6: Gpio6<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio7: Gpio7<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio8: Gpio8<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio9: Gpio9<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio10: Gpio10<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio11: Gpio11<Unknown>,
        pub gpio12: Gpio12<Unknown>,
        pub gpio13: Gpio13<Unknown>,
        pub gpio14: Gpio14<Unknown>,
        pub gpio15: Gpio15<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio16: Gpio16<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio17: Gpio17<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio18: Gpio18<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio19: Gpio19<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio21: Gpio21<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio22: Gpio22<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio23: Gpio23<Unknown>,
        pub gpio25: Gpio25<Unknown>,
        pub gpio26: Gpio26<Unknown>,
        pub gpio27: Gpio27<Unknown>,
        pub gpio32: Gpio32<Unknown>,
        pub gpio33: Gpio33<Unknown>,
        pub gpio34: Gpio34<Unknown>,
        pub gpio35: Gpio35<Unknown>,
        pub gpio36: Gpio36<Unknown>,
        pub gpio37: Gpio37<Unknown>,
        pub gpio38: Gpio38<Unknown>,
        pub gpio39: Gpio39<Unknown>,
        pub mcpwm_unit_0_gen_a: McPwmChannel,
        pub mcpwm_unit_0_gen_b: McPwmChannel,
        pub mcpwm_unit_1_gen_a: McPwmChannel,
        pub mcpwm_unit_1_gen_b: McPwmChannel,
        pub ledc_channel_0: LedcPwmChannel,
        pub ledc_channel_1: LedcPwmChannel,
        pub ledc_channel_2: LedcPwmChannel,
        pub ledc_channel_3: LedcPwmChannel,
        pub ledc_channel_4: LedcPwmChannel,
        pub ledc_channel_5: LedcPwmChannel,
        pub ledc_channel_6: LedcPwmChannel,
        pub ledc_channel_7: LedcPwmChannel,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio1: Gpio1::<Unknown>::new(),
                gpio2: Gpio2::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio3: Gpio3::<Unknown>::new(),
                gpio4: Gpio4::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio5: Gpio5::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio6: Gpio6::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio7: Gpio7::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio8: Gpio8::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio9: Gpio9::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio10: Gpio10::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio11: Gpio11::<Unknown>::new(),
                gpio12: Gpio12::<Unknown>::new(),
                gpio13: Gpio13::<Unknown>::new(),
                gpio14: Gpio14::<Unknown>::new(),
                gpio15: Gpio15::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio16: Gpio16::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio17: Gpio17::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio18: Gpio18::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio19: Gpio19::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio21: Gpio21::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio22: Gpio22::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio23: Gpio23::<Unknown>::new(),
                gpio25: Gpio25::<Unknown>::new(),
                gpio26: Gpio26::<Unknown>::new(),
                gpio27: Gpio27::<Unknown>::new(),
                gpio32: Gpio32::<Unknown>::new(),
                gpio33: Gpio33::<Unknown>::new(),
                gpio34: Gpio34::<Unknown>::new(),
                gpio35: Gpio35::<Unknown>::new(),
                gpio36: Gpio36::<Unknown>::new(),
                gpio37: Gpio37::<Unknown>::new(),
                gpio38: Gpio38::<Unknown>::new(),
                gpio39: Gpio39::<Unknown>::new(),
                ledc_channel_0: LedcPwmChannel(ledc_channel_t_LEDC_CHANNEL_0),
                ledc_channel_1: LedcPwmChannel(ledc_channel_t_LEDC_CHANNEL_1),
                ledc_channel_2: LedcPwmChannel(ledc_channel_t_LEDC_CHANNEL_2),
                ledc_channel_3: LedcPwmChannel(ledc_channel_t_LEDC_CHANNEL_3),
                ledc_channel_4: LedcPwmChannel(ledc_channel_t_LEDC_CHANNEL_4),
                ledc_channel_5: LedcPwmChannel(ledc_channel_t_LEDC_CHANNEL_5),
                ledc_channel_6: LedcPwmChannel(ledc_channel_t_LEDC_CHANNEL_6),
                ledc_channel_7: LedcPwmChannel(ledc_channel_t_LEDC_CHANNEL_7),
                mcpwm_unit_0_gen_a: McPwmChannel(
                    mcpwm_unit_t_MCPWM_UNIT_0,
                    mcpwm_generator_t_MCPWM_GEN_A,
                ),
                mcpwm_unit_0_gen_b: McPwmChannel(
                    mcpwm_unit_t_MCPWM_UNIT_0,
                    mcpwm_generator_t_MCPWM_GEN_B,
                ),
                mcpwm_unit_1_gen_a: McPwmChannel(
                    mcpwm_unit_t_MCPWM_UNIT_1,
                    mcpwm_generator_t_MCPWM_GEN_A,
                ),
                mcpwm_unit_1_gen_b: McPwmChannel(
                    mcpwm_unit_t_MCPWM_UNIT_1,
                    mcpwm_generator_t_MCPWM_GEN_B,
                ),
            }
        }
    }
}

#[cfg(any(esp32s2, esp32s3))]
mod chip {
    use core::marker::PhantomData;

    #[cfg(not(feature = "riscv-ulp-hal"))]
    use esp_idf_sys::*;

    use super::*;

    // NOTE: Gpio26 - Gpio32 (and Gpio33 - Gpio37 if using Octal RAM/Flash) are used
    //       by SPI0/SPI1 for external PSRAM/SPI Flash and are not recommended for
    //       other uses
    pin!(Gpio0:0, IO, RTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1, IO, RTC:1, ADC1:0, NODAC:0, TOUCH:1);
    pin!(Gpio2:2, IO, RTC:2, ADC1:1, NODAC:0, TOUCH:2);
    pin!(Gpio3:3, IO, RTC:3, ADC1:2, NODAC:0, TOUCH:3);
    pin!(Gpio4:4, IO, RTC:4, ADC1:3, NODAC:0, TOUCH:4);
    pin!(Gpio5:5, IO, RTC:5, ADC1:4, NODAC:0, TOUCH:5);
    pin!(Gpio6:6, IO, RTC:6, ADC1:5, NODAC:0, TOUCH:6);
    pin!(Gpio7:7, IO, RTC:7, ADC1:6, NODAC:0, TOUCH:7);
    pin!(Gpio8:8, IO, RTC:8, ADC1:7, NODAC:0, TOUCH:8);
    pin!(Gpio9:9, IO, RTC:9, ADC1:8, NODAC:0, TOUCH:9);
    pin!(Gpio10:10, IO, RTC:10, ADC1:9, NODAC:0, TOUCH:10);
    pin!(Gpio11:11, IO, RTC:11, ADC2:0, NODAC:0, TOUCH:11);
    pin!(Gpio12:12, IO, RTC:12, ADC2:1, NODAC:0, TOUCH:12);
    pin!(Gpio13:13, IO, RTC:13, ADC2:2, NODAC:0, TOUCH:13);
    pin!(Gpio14:14, IO, RTC:14, ADC2:3, NODAC:0, TOUCH:14);
    pin!(Gpio15:15, IO, RTC:15, ADC2:4, NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, RTC:16, ADC2:5, NODAC:0, NOTOUCH:0);
    #[cfg(esp32s2)]
    pin!(Gpio17:17, IO, RTC:17, ADC2:6, DAC:1, NOTOUCH:0);
    #[cfg(esp32s3)]
    pin!(Gpio17:17, IO, RTC:17, ADC2:6, NODAC:0, NOTOUCH:0);
    #[cfg(esp32s2)]
    pin!(Gpio18:18, IO, RTC:18, ADC2:7, DAC:2, NOTOUCH:0);
    #[cfg(esp32s3)]
    pin!(Gpio18:18, IO, RTC:18, ADC2:7, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, RTC:19, ADC2:8, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, RTC:20, ADC2:9, NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO, RTC:21, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio26:26, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio27:27, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio28:28, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio29:29, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio30:30, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio31:31, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio32:32, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio33:33, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio34:34, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio35:35, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio36:36, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio37:37, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio38:38, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio39:39, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio40:40, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio41:41, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio42:42, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio43:43, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio44:44, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio45:45, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s2, not(feature = "riscv-ulp-hal")))]
    pin!(Gpio46:46, Input, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
    pin!(Gpio46:46, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
    pin!(Gpio47:47, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
    pin!(Gpio48:48, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0<Unknown>,
        pub gpio1: Gpio1<Unknown>,
        pub gpio2: Gpio2<Unknown>,
        pub gpio3: Gpio3<Unknown>,
        pub gpio4: Gpio4<Unknown>,
        pub gpio5: Gpio5<Unknown>,
        pub gpio6: Gpio6<Unknown>,
        pub gpio7: Gpio7<Unknown>,
        pub gpio8: Gpio8<Unknown>,
        pub gpio9: Gpio9<Unknown>,
        pub gpio10: Gpio10<Unknown>,
        pub gpio11: Gpio11<Unknown>,
        pub gpio12: Gpio12<Unknown>,
        pub gpio13: Gpio13<Unknown>,
        pub gpio14: Gpio14<Unknown>,
        pub gpio15: Gpio15<Unknown>,
        pub gpio16: Gpio16<Unknown>,
        pub gpio17: Gpio17<Unknown>,
        pub gpio18: Gpio18<Unknown>,
        pub gpio19: Gpio19<Unknown>,
        pub gpio20: Gpio20<Unknown>,
        pub gpio21: Gpio21<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio26: Gpio26<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio27: Gpio27<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio28: Gpio28<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio29: Gpio29<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio30: Gpio30<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio31: Gpio31<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio32: Gpio32<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio33: Gpio33<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio34: Gpio34<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio35: Gpio35<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio36: Gpio36<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio37: Gpio37<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio38: Gpio38<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio39: Gpio39<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio40: Gpio40<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio41: Gpio41<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio42: Gpio42<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio43: Gpio43<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio44: Gpio44<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio45: Gpio45<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio46: Gpio46<Unknown>,
        #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
        pub gpio47: Gpio47<Unknown>,
        #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
        pub gpio48: Gpio48<Unknown>,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::<Unknown>::new(),
                gpio1: Gpio1::<Unknown>::new(),
                gpio2: Gpio2::<Unknown>::new(),
                gpio3: Gpio3::<Unknown>::new(),
                gpio4: Gpio4::<Unknown>::new(),
                gpio5: Gpio5::<Unknown>::new(),
                gpio6: Gpio6::<Unknown>::new(),
                gpio7: Gpio7::<Unknown>::new(),
                gpio8: Gpio8::<Unknown>::new(),
                gpio9: Gpio9::<Unknown>::new(),
                gpio10: Gpio10::<Unknown>::new(),
                gpio11: Gpio11::<Unknown>::new(),
                gpio12: Gpio12::<Unknown>::new(),
                gpio13: Gpio13::<Unknown>::new(),
                gpio14: Gpio14::<Unknown>::new(),
                gpio15: Gpio15::<Unknown>::new(),
                gpio16: Gpio16::<Unknown>::new(),
                gpio17: Gpio17::<Unknown>::new(),
                gpio18: Gpio18::<Unknown>::new(),
                gpio19: Gpio19::<Unknown>::new(),
                gpio20: Gpio20::<Unknown>::new(),
                gpio21: Gpio21::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio26: Gpio26::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio27: Gpio27::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio28: Gpio28::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio29: Gpio29::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio30: Gpio30::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio31: Gpio31::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio32: Gpio32::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio33: Gpio33::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio34: Gpio34::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio35: Gpio35::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio36: Gpio36::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio37: Gpio37::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio38: Gpio38::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio39: Gpio39::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio40: Gpio40::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio41: Gpio41::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio42: Gpio42::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio43: Gpio43::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio44: Gpio44::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio45: Gpio45::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio46: Gpio46::<Unknown>::new(),
                #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
                gpio47: Gpio47::<Unknown>::new(),
                #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
                gpio48: Gpio48::<Unknown>::new(),
            }
        }
    }
}

#[cfg(esp32c3)]
#[cfg(not(feature = "riscv-ulp-hal"))]
mod chip {
    use core::marker::PhantomData;

    use esp_idf_sys::*;

    use super::*;

    // NOTE: Gpio12 - Gpio17 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0,   IO,   RTC:0,  ADC1:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1,   IO,   RTC:1,  ADC1:1, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2,   IO,   RTC:2,  ADC1:2, NODAC:0, NOTOUCH:0);
    pin!(Gpio3:3,   IO,   RTC:3,  ADC1:3, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4,   IO,   RTC:4,  ADC1:4, NODAC:0, NOTOUCH:0);
    pin!(Gpio5:5,   IO,   RTC:5,  ADC2:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio13:13, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio14:14, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio15:15, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0<Unknown>,
        pub gpio1: Gpio1<Unknown>,
        pub gpio2: Gpio2<Unknown>,
        pub gpio3: Gpio3<Unknown>,
        pub gpio4: Gpio4<Unknown>,
        pub gpio5: Gpio5<Unknown>,
        pub gpio6: Gpio6<Unknown>,
        pub gpio7: Gpio7<Unknown>,
        pub gpio8: Gpio8<Unknown>,
        pub gpio9: Gpio9<Unknown>,
        pub gpio10: Gpio10<Unknown>,
        pub gpio11: Gpio11<Unknown>,
        pub gpio12: Gpio12<Unknown>,
        pub gpio13: Gpio13<Unknown>,
        pub gpio14: Gpio14<Unknown>,
        pub gpio15: Gpio15<Unknown>,
        pub gpio16: Gpio16<Unknown>,
        pub gpio17: Gpio17<Unknown>,
        pub gpio18: Gpio18<Unknown>,
        pub gpio19: Gpio19<Unknown>,
        pub gpio20: Gpio20<Unknown>,
        pub gpio21: Gpio21<Unknown>,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::<Unknown>::new(),
                gpio1: Gpio1::<Unknown>::new(),
                gpio2: Gpio2::<Unknown>::new(),
                gpio3: Gpio3::<Unknown>::new(),
                gpio4: Gpio4::<Unknown>::new(),
                gpio5: Gpio5::<Unknown>::new(),
                gpio6: Gpio6::<Unknown>::new(),
                gpio7: Gpio7::<Unknown>::new(),
                gpio8: Gpio8::<Unknown>::new(),
                gpio9: Gpio9::<Unknown>::new(),
                gpio10: Gpio10::<Unknown>::new(),
                gpio11: Gpio11::<Unknown>::new(),
                gpio12: Gpio12::<Unknown>::new(),
                gpio13: Gpio13::<Unknown>::new(),
                gpio14: Gpio14::<Unknown>::new(),
                gpio15: Gpio15::<Unknown>::new(),
                gpio16: Gpio16::<Unknown>::new(),
                gpio17: Gpio17::<Unknown>::new(),
                gpio18: Gpio18::<Unknown>::new(),
                gpio19: Gpio19::<Unknown>::new(),
                gpio20: Gpio20::<Unknown>::new(),
                gpio21: Gpio21::<Unknown>::new(),
            }
        }
    }
}
