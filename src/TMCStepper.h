#pragma once

//#define TMCDEBUG

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
	#include <SPI.h>
	#include <Stream.h>
#elif __cplusplus >= 201703L
	#if __has_include(<Arduino.h>)
		#include <Arduino.h>
	#endif
	#if __has_include(<SPI.h>)
		#include <SPI.h>
	#endif
	#if __has_include(<Stream.h>)
		#include <Stream.h>
	#endif
#endif


#pragma GCC diagnostic pop

#include "TMC2130_bitfields.h"

#define INIT_REGISTER(REG) REG##_t REG##_register = REG##_t
#define INIT2130_REGISTER(REG) TMC2130_n::REG##_t REG##_register = TMC2130_n::REG##_t
#define SET_ALIAS(TYPE, DRIVER, NEW, ARG, OLD) TYPE (DRIVER::*NEW)(ARG) = &DRIVER::OLD

#define TMCSTEPPER_VERSION 0x000703 // v0.7.3

struct MSCURACT_t {
  constexpr static uint8_t address = 0x6B;
  union {
    uint32_t sr : 25;
    struct {
      int16_t cur_a : 9,
                    : 7,
              cur_b : 9;
    };
  };
};

class TMCStepper {
	public:
		uint16_t cs2rms(uint8_t CS);
		void rms_current(uint16_t mA);
		void rms_current_hold(uint16_t mA);
		void rms_current(uint16_t mA, float mult);
		uint16_t rms_current();
		void hold_multiplier(float val) { holdMultiplier = val; }
		float hold_multiplier() { return holdMultiplier; }
		uint8_t test_connection();

		// Helper functions
		void microsteps(uint16_t ms);
		uint16_t microsteps();
		void blank_time(uint8_t value);
		uint8_t blank_time();
		void hysteresis_end(int8_t value);
		int8_t hysteresis_end();
		void hysteresis_start(uint8_t value);
		uint8_t hysteresis_start();

		// R+WC: GSTAT
		void 	GSTAT(							uint8_t input);
		uint8_t GSTAT();
		bool 	reset();
		bool 	drv_err();
		bool 	uv_cp();

		// W: IHOLD_IRUN
		void IHOLD_IRUN(					uint32_t input);
		uint32_t IHOLD_IRUN();
		void 	ihold(							uint8_t B);
		void 	irun(								uint8_t B);
		void 	iholddelay(					uint8_t B);
		uint8_t ihold();
		uint8_t irun();
		uint8_t iholddelay();

		// W: TPOWERDOWN
		uint8_t TPOWERDOWN();
		void TPOWERDOWN(					uint8_t input);

		// R: TSTEP
		uint32_t TSTEP();

		// W: TPWMTHRS
		uint32_t TPWMTHRS();
		void TPWMTHRS(						uint32_t input);

		// R: MSCNT
		uint16_t MSCNT();

		// R: MSCURACT
		uint32_t MSCURACT();
		int16_t cur_a();
		int16_t cur_b();

	protected:
		TMCStepper(float RS) : Rsense(RS) {};
		INIT_REGISTER(IHOLD_IRUN){{.sr=0}};	// 32b
		INIT_REGISTER(TPOWERDOWN){.sr=0};		// 8b
		INIT_REGISTER(TPWMTHRS){.sr=0};			// 32b

		static constexpr uint8_t TMC_READ = 0x00,
														TMC_WRITE = 0x80;

		struct TSTEP_t { constexpr static uint8_t address = 0x12; };
		struct MSCNT_t { constexpr static uint8_t address = 0x6A; };

		virtual void write(uint8_t, uint32_t) = 0;
		virtual uint32_t read(uint8_t) = 0;
		virtual void vsense(bool) = 0;
		virtual bool vsense(void) = 0;
		virtual uint32_t DRV_STATUS() = 0;
		virtual void hend(uint8_t) = 0;
		virtual uint8_t hend() = 0;
		virtual void hstrt(uint8_t) = 0;
		virtual uint8_t hstrt() = 0;
		virtual void mres(uint8_t) = 0;
		virtual uint8_t mres() = 0;
		virtual void tbl(uint8_t) = 0;
		virtual uint8_t tbl() = 0;

		const float Rsense;
		float holdMultiplier = 0.5;
};

class TMC2130Stepper : public TMCStepper {
	public:
		TMC2130Stepper(uint16_t pinCS, float RS = default_RS, int8_t link_index = -1);
		void begin();
		void defaults();
		void setSPISpeed(uint32_t speed);
		void switchCSpin(bool state);
		bool isEnabled();
		void push();

		// Helper functions
		void sg_current_decrease(uint8_t value);
		uint8_t sg_current_decrease();

		// RW: GCONF
		uint32_t GCONF();
		void GCONF(								uint32_t value);
		void I_scale_analog(			bool B);
		void internal_Rsense(			bool B);
		void en_pwm_mode(					bool B);
		void enc_commutation(			bool B);
		void shaft(								bool B);
		void diag0_error(					bool B);
		void diag0_otpw(					bool B);
		void diag0_stall(					bool B);
		void diag1_stall(					bool B);
		void diag1_index(					bool B);
		void diag1_onstate(				bool B);
		void diag1_steps_skipped(	bool B);
		void diag0_int_pushpull(	bool B);
		void diag1_pushpull(			bool B);
		void small_hysteresis(		bool B);
		void stop_enable(					bool B);
		void direct_mode(					bool B);
		bool I_scale_analog();
		bool internal_Rsense();
		bool en_pwm_mode();
		bool enc_commutation();
		bool shaft();
		bool diag0_error();
		bool diag0_otpw();
		bool diag0_stall();
		bool diag1_stall();
		bool diag1_index();
		bool diag1_onstate();
		bool diag1_steps_skipped();
		bool diag0_int_pushpull();
		bool diag1_pushpull();
		bool small_hysteresis();
		bool stop_enable();
		bool direct_mode();

		// R: IOIN
		uint32_t IOIN();
		bool step();
		bool dir();
		bool dcen_cfg4();
		bool dcin_cfg5();
		bool drv_enn_cfg6();
		bool dco();
		uint8_t version();

		// W: TCOOLTHRS
		uint32_t TCOOLTHRS();
		void TCOOLTHRS(						uint32_t input);

		// W: THIGH
		uint32_t THIGH();
		void THIGH(								uint32_t input);

		// RW: XDRIRECT
		uint32_t XDIRECT();
		void XDIRECT(							uint32_t input);
		void coil_A(							int16_t 	B);
		void coil_B(							int16_t 	B);
		int16_t coil_A();
		int16_t coil_B();

		// W: VDCMIN
		uint32_t VDCMIN();
		void VDCMIN(							uint32_t input);

		// RW: CHOPCONF
		uint32_t CHOPCONF();
		void CHOPCONF(						uint32_t value);
		void toff(								uint8_t B);
		void hstrt(								uint8_t B);
		void hend(								uint8_t B);
		//void fd(									uint8_t B);
		void disfdcc(							bool 		B);
		void rndtf(								bool 		B);
		void chm(									bool 		B);
		void tbl(									uint8_t B);
		void vsense(							bool 		B);
		void vhighfs(							bool 		B);
		void vhighchm(						bool 		B);
		void sync(								uint8_t B);
		void mres(								uint8_t B);
		void intpol(							bool 		B);
		void dedge(								bool 		B);
		void diss2g(							bool 		B);
		uint8_t toff();
		uint8_t hstrt();
		uint8_t hend();
		//uint8_t fd();
		bool 	disfdcc();
		bool 	rndtf();
		bool 	chm();
		uint8_t tbl();
		bool 	vsense();
		bool 	vhighfs();
		bool 	vhighchm();
		uint8_t sync();
		uint8_t mres();
		bool 	intpol();
		bool 	dedge();
		bool 	diss2g();

		// W: COOLCONF
		void COOLCONF(uint32_t value);
		uint32_t COOLCONF();
		void semin(								uint8_t B);
		void seup(								uint8_t B);
		void semax(								uint8_t B);
		void sedn(								uint8_t B);
		void seimin(							bool 		B);
		void sgt(									int8_t  B);
		void sfilt(								bool 		B);
		uint8_t semin();
		uint8_t seup();
		uint8_t semax();
		uint8_t sedn();
		bool seimin();
		int8_t sgt();
		bool sfilt();

		// W: DCCTRL
		void DCCTRL(uint32_t input);
		void dc_time(uint16_t input);
		void dc_sg(uint8_t input);
		uint32_t DCCTRL();
		uint16_t dc_time();
		uint8_t dc_sg();

		// R: DRV_STATUS
		uint32_t DRV_STATUS();
		uint16_t sg_result();
		bool fsactive();
		uint8_t cs_actual();
		bool stallguard();
		bool ot();
		bool otpw();
		bool s2ga();
		bool s2gb();
		bool ola();
		bool olb();
		bool stst();

		// W: PWMCONF
		void PWMCONF(							uint32_t value);
		uint32_t PWMCONF();
		void pwm_ampl(						uint8_t B);
		void pwm_grad(						uint8_t B);
		void pwm_freq(						uint8_t B);
		void pwm_autoscale(				bool		B);
		void pwm_symmetric(				bool		B);
		void freewheel(						uint8_t B);
		uint8_t pwm_ampl();
		uint8_t pwm_grad();
		uint8_t pwm_freq();
		bool 	pwm_autoscale();
		bool 	pwm_symmetric();
		uint8_t freewheel();

		// R: PWM_SCALE
		uint8_t PWM_SCALE();

		// W: ENCM_CTRL
		uint8_t ENCM_CTRL();
		void ENCM_CTRL(						uint8_t input);
		void inv(									bool B);
		void maxspeed(						bool B);
		bool inv();
		bool maxspeed();

		// R: LOST_STEPS
		uint32_t LOST_STEPS();

		// Function aliases

		uint8_t status_response;

	protected:
		void beginTransaction();
		void endTransaction();
		uint8_t transfer(const uint8_t data);
		void transferEmptyBytes(const uint8_t n);
		void write(uint8_t addressByte, uint32_t config);
		uint32_t read(uint8_t addressByte);

		INIT_REGISTER(GCONF){{.sr=0}};		// 32b
		INIT_REGISTER(TCOOLTHRS){.sr=0};	// 32b
		INIT_REGISTER(THIGH){.sr=0};			// 32b
		INIT_REGISTER(XDIRECT){{.sr=0}};	// 32b
		INIT_REGISTER(VDCMIN){.sr=0};			// 32b
		INIT_REGISTER(CHOPCONF){{.sr=0}};	// 32b
		INIT_REGISTER(COOLCONF){{.sr=0}};	// 32b
		INIT_REGISTER(DCCTRL){{.sr = 0}};	// 32b
		INIT_REGISTER(PWMCONF){{.sr=0}};	// 32b
		INIT_REGISTER(ENCM_CTRL){{.sr=0}};//  8b

		struct IOINT_t 		{ constexpr static uint8_t address = 0x04; };
		struct PWM_SCALE_t 	{ constexpr static uint8_t address = 0x71; };
		struct LOST_STEPS_t { constexpr static uint8_t address = 0x73; };
		struct DRV_STATUS_t { constexpr static uint8_t address = 0X6F; };

		static uint32_t spi_speed; // Default 2MHz
		const uint16_t _pinCS;
		static constexpr float default_RS = 0.11;

		int8_t link_index;
		static int8_t chain_length;
};
