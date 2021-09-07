#include "TMCStepper.h"

int8_t TMC2130Stepper::chain_length = 0;
uint32_t TMC2130Stepper::spi_speed = 16000000/8;

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS, int8_t link) :
  TMCStepper(RS),
  _pinCS(pinCS),
  link_index(link)
  {
    defaults();

    if (link > chain_length)
      chain_length = link;
  }

void TMC2130Stepper::defaults() {
  //MSLUT0_register.sr = ???;
  //MSLUT1_register.sr = ???;
  //MSLUT2_register.sr = ???;
  //MSLUT3_register.sr = ???;
  //MSLUT4_register.sr = ???;
  //MSLUT5_register.sr = ???;
  //MSLUT6_register.sr = ???;
  //MSLUT7_register.sr = ???;
  //MSLUTSTART_register.start_sin90 = 247;
  PWMCONF_register.sr = 0x00050480;
}

__attribute__((weak))
void TMC2130Stepper::setSPISpeed(uint32_t speed) {
  spi_speed = speed;
}

__attribute__((weak))
void TMC2130Stepper::switchCSpin(bool state) {
  digitalWrite(_pinCS, state);
}

__attribute__((weak))
void TMC2130Stepper::beginTransaction() {
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
}
__attribute__((weak))
void TMC2130Stepper::endTransaction() {
  SPI.endTransaction();
}

__attribute__((weak))
uint8_t TMC2130Stepper::transfer(const uint8_t data) {
  uint8_t out = 0;
  out = SPI.transfer(data);
  return out;
}

void TMC2130Stepper::transferEmptyBytes(const uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    transfer(0x00);
  }
}

__attribute__((weak))
uint32_t TMC2130Stepper::read(uint8_t addressByte) {
  uint32_t out = 0UL;
  int8_t i = 1;

  beginTransaction();
  switchCSpin(LOW);
  transfer(addressByte);
  // Clear SPI
  transferEmptyBytes(4);

  // Shift the written data to the correct driver in chain
  // Default link_index = -1 and no shifting happens
  while(i < link_index) {
    transferEmptyBytes(5);
    i++;
  }

  switchCSpin(HIGH);
  switchCSpin(LOW);

  // Shift data from target link into the last one...
  while(i < chain_length) {
    transferEmptyBytes(5);
    i++;
  }

  // ...and once more to MCU
  status_response = transfer(addressByte); // Send the address byte again
  out  = transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);
  out <<= 8;
  out |= transfer(0x00);

  endTransaction();
  switchCSpin(HIGH);
  return out;
}

__attribute__((weak))
void TMC2130Stepper::write(uint8_t addressByte, uint32_t config) {
  addressByte |= TMC_WRITE;
  int8_t i = 1;

  beginTransaction();
  switchCSpin(LOW);
  status_response = transfer(addressByte);
  transfer(config>>24);
  transfer(config>>16);
  transfer(config>>8);
  transfer(config);

  // Shift the written data to the correct driver in chain
  // Default link_index = -1 and no shifting happens
  while(i < link_index) {
    transferEmptyBytes(5);
    i++;
  }

  endTransaction();
  switchCSpin(HIGH);
}

void TMC2130Stepper::begin() {
  //set pins
  pinMode(_pinCS, OUTPUT);
  switchCSpin(HIGH);

  GCONF(GCONF_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  PWMCONF(PWMCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);

  toff(8); //off_time(8);
  tbl(1); //blank_time(24);
}

/**
 *  Helper functions
 */

bool TMC2130Stepper::isEnabled() { return !drv_enn_cfg6() && toff(); }

void TMC2130Stepper::push() {
  GCONF(GCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);
  TPOWERDOWN(TPOWERDOWN_register.sr);
  TPWMTHRS(TPWMTHRS_register.sr);
  TCOOLTHRS(TCOOLTHRS_register.sr);
  THIGH(THIGH_register.sr);
  XDIRECT(XDIRECT_register.sr);
  VDCMIN(VDCMIN_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  DCCTRL(DCCTRL_register.sr);
  PWMCONF(PWMCONF_register.sr);
  ENCM_CTRL(ENCM_CTRL_register.sr);
}

///////////////////////////////////////////////////////////////////////////////////////
// R: IOIN
uint32_t  TMC2130Stepper::IOIN()    { return read(IOIN_t::address); }
bool TMC2130Stepper::step()         { IOIN_t r{0}; r.sr = IOIN(); return r.step; }
bool TMC2130Stepper::dir()          { IOIN_t r{0}; r.sr = IOIN(); return r.dir; }
bool TMC2130Stepper::dcen_cfg4()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcen_cfg4; }
bool TMC2130Stepper::dcin_cfg5()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcin_cfg5; }
bool TMC2130Stepper::drv_enn_cfg6() { IOIN_t r{0}; r.sr = IOIN(); return r.drv_enn_cfg6; }
bool TMC2130Stepper::dco()          { IOIN_t r{0}; r.sr = IOIN(); return r.dco; }
uint8_t TMC2130Stepper::version()   { IOIN_t r{0}; r.sr = IOIN(); return r.version; }
///////////////////////////////////////////////////////////////////////////////////////
// W: TCOOLTHRS
uint32_t TMC2130Stepper::TCOOLTHRS() { return TCOOLTHRS_register.sr; }
void TMC2130Stepper::TCOOLTHRS(uint32_t input) {
  TCOOLTHRS_register.sr = input;
  write(TCOOLTHRS_register.address, TCOOLTHRS_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: THIGH
uint32_t TMC2130Stepper::THIGH() { return THIGH_register.sr; }
void TMC2130Stepper::THIGH(uint32_t input) {
  THIGH_register.sr = input;
  write(THIGH_register.address, THIGH_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// RW: XDIRECT
uint32_t TMC2130Stepper::XDIRECT() {
  return read(XDIRECT_register.address);
}
void TMC2130Stepper::XDIRECT(uint32_t input) {
  XDIRECT_register.sr = input;
  write(XDIRECT_register.address, XDIRECT_register.sr);
}
void TMC2130Stepper::coil_A(int16_t B)  { XDIRECT_register.coil_A = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
void TMC2130Stepper::coil_B(int16_t B)  { XDIRECT_register.coil_B = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
int16_t TMC2130Stepper::coil_A()        { XDIRECT_t r{0}; r.sr = XDIRECT(); return r.coil_A; }
int16_t TMC2130Stepper::coil_B()        { XDIRECT_t r{0}; r.sr = XDIRECT(); return r.coil_B; }
///////////////////////////////////////////////////////////////////////////////////////
// W: VDCMIN
uint32_t TMC2130Stepper::VDCMIN() { return VDCMIN_register.sr; }
void TMC2130Stepper::VDCMIN(uint32_t input) {
  VDCMIN_register.sr = input;
  write(VDCMIN_register.address, VDCMIN_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// RW: DCCTRL
void TMC2130Stepper::DCCTRL(uint32_t input) {
	DCCTRL_register.sr = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}
void TMC2130Stepper::dc_time(uint16_t input) {
	DCCTRL_register.dc_time = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}
void TMC2130Stepper::dc_sg(uint8_t input) {
	DCCTRL_register.dc_sg = input;
	write(DCCTRL_register.address, DCCTRL_register.sr);
}

uint32_t TMC2130Stepper::DCCTRL() {
	return read(DCCTRL_register.address);
}
uint16_t TMC2130Stepper::dc_time() {
	DCCTRL_t r{0};
  r.sr = DCCTRL();
	return r.dc_time;
}
uint8_t TMC2130Stepper::dc_sg() {
	DCCTRL_t r{0};
  r.sr = DCCTRL();
	return r.dc_sg;
}
///////////////////////////////////////////////////////////////////////////////////////
// R: PWM_SCALE
uint8_t TMC2130Stepper::PWM_SCALE() { return read(PWM_SCALE_t::address); }
///////////////////////////////////////////////////////////////////////////////////////
// W: ENCM_CTRL
uint8_t TMC2130Stepper::ENCM_CTRL() { return ENCM_CTRL_register.sr; }
void TMC2130Stepper::ENCM_CTRL(uint8_t input) {
  ENCM_CTRL_register.sr = input;
  write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr);
}
void TMC2130Stepper::inv(bool B)      { ENCM_CTRL_register.inv = B;       write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
void TMC2130Stepper::maxspeed(bool B) { ENCM_CTRL_register.maxspeed  = B; write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
bool TMC2130Stepper::inv()            { return ENCM_CTRL_register.inv; }
bool TMC2130Stepper::maxspeed()       { return ENCM_CTRL_register.maxspeed; }
///////////////////////////////////////////////////////////////////////////////////////
// R: LOST_STEPS
uint32_t TMC2130Stepper::LOST_STEPS() { return read(LOST_STEPS_t::address); }

void TMC2130Stepper::sg_current_decrease(uint8_t value) {
  switch(value) {
    case 32: sedn(0b00); break;
    case  8: sedn(0b01); break;
    case  2: sedn(0b10); break;
    case  1: sedn(0b11); break;
  }
}
uint8_t TMC2130Stepper::sg_current_decrease() {
  switch(sedn()) {
    case 0b00: return 32;
    case 0b01: return  8;
    case 0b10: return  2;
    case 0b11: return  1;
  }
  return 0;
}


// CHOPCONF
#define SET_REG_CHOPCONF(SETTING) CHOPCONF_register.SETTING = B; write(CHOPCONF_register.address, CHOPCONF_register.sr)
uint32_t TMC2130Stepper::CHOPCONF() {
	return read(CHOPCONF_register.address);
}
void TMC2130Stepper::CHOPCONF(uint32_t input) {
	CHOPCONF_register.sr = input;
	write(CHOPCONF_register.address, CHOPCONF_register.sr);
}

void TMC2130Stepper::toff(		uint8_t B )	{ SET_REG_CHOPCONF(toff);	}
void TMC2130Stepper::hstrt(		uint8_t B )	{ SET_REG_CHOPCONF(hstrt);	}
void TMC2130Stepper::hend(		uint8_t B )	{ SET_REG_CHOPCONF(hend);	}
//void TMC2130Stepper::fd(		uint8_t B )	{ SET_REG_CHOPCONF(fd);		}
void TMC2130Stepper::disfdcc(	bool 	B )	{ SET_REG_CHOPCONF(disfdcc);	}
void TMC2130Stepper::rndtf(		bool 	B )	{ SET_REG_CHOPCONF(rndtf);	}
void TMC2130Stepper::chm(		bool 	B )	{ SET_REG_CHOPCONF(chm);		}
void TMC2130Stepper::tbl(		uint8_t B )	{ SET_REG_CHOPCONF(tbl);		}
void TMC2130Stepper::vsense(	bool 	B )	{ SET_REG_CHOPCONF(vsense);	}
void TMC2130Stepper::vhighfs(	bool 	B )	{ SET_REG_CHOPCONF(vhighfs);	}
void TMC2130Stepper::vhighchm(	bool 	B )	{ SET_REG_CHOPCONF(vhighchm);}
void TMC2130Stepper::sync(		uint8_t B )	{ SET_REG_CHOPCONF(sync);	}
void TMC2130Stepper::mres(		uint8_t B )	{ SET_REG_CHOPCONF(mres);	}
void TMC2130Stepper::intpol(	bool 	B )	{ SET_REG_CHOPCONF(intpol);	}
void TMC2130Stepper::dedge(		bool 	B )	{ SET_REG_CHOPCONF(dedge);	}
void TMC2130Stepper::diss2g(	bool 	B )	{ SET_REG_CHOPCONF(diss2g);	}

uint8_t TMC2130Stepper::toff()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.toff;	}
uint8_t TMC2130Stepper::hstrt()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.hstrt;	}
uint8_t TMC2130Stepper::hend()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.hend;	}
//uint8_t TMC2130Stepper::fd()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.fd;		}
bool 	TMC2130Stepper::disfdcc()	{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.disfdcc;	}
bool 	TMC2130Stepper::rndtf()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.rndtf;	}
bool 	TMC2130Stepper::chm()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.chm;		}
uint8_t TMC2130Stepper::tbl()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.tbl;		}
bool 	TMC2130Stepper::vsense()	{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.vsense;	}
bool 	TMC2130Stepper::vhighfs()	{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.vhighfs;	}
bool 	TMC2130Stepper::vhighchm()	{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.vhighchm;}
uint8_t TMC2130Stepper::sync()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.sync;	}
uint8_t TMC2130Stepper::mres()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.mres;	}
bool 	TMC2130Stepper::intpol()	{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.intpol;	}
bool 	TMC2130Stepper::dedge()		{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.dedge;	}
bool 	TMC2130Stepper::diss2g()	{ CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.diss2g;	}


// COOLCONF
#define SET_REG_COOLCONF(SETTING) COOLCONF_register.SETTING = B; write(COOLCONF_register.address, COOLCONF_register.sr);
#define GET_REG_COOLCONF(SETTING) return COOLCONF_register.SETTING;
uint32_t TMC2130Stepper::COOLCONF() { return COOLCONF_register.sr; }
void TMC2130Stepper::COOLCONF(uint32_t input) {
	COOLCONF_register.sr = input;
	write(COOLCONF_register.address, COOLCONF_register.sr);
}

void TMC2130Stepper::semin(	uint8_t B )	{ SET_REG_COOLCONF(semin);	}
void TMC2130Stepper::seup(	uint8_t B )	{ SET_REG_COOLCONF(seup);	}
void TMC2130Stepper::semax(	uint8_t B )	{ SET_REG_COOLCONF(semax);	}
void TMC2130Stepper::sedn(	uint8_t B )	{ SET_REG_COOLCONF(sedn);	}
void TMC2130Stepper::seimin(bool 	B )	{ SET_REG_COOLCONF(seimin);	}
void TMC2130Stepper::sgt(	int8_t  B )	{ SET_REG_COOLCONF(sgt);		}
void TMC2130Stepper::sfilt(	bool 	B )	{ SET_REG_COOLCONF(sfilt);	}

uint8_t TMC2130Stepper::semin()	{ GET_REG_COOLCONF(semin);	}
uint8_t TMC2130Stepper::seup()	{ GET_REG_COOLCONF(seup);	}
uint8_t TMC2130Stepper::semax()	{ GET_REG_COOLCONF(semax);	}
uint8_t TMC2130Stepper::sedn()	{ GET_REG_COOLCONF(sedn);	}
bool 	TMC2130Stepper::seimin(){ GET_REG_COOLCONF(seimin);	}
bool 	TMC2130Stepper::sfilt()	{ GET_REG_COOLCONF(sfilt);	}

int8_t TMC2130Stepper::sgt() {
	// Two's complement in a 7bit value
	int8_t val = (COOLCONF_register.sgt &  0x40) << 1; // Isolate sign bit
	val |= COOLCONF_register.sgt & 0x7F;
	return val;
}


// DRV_STATUS
#define GET_REG_DRV_STATUS(NS, SETTING) NS::DRV_STATUS_t r{0}; r.sr = DRV_STATUS(); return r.SETTING

uint32_t TMC2130Stepper::DRV_STATUS() { return read(DRV_STATUS_t::address); }

uint16_t TMC2130Stepper::sg_result(){ GET_REG_DRV_STATUS(TMC2130_n, sg_result); 	}
bool TMC2130Stepper::fsactive()		{ GET_REG_DRV_STATUS(TMC2130_n, fsactive); 	}
uint8_t TMC2130Stepper::cs_actual()	{ GET_REG_DRV_STATUS(TMC2130_n, cs_actual); 	}
bool TMC2130Stepper::stallguard()	{ GET_REG_DRV_STATUS(TMC2130_n, stallGuard); 	}
bool TMC2130Stepper::ot()			{ GET_REG_DRV_STATUS(TMC2130_n, ot); 			}
bool TMC2130Stepper::otpw()			{ GET_REG_DRV_STATUS(TMC2130_n, otpw); 		}
bool TMC2130Stepper::s2ga()			{ GET_REG_DRV_STATUS(TMC2130_n, s2ga); 		}
bool TMC2130Stepper::s2gb()			{ GET_REG_DRV_STATUS(TMC2130_n, s2gb); 		}
bool TMC2130Stepper::ola()			{ GET_REG_DRV_STATUS(TMC2130_n, ola); 			}
bool TMC2130Stepper::olb()			{ GET_REG_DRV_STATUS(TMC2130_n, olb); 			}
bool TMC2130Stepper::stst()			{ GET_REG_DRV_STATUS(TMC2130_n, stst); 		}


// GCONF
#define SET_REG_GCONF(SETTING) GCONF_register.SETTING = B; write(GCONF_register.address, GCONF_register.sr)
uint32_t TMC2130Stepper::GCONF() {
	return read(GCONF_register.address);
}
void TMC2130Stepper::GCONF(uint32_t input) {
	GCONF_register.sr = input;
	write(GCONF_register.address, GCONF_register.sr);
}

void TMC2130Stepper::I_scale_analog(bool B)			{ SET_REG_GCONF(i_scale_analog);			}
void TMC2130Stepper::internal_Rsense(bool B)		{ SET_REG_GCONF(internal_rsense);			}
void TMC2130Stepper::en_pwm_mode(bool B)			{ SET_REG_GCONF(en_pwm_mode);				}
void TMC2130Stepper::enc_commutation(bool B)		{ SET_REG_GCONF(enc_commutation);			}
void TMC2130Stepper::shaft(bool B) 					{ SET_REG_GCONF(shaft);					}
void TMC2130Stepper::diag0_error(bool B) 			{ SET_REG_GCONF(diag0_error);				}
void TMC2130Stepper::diag0_otpw(bool B) 			{ SET_REG_GCONF(diag0_otpw);				}
void TMC2130Stepper::diag0_stall(bool B) 			{ SET_REG_GCONF(diag0_stall);				}
void TMC2130Stepper::diag1_stall(bool B) 			{ SET_REG_GCONF(diag1_stall);				}
void TMC2130Stepper::diag1_index(bool B) 			{ SET_REG_GCONF(diag1_index);				}
void TMC2130Stepper::diag1_onstate(bool B) 			{ SET_REG_GCONF(diag1_onstate);			}
void TMC2130Stepper::diag1_steps_skipped(bool B) 	{ SET_REG_GCONF(diag1_steps_skipped);		}
void TMC2130Stepper::diag0_int_pushpull(bool B) 	{ SET_REG_GCONF(diag0_int_pushpull);		}
void TMC2130Stepper::diag1_pushpull(bool B) 		{ SET_REG_GCONF(diag1_poscomp_pushpull);	}
void TMC2130Stepper::small_hysteresis(bool B) 		{ SET_REG_GCONF(small_hysteresis);		}
void TMC2130Stepper::stop_enable(bool B) 			{ SET_REG_GCONF(stop_enable);				}
void TMC2130Stepper::direct_mode(bool B) 			{ SET_REG_GCONF(direct_mode);				}

bool TMC2130Stepper::I_scale_analog()				{ GCONF_t r{0}; r.sr = GCONF(); return r.i_scale_analog;		}
bool TMC2130Stepper::internal_Rsense()				{ GCONF_t r{0}; r.sr = GCONF(); return r.internal_rsense;		}
bool TMC2130Stepper::en_pwm_mode()					{ GCONF_t r{0}; r.sr = GCONF(); return r.en_pwm_mode;			}
bool TMC2130Stepper::enc_commutation()				{ GCONF_t r{0}; r.sr = GCONF(); return r.enc_commutation;		}
bool TMC2130Stepper::shaft() 						{ GCONF_t r{0}; r.sr = GCONF(); return r.shaft;					}
bool TMC2130Stepper::diag0_error() 					{ GCONF_t r{0}; r.sr = GCONF(); return r.diag0_error;			}
bool TMC2130Stepper::diag0_otpw() 					{ GCONF_t r{0}; r.sr = GCONF(); return r.diag0_otpw;			}
bool TMC2130Stepper::diag0_stall() 					{ GCONF_t r{0}; r.sr = GCONF(); return r.diag0_stall;			}
bool TMC2130Stepper::diag1_stall() 					{ GCONF_t r{0}; r.sr = GCONF(); return r.diag1_stall;			}
bool TMC2130Stepper::diag1_index() 					{ GCONF_t r{0}; r.sr = GCONF(); return r.diag1_index;			}
bool TMC2130Stepper::diag1_onstate() 				{ GCONF_t r{0}; r.sr = GCONF(); return r.diag1_onstate;			}
bool TMC2130Stepper::diag1_steps_skipped() 			{ GCONF_t r{0}; r.sr = GCONF(); return r.diag1_steps_skipped;	}
bool TMC2130Stepper::diag0_int_pushpull() 			{ GCONF_t r{0}; r.sr = GCONF(); return r.diag0_int_pushpull;	}
bool TMC2130Stepper::diag1_pushpull()		 		{ GCONF_t r{0}; r.sr = GCONF(); return r.diag1_poscomp_pushpull;}
bool TMC2130Stepper::small_hysteresis() 			{ GCONF_t r{0}; r.sr = GCONF(); return r.small_hysteresis;		}
bool TMC2130Stepper::stop_enable() 					{ GCONF_t r{0}; r.sr = GCONF(); return r.stop_enable;			}
bool TMC2130Stepper::direct_mode() 					{ GCONF_t r{0}; r.sr = GCONF(); return r.direct_mode;			}

/*
bit 18 not implemented:
test_mode 0:
Normal operation 1:
Enable analog test output on pin DCO. IHOLD[1..0] selects the function of DCO:
0…2: T120, DAC, VDDH Attention:
Not for user, set to 0 for normal operation!
*/


// IHOLD_IRUN
#define SET_REG_IHOLD_IRUN(SETTING) IHOLD_IRUN_register.SETTING = B; write(IHOLD_IRUN_register.address, IHOLD_IRUN_register.sr);
#define GET_REG_IHOLD_IRUN(SETTING) return IHOLD_IRUN_register.SETTING;
uint32_t TMCStepper::IHOLD_IRUN() { return IHOLD_IRUN_register.sr; }
void TMCStepper::IHOLD_IRUN(uint32_t input) {
	IHOLD_IRUN_register.sr = input;
	write(IHOLD_IRUN_register.address, IHOLD_IRUN_register.sr);
}

void 	TMCStepper::ihold(uint8_t B) 		{ SET_REG_IHOLD_IRUN(ihold);		}
void 	TMCStepper::irun(uint8_t B)  		{ SET_REG_IHOLD_IRUN(irun); 		}
void 	TMCStepper::iholddelay(uint8_t B)	{ SET_REG_IHOLD_IRUN(iholddelay); 	}

uint8_t TMCStepper::ihold() 				{ GET_REG_IHOLD_IRUN(ihold);		}
uint8_t TMCStepper::irun()  				{ GET_REG_IHOLD_IRUN(irun); 		}
uint8_t TMCStepper::iholddelay()  			{ GET_REG_IHOLD_IRUN(iholddelay);	}


// PWMCONF
#define SET_REG_PWMCONF(SETTING) PWMCONF_register.SETTING = B; write(PWMCONF_register.address, PWMCONF_register.sr)
#define GET_REG_PWMCONF(SETTING) return PWMCONF_register.SETTING
uint32_t TMC2130Stepper::PWMCONF() { return PWMCONF_register.sr; }
void TMC2130Stepper::PWMCONF(uint32_t input) {
	PWMCONF_register.sr = input;
	write(PWMCONF_register.address, PWMCONF_register.sr);
}

void TMC2130Stepper::pwm_ampl(		uint8_t B )	{ SET_REG_PWMCONF(pwm_ampl);		}
void TMC2130Stepper::pwm_grad(		uint8_t B )	{ SET_REG_PWMCONF(pwm_grad);		}
void TMC2130Stepper::pwm_freq(		uint8_t B )	{ SET_REG_PWMCONF(pwm_freq);		}
void TMC2130Stepper::pwm_autoscale(	bool 	B )	{ SET_REG_PWMCONF(pwm_autoscale);	}
void TMC2130Stepper::pwm_symmetric(	bool 	B )	{ SET_REG_PWMCONF(pwm_symmetric);	}
void TMC2130Stepper::freewheel(		uint8_t B )	{ SET_REG_PWMCONF(freewheel);		}

uint8_t TMC2130Stepper::pwm_ampl()		{ GET_REG_PWMCONF(pwm_ampl);		}
uint8_t TMC2130Stepper::pwm_grad()		{ GET_REG_PWMCONF(pwm_grad);		}
uint8_t TMC2130Stepper::pwm_freq()		{ GET_REG_PWMCONF(pwm_freq);		}
bool 	TMC2130Stepper::pwm_autoscale()	{ GET_REG_PWMCONF(pwm_autoscale);	}
bool 	TMC2130Stepper::pwm_symmetric()	{ GET_REG_PWMCONF(pwm_symmetric);	}
uint8_t TMC2130Stepper::freewheel()		{ GET_REG_PWMCONF(freewheel);		}
