#include "cw_letnany.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#ifdef CUSTOM_CW_LETNANY



// custom CW_LETNANY stuff
bool is_washing_pump_on(){ return digitalReadExt(PIN_WASHING_PUMP); }
void do_washing_pump_on(){ digitalWriteExt(PIN_WASHING_PUMP, HIGH); }
void do_washing_pump_off(){ digitalWriteExt(PIN_WASHING_PUMP, LOW); }
MenuItemToggleCallable item_washing_pump_on_off(&is_washing_pump_on, "Washing: on", "Washing: off", &do_washing_pump_off, &do_washing_pump_on);


bool is_heater_on(){ return digitalReadExt(PIN_HEATER); }
void do_heater_on(){ digitalWriteExt(PIN_HEATER, HIGH); }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, LOW); }
MenuItemToggleCallable item_heater_on_off(&is_heater_on, "Heater: on", "Heater: off", &do_heater_off, &do_heater_on);





#define OVERSAMPLENR 32 // 16
const short temptable_1[][2] PROGMEM = {
{       23*OVERSAMPLENR ,       300     },
{       25*OVERSAMPLENR ,       295     },
{       27*OVERSAMPLENR ,       290     },
{       28*OVERSAMPLENR ,       285     },
{       31*OVERSAMPLENR ,       280     },
{       33*OVERSAMPLENR ,       275     },
{       35*OVERSAMPLENR ,       270     },
{       38*OVERSAMPLENR ,       265     },
{       41*OVERSAMPLENR ,       260     },
{       44*OVERSAMPLENR ,       255     },
{       48*OVERSAMPLENR ,       250     },
{       52*OVERSAMPLENR ,       245     },
{       56*OVERSAMPLENR ,       240     },
{       61*OVERSAMPLENR ,       235     },
{       66*OVERSAMPLENR ,       230     },
{       71*OVERSAMPLENR ,       225     },
{       78*OVERSAMPLENR ,       220     },
{       84*OVERSAMPLENR ,       215     },
{       92*OVERSAMPLENR ,       210     },
{       100*OVERSAMPLENR        ,       205     },
{       109*OVERSAMPLENR        ,       200     },
{       120*OVERSAMPLENR        ,       195     },
{       131*OVERSAMPLENR        ,       190     },
{       143*OVERSAMPLENR        ,       185     },
{       156*OVERSAMPLENR        ,       180     },
{       171*OVERSAMPLENR        ,       175     },
{       187*OVERSAMPLENR        ,       170     },
{       205*OVERSAMPLENR        ,       165     },
{       224*OVERSAMPLENR        ,       160     },
{       245*OVERSAMPLENR        ,       155     },
{       268*OVERSAMPLENR        ,       150     },
{       293*OVERSAMPLENR        ,       145     },
{       320*OVERSAMPLENR        ,       140     },
{       348*OVERSAMPLENR        ,       135     },
{       379*OVERSAMPLENR        ,       130     },
{       411*OVERSAMPLENR        ,       125     },
{       445*OVERSAMPLENR        ,       120     },
{       480*OVERSAMPLENR        ,       115     },
{       516*OVERSAMPLENR        ,       110     },
{       553*OVERSAMPLENR        ,       105     },
{       591*OVERSAMPLENR        ,       100     },
{       628*OVERSAMPLENR        ,       95      },
{       665*OVERSAMPLENR        ,       90      },
{       702*OVERSAMPLENR        ,       85      },
{       737*OVERSAMPLENR        ,       80      },
{       770*OVERSAMPLENR        ,       75      },
{       801*OVERSAMPLENR        ,       70      },
{       830*OVERSAMPLENR        ,       65      },
{       857*OVERSAMPLENR        ,       60      },
{       881*OVERSAMPLENR        ,       55      },
{       903*OVERSAMPLENR        ,       50      },
{       922*OVERSAMPLENR        ,       45      },
{       939*OVERSAMPLENR        ,       40      },
{       954*OVERSAMPLENR        ,       35      },
{       966*OVERSAMPLENR        ,       30      },
{       977*OVERSAMPLENR        ,       25      },
{       985*OVERSAMPLENR        ,       20      },
{       993*OVERSAMPLENR        ,       15      },
{       999*OVERSAMPLENR        ,       10      },
{       1004*OVERSAMPLENR       ,       5       },
{       1008*OVERSAMPLENR       ,       0       } //safety
};

#define BEDTEMPTABLE temptable_1 // TT_NAME(1)
#define BEDTEMPTABLE_LEN (sizeof(BEDTEMPTABLE)/sizeof(*BEDTEMPTABLE))

#define BED_OFFSET 10
#define BED_OFFSET_START 40
#define BED_OFFSET_CENTER 50

#define PGM_RD_W(x)   (short)pgm_read_word(&x)

static float analog2tempBed(int raw) {
  float celsius = 0;
  byte i;

  for (i=1; i<BEDTEMPTABLE_LEN; i++)
  {
    if (PGM_RD_W(BEDTEMPTABLE[i][0]) > raw)
    {
      celsius  = PGM_RD_W(BEDTEMPTABLE[i-1][1]) +
        (raw - PGM_RD_W(BEDTEMPTABLE[i-1][0])) *
        (float)(PGM_RD_W(BEDTEMPTABLE[i][1]) - PGM_RD_W(BEDTEMPTABLE[i-1][1])) /
        (float)(PGM_RD_W(BEDTEMPTABLE[i][0]) - PGM_RD_W(BEDTEMPTABLE[i-1][0]));
      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(BEDTEMPTABLE[i-1][1]);

	// temperature offset adjustment
#ifdef BED_OFFSET
	float _offset = BED_OFFSET;
	float _offset_center = BED_OFFSET_CENTER;
	float _offset_start = BED_OFFSET_START;
	float _first_koef = (_offset / 2) / (_offset_center - _offset_start);
	float _second_koef = (_offset / 2) / (100 - _offset_center);

	if (celsius >= _offset_start && celsius <= _offset_center){
		celsius = celsius + (_first_koef * (celsius - _offset_start));
	}
	else if (celsius > _offset_center && celsius <= 100){
		celsius = celsius + (_first_koef * (_offset_center - _offset_start)) + ( _second_koef * ( celsius - ( 100 - _offset_center ) )) ;
	}
	else if (celsius > 100){
		celsius = celsius + _offset;
	}
#endif

  return celsius;
}


#define THERMISTOR_CNT 3

uint8_t analog_samples = 0;
uint16_t analog_sum[THERMISTOR_CNT] = {0};
double temp[THERMISTOR_CNT] = {0};


void loopCustom(){
  analog_sum[0] += analogRead(A0);
  analog_sum[1] += analogRead(A1);
  analog_sum[2] += analogRead(A2);
  if(++analog_samples >= OVERSAMPLENR){
    // for (size_t i = 0; i < THERMISTOR_CNT; i++) temp[i] = analog2tempBed(analog_sum[i]);
    for (size_t i = 0; i < THERMISTOR_CNT; i++){
      temp[i] = analog2tempBed(analog_sum[i]);
      analog_sum[i] = 0;

      // Serial.print(temp[i]);
      // Serial.print("°C\t");
    }
    // Serial.println();
    analog_samples = 0;
  }

}


uint16_t get_temp2(){
  return temp[2] * 100;
}
MenuItemDynamicCallable<uint16_t> item_temp2("Temp", &get_temp2);


// debug menu
MenuItem* debug_menu_items[] = {
  &back,
  &motor_x,
  &motor_z,
};
Menu debug_menu(debug_menu_items, sizeof(debug_menu_items) / 2);
MenuItem item_debug_menu("!!! Debug", &debug_menu);


// main menu
MenuItem* main_menu_items[] = {
  &item_washing_pump_on_off,
  &item_heater_on_off,
  &item_temp2,
  &item_debug_menu,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);




void setupCustom(){
  for (size_t i = 0; i < MOTORS_MAX; i++) {
    motors[i].driver.rms_current(500);
    motors[i].driver.sgt(2);

    motors[i].driver.en_pwm_mode(1);
    motors[i].driver.pwm_autoscale(1);
    motors[i].driver.intpol(1);

    motors[i].rpm(120);
  }

  // power output
  pinModeOutput(PIN_HEATER);
  digitalWriteExt(PIN_HEATER, LOW);
  pinModeOutput(PIN_WASHING_PUMP);
  digitalWriteExt(PIN_WASHING_PUMP, LOW);

  pinModeInput(A0);
  pinModeInput(A1);
  pinModeInput(A2);

  main_menu.redraw_interval = 50;

}


#endif
