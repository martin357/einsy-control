#pragma once
#pragma pack(push, 1)

union ad7150_reg_status{
  uint8_t reg : 8;
  struct{
    bool RDY1 : 1,
      RDY2 : 1,
      C1C2 : 1,
      OUT1 : 1,
      DacStep1 : 1,
      OUT2 : 1,
      DacStep2 : 1,
      PwrDown : 1;
  };
};


union ad7150_reg_setup{
  uint8_t reg = 0b00001011u;
  struct{
    uint8_t ThrSettling : 4;
    bool Hyst : 1,
       : 1,
      RngL : 1,
      RngH : 1;
  };
  struct{
    uint8_t : 6;
    uint8_t Rng : 2;
  };
};


union ad7150_reg_configuration{
  uint8_t reg = 0b00011001u;
  struct{
    bool MD0 : 1,
      MD1 : 1,
      MD2 : 1,
      EnCh2 : 1,
      EnCh1 : 1,
      ThrMD0 : 1,
      ThrMD1 : 1,
      ThrFixed : 1;
  };
  struct{
    uint8_t MD : 3,
     : 2;
    uint8_t ThrMD : 2,
     : 1;
  };
};


union ad7150_reg_capdac{
  uint8_t reg = 0b11000000u;
  struct{
    uint8_t DacValue : 6;
    bool DacAuto : 1,
      DacEn : 1;
  };
};
