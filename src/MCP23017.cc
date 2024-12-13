#include <MCP23017.h>

#include <fqa.h>
#include <device.h>
#include <debug.h>

#ifndef mcp23017_member_guard
bool MCP23017::_id_set = false;
char MCP23017::_id[I2CIP_ID_SIZE] = { '\0' };
#define mcp23017_member_guard
#endif

using namespace I2CIP;

void MCP23017::loadID() {
  uint8_t idlen = strlen_P(i2cip_mcp23017_id_progmem);

  // #ifdef I2CIP_DEBUG_SERIAL
  //   DEBUG_DELAY();
  //   I2CIP_DEBUG_SERIAL.print(F("Loading MCP23017 ID PROGMEM to Static Array @0x"));
  //   I2CIP_DEBUG_SERIAL.print((uintptr_t)(MCP23017::_id), HEX);
  //   I2CIP_DEBUG_SERIAL.print(F(" ("));
  //   I2CIP_DEBUG_SERIAL.print(idlen+1);
  //   I2CIP_DEBUG_SERIAL.print(F(" bytes) '"));
  // #endif

  // Read in PROGMEM
  for (uint8_t k = 0; k < idlen; k++) {
    char c = pgm_read_byte_near(i2cip_mcp23017_id_progmem + k);
    // #ifdef I2CIP_DEBUG_SERIAL
    //   DEBUG_SERIAL.print(c);
    // #endif
    MCP23017::_id[k] = c;
  }

  // #ifdef I2CIP_DEBUG_SERIAL
  //   DEBUG_SERIAL.print(F("'\n"));
  //   DEBUG_DELAY();
  // #endif

  #ifdef I2CIP_DEBUG_SERIAL
    DEBUG_DELAY();
    I2CIP_DEBUG_SERIAL.print(F("MCP23017 ID Loaded: '"));
    I2CIP_DEBUG_SERIAL.print(MCP23017::_id);
    I2CIP_DEBUG_SERIAL.print(F("' @"));
    I2CIP_DEBUG_SERIAL.println((uintptr_t)(&MCP23017::_id[0]), HEX);
    DEBUG_DELAY();
  #endif

  MCP23017::_id[idlen] = '\0';
  MCP23017::_id_set = true;
}

// Handles ID pointer assignment too
// NEVER returns nullptr, unless out of memory
Device* MCP23017::mcp23017Factory(const i2cip_fqa_t& fqa, const i2cip_id_t& id) {
  if(MCP23017::_id_set != true || id == nullptr) {
    loadID();

    return (Device*)(new MCP23017(fqa, (id == nullptr ? _id : id)));
  }

  return (Device*)(new MCP23017(fqa, id));
}

Device* MCP23017::mcp23017Factory(const i2cip_fqa_t& fqa) { return mcp23017Factory(fqa, MCP23017::_id); }

MCP23017::MCP23017(const i2cip_fqa_t& fqa) : MCP23017(fqa, _id) { loadID(); }
MCP23017::MCP23017(const i2cip_fqa_t& fqa, const i2cip_id_t& id) : Device(fqa, id), IOInterface<i2cip_mcp23017_t, i2cip_mcp23017_bitmask_t, i2cip_mcp23017_t, i2cip_mcp23017_bitmask_t>((Device*)this), pinFactory(&MCP23017_Pin::mcpPinFactory) {
  #ifdef I2CIP_DEBUG_SERIAL
    DEBUG_DELAY();
    I2CIP_DEBUG_SERIAL.print(F("MCP23017 Constructed (ID '"));
    I2CIP_DEBUG_SERIAL.print(this->id);
    I2CIP_DEBUG_SERIAL.print(F("' @0x"));
    I2CIP_DEBUG_SERIAL.print((uintptr_t)&(this->id[0]), HEX);
    I2CIP_DEBUG_SERIAL.print(F("; FQA "));
    I2CIP_DEBUG_SERIAL.print(I2CIP_FQA_SEG_I2CBUS(fqa), HEX);
    I2CIP_DEBUG_SERIAL.print(F(":"));
    I2CIP_DEBUG_SERIAL.print(I2CIP_FQA_SEG_MODULE(fqa), HEX);
    I2CIP_DEBUG_SERIAL.print(F(":"));
    I2CIP_DEBUG_SERIAL.print(I2CIP_FQA_SEG_MUXBUS(fqa), HEX);
    I2CIP_DEBUG_SERIAL.print(F(":"));
    I2CIP_DEBUG_SERIAL.print(I2CIP_FQA_SEG_DEVADR(fqa), HEX);
    I2CIP_DEBUG_SERIAL.print(F(")\n"));
    DEBUG_DELAY();
  #endif
}

MCP23017::~MCP23017() {
  #ifdef I2CIP_DEBUG_SERIAL
    DEBUG_DELAY();
    I2CIP_DEBUG_SERIAL.print(F("MCP23017 Destructed "));
    I2CIP_DEBUG_SERIAL.print(I2CIP_FQA_SEG_I2CBUS(this->fqa), HEX);
    I2CIP_DEBUG_SERIAL.print(F(":"));
    I2CIP_DEBUG_SERIAL.print(I2CIP_FQA_SEG_MODULE(this->fqa), HEX);
    I2CIP_DEBUG_SERIAL.print(F(":"));
    I2CIP_DEBUG_SERIAL.print(I2CIP_FQA_SEG_MUXBUS(this->fqa), HEX);
    I2CIP_DEBUG_SERIAL.print(F(":"));
    I2CIP_DEBUG_SERIAL.print(I2CIP_FQA_SEG_DEVADR(this->fqa), HEX);
    I2CIP_DEBUG_SERIAL.print(F(" @0x"));
    I2CIP_DEBUG_SERIAL.println((uintptr_t)this, HEX);
    DEBUG_DELAY();
  #endif

  // Cleanup
  // if(this->getValue() != nullptr && this->getValue() != _failsafe) delete this->getValue();
}

i2cip_errorlevel_t MCP23017::get(i2cip_mcp23017_t& dest, const i2cip_mcp23017_bitmask_t& args) {
  #ifdef I2CIP_DEBUG_SERIAL
    DEBUG_DELAY();
    I2CIP_DEBUG_SERIAL.print(F("MCP23017 GET: MASK 0b"));
    I2CIP_DEBUG_SERIAL.print((uint8_t)(args >> 8), BIN);
    I2CIP_DEBUG_SERIAL.print(' ');
    I2CIP_DEBUG_SERIAL.print((uint8_t)args, BIN);
    I2CIP_DEBUG_SERIAL.print(F(" -> ["));
    uint8_t bits = sizeof(i2cip_mcp23017_bitmask_t) * 8;
    uint8_t cmask = 1; bool comma = false;
    for(uint8_t bit = 0; bit < bits; bit++) {
      if(args & cmask) {
        if(comma) { I2CIP_DEBUG_SERIAL.print(F(", ")); } else { comma = true; }
        I2CIP_DEBUG_SERIAL.print(bit > 7 ? F("B") : F("A"));
        I2CIP_DEBUG_SERIAL.print(bit % 8);
      }
      cmask << 1;
    }
    I2CIP_DEBUG_SERIAL.println(']');
    DEBUG_DELAY();
  #endif

  // IODIR: 1 = input, 0 = output

  // uint16_t iodir = 0; // All output; no read
  i2cip_mcp23017_bitmask_t iodir = getArgsA();

  i2cip_errorlevel_t errlev = getIODIR(iodir);
  I2CIP_ERR_BREAK(errlev);

  // Set selected zeroes to ones, leave everything else
  iodir |= args;

  // Write IODIR
  errlev = setIODIR(iodir);
  I2CIP_ERR_BREAK(errlev);

  // errlev = getIODIR(iodir); // Uncomment to verify
  // I2CIP_ERR_BREAK(errlev);

  // Read GPIO LSB Bank A
  uint8_t gpioa = 0; // Low
  errlev = readRegisterByte(MCP23XXX_GPIO, gpioa, false);
  I2CIP_ERR_BREAK(errlev);

  // Read GPIO MSB Bank B
  uint8_t gpiob = 0;
  errlev = readRegisterByte((uint8_t)(MCP23XXX_GPIO + I2CIP_MCP23017_BANKJUMP), gpiob, false);
  I2CIP_ERR_BREAK(errlev);

  dest = ((uint16_t)gpiob << 8) | gpioa;

  return errlev;
}

// #ifdef I2CIP_DEBUG_SERIAL
//   DEBUG_DELAY();
//   I2CIP_DEBUG_SERIAL.print(F("MCP23017 IODIR 0b"));
//   I2CIP_DEBUG_SERIAL.print(iodir, BIN);
//   I2CIP_DEBUG_SERIAL.print(F(" -> 0b"));
//   I2CIP_DEBUG_SERIAL.print((iodir & 0xFF), BIN);
//   #endif

i2cip_errorlevel_t MCP23017::getIODIR(i2cip_mcp23017_t& dest) {
  // Read IODIR LSB Bank A
  uint8_t reg = 0;
  i2cip_errorlevel_t errlev = readRegisterByte(MCP23XXX_IODIR, reg, false);
  I2CIP_ERR_BREAK(errlev);

  // Read IODIR MSB Bank B
  uint8_t reg2 = 0;
  errlev = readRegisterByte((uint8_t)(MCP23XXX_IODIR + I2CIP_MCP23017_BANKJUMP), reg2, false, false);
  I2CIP_ERR_BREAK(errlev);

  dest = (((uint16_t)reg2) << 8) + (uint16_t)reg;

  #ifdef I2CIP_DEBUG_SERIAL
    DEBUG_DELAY();
    I2CIP_DEBUG_SERIAL.print(F("MCP23017 IODIR 0b"));
    I2CIP_DEBUG_SERIAL.print((uint8_t)(dest >> 8), BIN);
    I2CIP_DEBUG_SERIAL.print(' ');
    I2CIP_DEBUG_SERIAL.print((uint8_t)dest, BIN);
    I2CIP_DEBUG_SERIAL.println();
    DEBUG_DELAY();
  #endif

  return errlev;
}

i2cip_errorlevel_t MCP23017::setIODIR(const i2cip_mcp23017_t& value) {
  // Write IODIR
  i2cip_errorlevel_t errlev = writeRegister(MCP23XXX_IODIR, (uint8_t)(value & 0xFF), false);
  I2CIP_ERR_BREAK(errlev);
  errlev = writeRegister((uint8_t)(MCP23XXX_IODIR + I2CIP_MCP23017_BANKJUMP), (uint8_t)((value >> 8) & 0xFF), false);

  return errlev;
}

i2cip_errorlevel_t MCP23017::set(const i2cip_mcp23017_t& value, const i2cip_mcp23017_bitmask_t& args) {
  // IODIR: 1 = input, 0 = output
  #ifdef I2CIP_DEBUG_SERIAL
    DEBUG_DELAY();
    I2CIP_DEBUG_SERIAL.print(F("MCP23017 SET: MASK 0b"));
    I2CIP_DEBUG_SERIAL.print((uint8_t)(args >> 8), BIN);
    I2CIP_DEBUG_SERIAL.print(' ');
    I2CIP_DEBUG_SERIAL.print((uint8_t)args, BIN);
    I2CIP_DEBUG_SERIAL.print(F(" -> {"));
    uint8_t bits = sizeof(i2cip_mcp23017_bitmask_t) * 8;
    i2cip_mcp23017_t tval = value; uint8_t cmask = 1; bool comma = false;
    for(uint8_t bit = 0; bit < bits; bit++) {
      if(args & cmask) {
        if(comma) { I2CIP_DEBUG_SERIAL.print(F(", ")); } else { comma = true; }
        I2CIP_DEBUG_SERIAL.print(bit > 7 ? F("B") : F("A"));
        I2CIP_DEBUG_SERIAL.print(bit % 8);
        I2CIP_DEBUG_SERIAL.print(F(": "));
        I2CIP_DEBUG_SERIAL.print(tval & 1 ? F("HIGH") : F("LOW"));
      }
      tval >> 1;
      cmask << 1;
    }
    I2CIP_DEBUG_SERIAL.println('}');
    DEBUG_DELAY();
  #endif

  // SPECIAL CASE: Default args == 0; Just ping
  // if(args == 0) {
  //   I2CIP_DEBUG_SERIAL.println(F("MCP23017 SETNOP PING"));
  //   return ping();
  // }

  // Get current IODIR
  // uint16_t iodir = ~0; // All input; no output
  uint16_t iodir = getArgsB(); // Last known bitmask

  i2cip_errorlevel_t errlev = getIODIR(iodir);
  I2CIP_ERR_BREAK(errlev);

  iodir &= ~args; // Set selected ones to zeroes, leave everything else

  errlev = setIODIR(iodir);
  I2CIP_ERR_BREAK(errlev);

  // errlev = getIODIR(iodir); // Uncomment to verify
  // I2CIP_ERR_BREAK(errlev);

  #ifdef I2CIP_MCP23017_USEPULLUPS
    // GPPU (100kOhm): 1 = input-pullup, 0 = none

    // Identical to IODIR (All inputs are pulled up)
    errlev = writeRegister(MCP23XXX_GPPU, (uint8_t)(iodir & 0xFF) & (args & 0xFF), false);
    I2CIP_ERR_BREAK(errlev);
    errlev = writeRegister((uint8_t)(MCP23XXX_GPPU + I2CIP_MCP23017_BANKJUMP), (uint8_t)((iodir >> 8) & 0xFF) & (args >> 8), false);
    I2CIP_ERR_BREAK(errlev);

  #else
    // ALL OFF
    errlev = writeRegister(MCP23XXX_GPPU, (uint8_t)0x00, false);
    I2CIP_ERR_BREAK(errlev);
    errlev = writeRegister((uint8_t)(MCP23XXX_GPPU + I2CIP_MCP23017_BANKJUMP), (uint8_t)0x00, false);
    I2CIP_ERR_BREAK(errlev);

  #endif

  // GPIO: 1 = high, 0 = low

  // Read GPIO LSB Bank A
  uint8_t gpioa = 0; // Low
  errlev = readRegisterByte(MCP23XXX_GPIO, gpioa, false, false);
  I2CIP_ERR_BREAK(errlev);

  // Read GPIO MSB Bank B
  uint8_t gpiob = 0;
  errlev = readRegisterByte((uint8_t)(MCP23XXX_GPIO + I2CIP_MCP23017_BANKJUMP), gpiob, false, false);
  I2CIP_ERR_BREAK(errlev);

  i2cip_mcp23017_t gpio = ((uint16_t)gpiob << 8) | gpioa;
  i2cip_mcp23017_t send = (gpio & ~args); // Set selected to zeroes, leave everything else
  send |= (value & args); // Set selected to value (arg-selected), leave everything else

  // Write GPIO
  errlev = writeRegister(MCP23XXX_GPIO, (uint8_t)(send & 0xFF), false);
  I2CIP_ERR_BREAK(errlev);
  errlev = writeRegister((uint8_t)(MCP23XXX_GPIO + I2CIP_MCP23017_BANKJUMP), (uint8_t)((send >> 8) & 0xFF), false);

  return errlev;
}

// Adafruit_BusIO_Register IODIR(i2c_dev, spi_dev, MCP23XXX_SPIREG,
//                                 getRegister(MCP23XXX_IODIR << 8 | (args)));
// Adafruit_BusIO_Register GPPU(i2c_dev, spi_dev, MCP23XXX_SPIREG,
//                               getRegister(MCP23XXX_GPPU, MCP_PORT(pin)));
// Adafruit_BusIO_RegisterBits dir_bit(&IODIR, 1, pin % 8);
// Adafruit_BusIO_RegisterBits pullup_bit(&GPPU, 1, pin % 8);

// dir_bit.write((mode == OUTPUT) ? 0 : 1);
// pullup_bit.write((mode == INPUT_PULLUP) ? 1 : 0);
