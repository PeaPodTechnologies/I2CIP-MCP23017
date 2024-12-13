#ifndef I2CIP_MCP23017_H_
#define I2CIP_MCP23017_H_

#include <Arduino.h>

#include <I2CIP.h>

// Registers
#define MCP23XXX_IODIR    (uint8_t)0x00   //!< I/O direction register
#define MCP23XXX_IPOL     (uint8_t)0x01    //!< Input polarity register
#define MCP23XXX_GPINTEN  (uint8_t)0x02 //!< Interrupt-on-change control register
#define MCP23XXX_DEFVAL   (uint8_t)0x03 //!< Default compare register for interrupt-on-change
#define MCP23XXX_INTCON   (uint8_t)0x04 //!< Interrupt control register
#define MCP23XXX_IOCON    (uint8_t)0x05  //!< Configuration register
#define MCP23XXX_GPPU     (uint8_t)0x06   //!< Pull-up resistor configuration register
#define MCP23XXX_INTF     (uint8_t)0x07   //!< Interrupt flag register
#define MCP23XXX_INTCAP   (uint8_t)0x08 //!< Interrupt capture register
#define MCP23XXX_GPIO     (uint8_t)0x09   //!< Port register
#define MCP23XXX_OLAT     (uint8_t)0x0A   //!< Output latch register

// #define I2CIP_MCP23017_SIZE     100   // MCP23017 size in bytes
#define I2CIP_MCP23017_ADDRESS     0x20    // MCP23017 address (0x20-0x27)
#define I2CIP_MCP23017_BANKJUMP     (uint8_t)0xA
// #define I2CIP_MCP23017_TIMEOUT  1000   // If we're going to crash on a module ping fail, we should wait a bit

#define I2CIP_MCP23017_ID       "MCP23017"
// #define I2CIP_MCP23017_USEPULLUPS true // Uncomment to enable pullups

// Future-Proofing ;)
// namespace ControlSystemsOS {
//   class Linker;
// }

const char i2cip_mcp23017_id_progmem[] PROGMEM = {I2CIP_MCP23017_ID};

using namespace I2CIP;

typedef uint16_t i2cip_mcp23017_t;
typedef uint16_t i2cip_mcp23017_bitmask_t;

class MCP23017_Pin; class MCP23017;

typedef enum { PIN_A0 = 0, PIN_A1, PIN_A2, PIN_A3, PIN_A4, PIN_A5, PIN_A6, PIN_A7, PIN_B0, PIN_B1, PIN_B2, PIN_B3, BPIN_4, PIN_B5, PIN_B6, PIN_B7 } i2cip_mcp23017_pinsel_t;
typedef MCP23017_Pin* (*i2cip_mcp23017_pinFactory)(MCP23017* that, i2cip_mcp23017_pinsel_t pin);

// Default/NOP behaviour: output all off (0, 0)

class MCP23017 : public Device, public IOInterface<i2cip_mcp23017_t, i2cip_mcp23017_bitmask_t, i2cip_mcp23017_t, i2cip_mcp23017_bitmask_t> {
    // friend Device* I2CIP::mcp23017Factory(const i2cip_fqa_t& fqa);
    // friend class ControlSystemsOS::Linker; // Future-Proofing ;)
    
  private:
    MCP23017(const i2cip_fqa_t& fqa);
    static bool _id_set;
    static char _id[]; // to be loaded from progmem

    const i2cip_mcp23017_bitmask_t _failsafe_args = 0x0; // NONE SELECTED
    const i2cip_mcp23017_t _failsafe = 0x0; // ALL OFF

    uint16_t readBuffer = 0x00;

    friend class MCP23017_Pin;
    const i2cip_mcp23017_pinFactory pinFactory;

  // protected:
    i2cip_errorlevel_t getIODIR(i2cip_mcp23017_t& dest);
    i2cip_errorlevel_t setIODIR(const i2cip_mcp23017_t& value);

    // i2cip_errorlevel_t getGPPU(i2cip_mcp23017_t& dest);
    // i2cip_errorlevel_t setGPPU(const i2cip_mcp23017_t& value);
  public:
    MCP23017(const i2cip_fqa_t& fqa, const i2cip_id_t& id);
    static void loadID(void);
    static Device* mcp23017Factory(const i2cip_fqa_t& fqa, const i2cip_id_t& id);
    static Device* mcp23017Factory(const i2cip_fqa_t& fqa);

    ~MCP23017();

    // i2cip_errorlevel_t readContents(uint8_t* dest, size_t& num_read, size_t max_read = I2CIP_MCP23017_SIZE);

    /**
     * Read a section from MCP23017.
     * @param dest Destination heap (pointer reassigned, not overwritten)
     * @param args Number of bytes to read
     **/
    i2cip_errorlevel_t get(i2cip_mcp23017_t& dest, const i2cip_mcp23017_bitmask_t& args) override;
  
    /**
     * Write to a section of MCP23017.
     * @param value Value to write (null-terminated)
     * @param args Number of bytes to write
     **/
    i2cip_errorlevel_t set(const i2cip_mcp23017_t& value, const i2cip_mcp23017_bitmask_t& args) override;

    void clearCache(void) override { setCache(0x0); }
    void resetFailsafe(void) override { setValue(_failsafe); }
    const i2cip_mcp23017_bitmask_t& getDefaultA(void) const override { return _failsafe_args; };
    const i2cip_mcp23017_bitmask_t& getDefaultB(void) const override { return _failsafe_args; };

    static const char* getStaticIDBuffer() { return MCP23017::_id_set ? MCP23017::_id : nullptr; }

    MCP23017_Pin* operator[](i2cip_mcp23017_pinsel_t pin) { return (pin >= PIN_A0 && pin <= PIN_B7) ? pinFactory(this, pin) : nullptr; }

    // static const char* getStaticIDBuffer() { return MCP23017::_id; }
};

// Easily edit and read one pin at a time

class MCP23017_Pin : public IOInterface<i2cip_state_pin_t, i2cip_mcp23017_pinsel_t, i2cip_state_pin_t, i2cip_mcp23017_pinsel_t> {
  private:
    MCP23017* const mcp;
    const i2cip_mcp23017_pinsel_t pin;
  public:
    MCP23017_Pin(MCP23017* that, i2cip_mcp23017_pinsel_t pin) : IOInterface<i2cip_state_pin_t, i2cip_mcp23017_pinsel_t, i2cip_state_pin_t, i2cip_mcp23017_pinsel_t>(nullptr), pin(pin), mcp(that) { }

    static MCP23017_Pin* mcpPinFactory(MCP23017* that, i2cip_mcp23017_pinsel_t pin) { return new MCP23017_Pin(that, pin); }

    i2cip_errorlevel_t get(i2cip_state_pin_t& dest, const i2cip_mcp23017_pinsel_t& args) override {
      i2cip_mcp23017_bitmask_t a = (mcp->getArgsA() & ~(1 << args)) | (1 << args);
      i2cip_errorlevel_t errlev = ((Device*)mcp)->get(&a);
      I2CIP_ERR_BREAK(errlev);
      i2cip_mcp23017_t cache = mcp->getCache();
      dest = (cache & a) ? PIN_ON : PIN_OFF;
      return errlev;
    }
    i2cip_errorlevel_t set(const i2cip_state_pin_t& value, const i2cip_mcp23017_pinsel_t& args) override {
      i2cip_mcp23017_bitmask_t s = (mcp->getValue() & ~(1 << args)) | (value ? (1 << args) : 0);
      i2cip_mcp23017_bitmask_t b = (mcp->getArgsB() & ~(1 << args)); // Set bit output (0)
      i2cip_errorlevel_t errlev = ((Device*)mcp)->set(&s, &b);
      return errlev;
    }

    void clearCache(void) override { setCache(PIN_UNDEF); }
    const i2cip_mcp23017_pinsel_t& getDefaultA(void) const override { return pin; }
    const i2cip_mcp23017_pinsel_t& getDefaultB(void) const override { return pin; }
};

#endif