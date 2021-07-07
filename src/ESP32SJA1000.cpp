// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifdef ARDUINO_ARCH_ESP32

#include "esp_intr.h"
#include "soc/dport_reg.h"
#include "driver/gpio.h"

#include "ESP32SJA1000.h"

#define REG_BASE                   0x3ff6b000

//All registers bellow are calculated by formula: Register add = (Add-REG_BASE)/4
//Address are indicated in page 555 of manual: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
#define REG_MOD                    0x00 //TWAI_MODE_REG
#define REG_CMR                    0x01 //TWAI_CMD_REG
#define REG_SR                     0x02 //TWAI_STATUS_REG
#define REG_IR                     0x03 //TWAI_INT_RAW_REG
#define REG_IER                    0x04 //TWAI_INT_ENA_REG New ESP revision 2 have brp_div bit. Search in this link for CONFIG_ESP32_REV_MIN: https://github.com/espressif/esp-idf/commit/03d5742e110d2d5a8fbdf60ad9fcf894d3f98eb5
#define  DRIVER_DEFAULT_INTERRUPTS 0xE7 //Exclude data overrun (bit[3]) and brp_div (bit[4])
#define  BRP_DIV_EN_BIT            0x10 //Bit mask for brp_div in the interrupt register: https://www.esp32.com/viewtopic.php?t=15581
#define REG_BTR0                   0x06 //TWAI_BUS_TIMING_0_REG
#define REG_BTR1                   0x07 //TWAI_BUS_TIMING_1_REG
#define REG_OCR                    0x08 //(By Henry warning) Is this address correct? I dont find 0x3FF6B020 register in the manual

#define REG_ALC                    0x0b //TWAI_ARB_LOST_CAP_REG
#define REG_ECC                    0x0c //TWAI_ERR_CODE_CAP_REG
#define REG_EWLR                   0x0d //TWAI_ERR_WARNING_LIMIT_REG
#define REG_RXERR                  0x0e //TWAI_RX_ERR_CNT_REG
#define REG_TXERR                  0x0f //TWAI_TX_ERR_CNT_REG
#define REG_SFF                    0x10 //TWAI_DATA_0_REG
#define REG_EFF                    0x10 //TWAI_DATA_0_REG (By Henry warning) Is this address correct? Is the same address as REG_SFF?
#define REG_ACRn(n)                (0x10 + n)
#define REG_AMRn(n)                (0x14 + n)

#define REG_CDR                    0x1F


ESP32SJA1000Class::ESP32SJA1000Class() :
  CANControllerClass(),
  _rxPin(DEFAULT_CAN_RX_PIN),
  _txPin(DEFAULT_CAN_TX_PIN),
  _loopback(false),
  _intrHandle(NULL)
{
}

ESP32SJA1000Class::~ESP32SJA1000Class()
{
}

int ESP32SJA1000Class::begin(long baudRate)
{
  CANControllerClass::begin(baudRate);

  _loopback = false;

  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);

  // RX pin
  gpio_set_direction(_rxPin, GPIO_MODE_INPUT);
  gpio_matrix_in(_rxPin, CAN_RX_IDX, 0);
  gpio_pad_select_gpio(_rxPin);

  // TX pin
  gpio_set_direction(_txPin, GPIO_MODE_OUTPUT);
  gpio_matrix_out(_txPin, CAN_TX_IDX, 0, 0);
  gpio_pad_select_gpio(_txPin);

  modifyRegister(REG_CDR, 0x80, 0x80); // pelican mode
  modifyRegister(REG_BTR0, 0xc0, 0x40); // SJW = 1
  modifyRegister(REG_BTR1, 0x70, 0x10); // TSEG2 = 1

  if (baudRate >= 50E3)
  {
    if (ESP.getChipRevision() >= 2)       // New ESP revision 2 have brp_div bit (CONFIG_ESP32_REV_MIN). It mantain this div disabled. If enabled baudRate configuration need to be multiplied by 2.
      modifyRegister(REG_IER, BRP_DIV_EN_BIT, 0x00);
    else {}                               //Do nothing, this bit is reserved on Rev 1 chips.

    switch (baudRate) {
    case (long)1000E3:
      modifyRegister(REG_BTR1, 0x0f, 0x04);
      modifyRegister(REG_BTR0, 0x3f, 4);
      break;

    case (long)500E3:
      modifyRegister(REG_BTR1, 0x0f, 0x0c);
      modifyRegister(REG_BTR0, 0x3f, 4);
      break;

    case (long)250E3:
      modifyRegister(REG_BTR1, 0x0f, 0x0c);
      modifyRegister(REG_BTR0, 0x3f, 9);
      break;

    case (long)200E3:
      modifyRegister(REG_BTR1, 0x0f, 0x0c);
      modifyRegister(REG_BTR0, 0x3f, 12);
      break;

    case (long)125E3:
      modifyRegister(REG_BTR1, 0x0f, 0x0c);
      modifyRegister(REG_BTR0, 0x3f, 19);
      break;

    case (long)100E3:
      modifyRegister(REG_BTR1, 0x0f, 0x0c);
      modifyRegister(REG_BTR0, 0x3f, 24);
      break;

    case (long)80E3:
      modifyRegister(REG_BTR1, 0x0f, 0x0c);
      modifyRegister(REG_BTR0, 0x3f, 30);
      break;

    case (long)50E3:
      modifyRegister(REG_BTR1, 0x0f, 0x0c);
      modifyRegister(REG_BTR0, 0x3f, 49);
      break;

    default:
      return 0;
      break;
    }//switch
  }
  else
  {
    if (ESP.getChipRevision() >= 2)       // New ESP revision 2 have brp_div bit (CONFIG_ESP32_REV_MIN) that can be used to divide by 2 the frequencies.
      modifyRegister(REG_IER, BRP_DIV_EN_BIT, BRP_DIV_EN_BIT);
    else  return 0;
    switch (baudRate)
    {
    case (long)40E3:
      modifyRegister(REG_BTR1, 0x0f, 0x0c);
      modifyRegister(REG_BTR0, 0x3f, 30);
      break;

    case (long)20E3:
      modifyRegister(REG_BTR1, 0x0f, 0x4d);
      modifyRegister(REG_BTR0, 0x3f, 30);
      break;

    default:
      return 0;
      break;
    }
  }

  modifyRegister(REG_BTR1, 0x80, 0x80); // SAM = 1
  modifyRegister(REG_IER, 0xef, DRIVER_DEFAULT_INTERRUPTS); // Enable some interrupts

  // set filter to allow anything
  writeRegister(REG_ACRn(0), 0x00);//id
  writeRegister(REG_ACRn(1), 0x00);
  writeRegister(REG_ACRn(2), 0x00);
  writeRegister(REG_ACRn(3), 0x00);
  writeRegister(REG_AMRn(0), 0xff);//mask
  writeRegister(REG_AMRn(1), 0xff);
  writeRegister(REG_AMRn(2), 0xff);
  writeRegister(REG_AMRn(3), 0xff);


  modifyRegister(REG_OCR, 0x03, 0x02); // normal output mode
  // reset error counters
  writeRegister(REG_TXERR, 0x00);
  writeRegister(REG_RXERR, 0x00);

  // clear errors and interrupts
  readRegister(REG_ECC);
  readRegister(REG_IR);

  // single filter mode
  modifyRegister(REG_MOD, 0x08, 0x08);//Single filter mode

  // normal mode
  modifyRegister(REG_MOD, 0x07, 0x00);//Self test mode off / Listen only mode off / Operating mode

  return 1;
}

void ESP32SJA1000Class::end()
{
  if (_intrHandle) {
    esp_intr_free(_intrHandle);
    _intrHandle = NULL;
  }

  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);

  CANControllerClass::end();
}

int ESP32SJA1000Class::endPacket()
{
  if (!CANControllerClass::endPacket()) {
    return 0;
  }

  // wait for TX buffer to free
  while ((readRegister(REG_SR) & 0x04) != 0x04) {
    yield();
  }

  int dataReg;

  if (_txExtended) {
    writeRegister(REG_EFF, 0x80 | (_txRtr ? 0x40 : 0x00) | (0x0f & _txLength));
    writeRegister(REG_EFF + 1, _txId >> 21);
    writeRegister(REG_EFF + 2, _txId >> 13);
    writeRegister(REG_EFF + 3, _txId >> 5);
    writeRegister(REG_EFF + 4, _txId << 3);

    dataReg = REG_EFF + 5;
  } else {
    writeRegister(REG_SFF, (_txRtr ? 0x40 : 0x00) | (0x0f & _txLength));
    writeRegister(REG_SFF + 1, _txId >> 3);
    writeRegister(REG_SFF + 2, _txId << 5);

    dataReg = REG_SFF + 3;
  }

  for (int i = 0; i < _txLength; i++) {
    writeRegister(dataReg + i, _txData[i]);
  }

  if ( _loopback) {
    // self reception request
    modifyRegister(REG_CMR, 0x10, 0x10); // Only bit TWAI_SELF_RX_REQ is modified.
  } else {
    // transmit request
    modifyRegister(REG_CMR, 0x01, 0x01); // Only bit TWAI_TX_REQ is modified.
  }

  // wait for TX complete
  while ((readRegister(REG_SR) & 0x08) != 0x08) {
    if (readRegister(REG_ECC) == 0xd9) {//(By Henry warning) I think that is not correct. 0xd9 is a very specific conjunction of errors codes. Look this register in the manual: TWAI_ERR_CODE_CAP_REG
      modifyRegister(REG_CMR, 0x02, 0x02); // Only bit TWAI_ABORT_TX is modified.
      return 0;
    }
    yield();
  }

  return 1;
}

int ESP32SJA1000Class::parsePacket()
{
  if ((readRegister(REG_SR) & 0x01) != 0x01) {
    // no packet
    return 0;
  }

  _rxExtended = (readRegister(REG_SFF) & 0x80) ? true : false;
  _rxRtr = (readRegister(REG_SFF) & 0x40) ? true : false;
  _rxDlc = (readRegister(REG_SFF) & 0x0f);
  _rxIndex = 0;

  int dataReg;

  if (_rxExtended) {
    _rxId = (readRegister(REG_EFF + 1) << 21) |
            (readRegister(REG_EFF + 2) << 13) |
            (readRegister(REG_EFF + 3) << 5) |
            (readRegister(REG_EFF + 4) >> 3);

    dataReg = REG_EFF + 5;
  } else {
    _rxId = (readRegister(REG_SFF + 1) << 3) | ((readRegister(REG_SFF + 2) >> 5) & 0x07);

    dataReg = REG_SFF + 3;
  }

  if (_rxRtr) {
    _rxLength = 0;
  } else {
    _rxLength = _rxDlc;

    if (_rxLength > 8)_rxLength = 8;  // Protect _rxData pointer overload
    for (int i = 0; i < _rxLength; i++) {
      _rxData[i] = readRegister(dataReg + i);
    }
  }

  // release RX buffer
  modifyRegister(REG_CMR, 0x04, 0x04);

  return _rxDlc;
}

void ESP32SJA1000Class::onReceive(void(*callback)(int))
{
  CANControllerClass::onReceive(callback);

  if (_intrHandle) {
    esp_intr_free(_intrHandle);
    _intrHandle = NULL;
  }

  if (callback) {
    esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, ESP32SJA1000Class::onInterrupt, this, &_intrHandle);
  }
}

int ESP32SJA1000Class::filter(int id, int mask)
{
  id &= 0x7ff;
  mask = ~(mask & 0x7ff);

  modifyRegister(REG_MOD, 0x01, 0x01); // Only reset mode bit is modified.

  writeRegister(REG_ACRn(0), id >> 3);
  writeRegister(REG_ACRn(1), id << 5);
  writeRegister(REG_ACRn(2), 0x00);
  writeRegister(REG_ACRn(3), 0x00);

  writeRegister(REG_AMRn(0), mask >> 3);
  writeRegister(REG_AMRn(1), (mask << 5) | 0x1f);
  writeRegister(REG_AMRn(2), 0xff);
  writeRegister(REG_AMRn(3), 0xff);

  modifyRegister(REG_MOD, 0x01, 0x00); // Only operating mode bit is modified.

  return 1;
}

int ESP32SJA1000Class::filterExtended(long id, long mask, bool rtrId, bool rtrMask)
{
  modifyRegister(REG_MOD, 0x01, 0x01); // reset mode

  uint32_t idFilter = id & 0x1FFFFFFF;        //'1' and '0' bits need to be exact as the received bit to be a valid ID.
  uint32_t maskFilter = mask & 0x1FFFFFFF;    //'1' bits admit all bits to be received. '0' bits admit bits to be received only if the idFilter bits equals the id input bit in CAN.
  uint8_t temp[4];

  idFilter <<= 3;
  if (rtrId) idFilter |= (1 << 2); //RTR in bit 2
  memcpy(temp, &idFilter, 4);
  writeRegister(REG_ACRn(0), temp[3]);//id
  writeRegister(REG_ACRn(1), temp[2]);
  writeRegister(REG_ACRn(2), temp[1]);
  writeRegister(REG_ACRn(3), temp[0]);

  maskFilter <<= 3;
  if (rtrMask) maskFilter |= (1 << 2); //RTR in bit 2
  memcpy(temp, &maskFilter, 4);
  writeRegister(REG_AMRn(0), temp[3]);//mask
  writeRegister(REG_AMRn(1), temp[2]);
  writeRegister(REG_AMRn(2), temp[1]);
  writeRegister(REG_AMRn(3), temp[0]);

  modifyRegister(REG_MOD, 0x01, 0x00); // (By Henry) Operating mode
  modifyRegister(REG_MOD, 0x08, 0x08); // (By Henry) Single filter mode TWAI_RX_FILTER_MODE

  return 1;
}

int ESP32SJA1000Class::observe()
{
  modifyRegister(REG_MOD, 0x01, 0x01); // Only reset mode bit is modified.
  modifyRegister(REG_MOD, 0x02, 0x02); // Only listen only mode bit is modified.
  modifyRegister(REG_MOD, 0x01, 0x00); // Only Operating mode bit is modified.

  return 1;
}

int ESP32SJA1000Class::loopback()
{
  _loopback = true;

  modifyRegister(REG_MOD, 0x01, 0x01); // Only reset mode bit is modified.
  modifyRegister(REG_MOD, 0x04, 0x04); // Only self test mode bit is modified.
  modifyRegister(REG_MOD, 0x01, 0x00); // Only Operating mode bit is modified.

  return 1;
}

int ESP32SJA1000Class::sleep()
{
  modifyRegister(REG_MOD, 0x1f, 0x10);
  //ATENTION: (By Henry warning) I dont find this flag in manual. The TWAI_MODE_REG have only bits 0~3.

  return 1;
}

int ESP32SJA1000Class::wakeup()
{
  modifyRegister(REG_MOD, 0x1f, 0x00);
  //ATENTION: (By Henry warning) I dont find this flag in manual. The TWAI_MODE_REG have only bits 0~3.

  return 1;
}

void ESP32SJA1000Class::setPins(int rx, int tx)
{
  _rxPin = (gpio_num_t)rx;
  _txPin = (gpio_num_t)tx;
}

void ESP32SJA1000Class::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 32; i++) {
    byte b = readRegister(i);

    out.print("0x");
    if (i < 16) {
      out.print('0');
    }
    out.print(i, HEX);
    out.print(": 0x");
    if (b < 16) {
      out.print('0');
    }
    out.println(b, HEX);
  }
}

void ESP32SJA1000Class::handleInterrupt()
{
  uint8_t ir = readRegister(REG_IR);

  if (ir & 0x01) {
    // received packet, parse and call callback
    parsePacket();

  if (_onReceive != 0)  // Protect to use only when a valid pointer is used.
    _onReceive(available());
  }
}

uint8_t ESP32SJA1000Class::readRegister(uint8_t address)
{
  volatile uint32_t* reg = (volatile uint32_t*)(REG_BASE + address * 4);

  return *reg;
}

void ESP32SJA1000Class::modifyRegister(uint8_t address, uint8_t mask, uint8_t value)
{
  volatile uint32_t* reg = (volatile uint32_t*)(REG_BASE + address * 4);

  *reg = (*reg & ~mask) | value;
}

void ESP32SJA1000Class::writeRegister(uint8_t address, uint8_t value)
{
  volatile uint32_t* reg = (volatile uint32_t*)(REG_BASE + address * 4);

  *reg = value;
}

void ESP32SJA1000Class::onInterrupt(void* arg)
{
  ((ESP32SJA1000Class*)arg)->handleInterrupt();
}

ESP32SJA1000Class CAN;

#endif
