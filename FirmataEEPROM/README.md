# FirmataEEPROM

```
#define EEPROM_READ                 0x40
#define EEPROM_WRITE                0x41
#define EERPROM_READ_RESPONSE       0x42
```

## EEPROM Read Message Format
| type  |  command |
|---|---|
|  sysex start |  0xF0  |
|  EEPROM CMD  |  0x40  |
|  ADDRESS Byte 1|  0x00 - 0x7F |
|  ADDRESS Byte 2|  0x00 - 0x7F |
|  sysex end   |  0xF7  |

## EEPROM READ RESPONSE Message Format
| type  |  command |
|---|---|
|  sysex start |  0xF0  |
|  EEPROM CMD  |  0x42  |
|  ADDRESS Byte 1|  0x00 - 0x7F |
|  ADDRESS Byte 2|  0x00 - 0x7F |
|  EEPROM DATA Byte 1 |  0x00 - 0x7F |
|  EEPROM DATA Byte 2 |  0x00 - 0x7F |
|  sysex end   |  0xF7  |

## EEPROM Write Message Format
| type  |  command |
|---|---|
|  sysex start |  0xF0  |
|  EEPROM CMD  |  0x42  |
|  ADDRESS Byte 1|  0x00 - 0x7F |
|  ADDRESS Byte 2|  0x00 - 0x7F |
|  EEPROM DATA Byte 1 |  0x00 - 0x7F |
|  EEPROM DATA Byte 2 |  0x00 - 0x7F |
|  sysex end   |  0xF7  |
