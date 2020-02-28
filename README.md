# ST25DV
Arduino library to support the NFC ST25DV components:
 * ST25DV04K
 * ST25DV64K

## API

This NFC sensor uses I2C to communicate.
It creates the instance st25dv.

It is then required to call begin API with the 2 pins to be used before accessing to the sensors, and optionally wire instance:
    `    st25dv->begin(gpo_pin, lpd_pin);`
or
```
    TwoWire MyWire(SDA_PIN, SCL_PIN);
    st25dv->begin(gpo_pin, lpd_pin, MyWire);
```


It is then possible to read/write NFC URI:

```
    int writeURI(String protocol, String uri, String info);
    String readURI();
```


## Documentation

You can find the source files at
https://github.com/stm32duino/ST25DV

The ST25DV datasheets are available at
https://www.st.com/content/st_com/en/products/nfc/st25-nfc-rfid-tags-readers/st25-dynamic-nfc-tags/st25dv-i2c-series-dynamic-nfc-tags/st25dv04k.html
https://www.st.com/content/st_com/en/products/nfc/st25-nfc-rfid-tags-readers/st25-dynamic-nfc-tags/st25dv-i2c-series-dynamic-nfc-tags/st25dv64k.html