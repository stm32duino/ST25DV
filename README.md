# ST25DV
Arduino library to support the NFC ST25DV components:
 * ST25DV04K
 * ST25DV64K
 * ST25DV04KC
 * ST25DV64KC

## API

This NFC sensor uses I2C to communicate.
It creates the instance st25dv.

It is then required to call begin API:
```cpp
    st25dv.begin();
```

Or you can make your own instance of the component
```cpp
    TwoWire MyWire(SDA_PIN, SCL_PIN);
    ST25DV st25dv(gpo_pin, lpd_pin, MyWire, MySerial);
```


It is then possible to read/write NFC URI:

```cpp
    int writeURI(String protocol, String uri, String info);
    String readURI();
```


## Examples

There are 2 examples with the ST25DV library:
* ST25DV_HelloWorld: This application writes a URI tag on the device. It records an URI.

When the NFC module is started and ready, the message "System init done!" is displayed on the monitor window.
Next, the tag is written, we wait few seconds, we read the same tag and print it on the monitor window.

You can test this application by connecting it with your smartphone.
On Android, download a NFC Tools. Then start the app, check if NFC is activated
on your smartphone. Put your smartphone near the tag, you can read it. You can
write a tag with this app.

* ST25DV_SimpleWrite: This application writes a NDEF message, containing a URI record, to the tag.

When the NFC module is started and ready, the message "System init done!" is displayed on the monitor window.
Next, the tag is written with a URI.

You can test this application by tapping the tag with your smartphone.
On Android, check if NFC is activated on your smartphone.
Put your smartphone near the tag to read it.
The preferred Internet Browser is automatically opened with the provided URI.


## Documentation

You can find the source files at
https://github.com/stm32duino/ST25DV

The ST25DV datasheets are available at:
* https://www.st.com/content/st_com/en/products/nfc/st25-nfc-rfid-tags-readers/st25-dynamic-nfc-tags/st25dv-i2c-series-dynamic-nfc-tags/st25dv04k.html
* https://www.st.com/content/st_com/en/products/nfc/st25-nfc-rfid-tags-readers/st25-dynamic-nfc-tags/st25dv-i2c-series-dynamic-nfc-tags/st25dv64k.html
* https://www.st.com/en/nfc/st25dv04kc.html
* https://www.st.com/en/nfc/st25dv64kc.html
