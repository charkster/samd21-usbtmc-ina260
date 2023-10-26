# samd21-usbtmc-ina260
Combine a SAMD21 and a INA260 to create a USBTMC instrument.

Adafruit SAMD21 QT PY             |  Adafruit INA260            | STEMMA QT / QWIIC
:-------------------------:|:-------------------------:|:-------------------------:|
![](https://cdn-learn.adafruit.com/assets/assets/000/095/173/large1024/adafruit_products_QTPy_top.jpg)  |  ![](https://d2t1xqejof9utc.cloudfront.net/screenshots/pics/a0f601ef2c4a887e5ef29240891e0e13/large.png)  | ![](https://cdn-shop.adafruit.com/970x728/4397-02.jpg)

SDA, SCL, 3.3V and GND signals shared using the STEMMA QT cable (just add pins to INA260 board).
Program the SAMD21 QT_PY board by double-clicking the push-button and drag-n-drop the provided [UF2 file](https://github.com/charkster/samd21-usbtmc-ina260/blob/main/qt_py-samd21-usbtmc-ina260.uf2)

Here are the supported SCPI commands:

***IDN?** (query the instrument name, returns 'INA260\nINA260:VOLTAGE? INA260:CURRENT? INA260:POWER?')

***RST** (Sends the Reset command to INA260 over I2C)

**INA260:VOLTAGE?** (query the Bus voltage on INA260)

**INA260:CURRENT?** (query the current flow on INA260, negative means reverse direction)

**INA260:POWER?** (query the power on INA260)

My C string parsing is very crude so the **SCPI** command **must match** what I have listed above (or be in lowercase).
All the major credit goes to [Nathan Conrad (TINYUSB's USBTMC author)](https://github.com/pigrew) and [Alex Taradov's SAMD21 bare metal peripheral control](https://github.com/ataradov/dgw).
