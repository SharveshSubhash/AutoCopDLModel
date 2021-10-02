
## Objective
* Raspberry Pi was the preferred microcontroller for this project as all of the Computer-vision code was written in Python.
* Since, none of our team members have ever worked with a Raspberry Pi, but were very well experienced with the Arduino IDE, we looked for ways to make Arduino code for the PID line-follower bot work on Raspberry Pi. And indeed! there was a way:-

### Instructions for Arduino IDE
* Open the installation folder of Arduino IDE
* Create a folder named "RaspberryPi" inside "hardware" and clone the repository to a folder named "piduino"
```bash
mkdir hardware/RaspberryPi
cd hardware/RaspberryPi
git clone https://github.com/me-no-dev/RasPiArduino piduino
```
* Download , extract and copy the toolchain to piduino/tools/arm-linux-gnueabihf
  - Windows: [gnutoolchains.com](http://gnutoolchains.com/raspberry/)
    * The toolchain for Jessie will work ONLY on RaspberryPi 2
    * The toolchain for Wheezy will work on ALL RaspberryPi boards (recommended)
    * [Video Instructions](https://www.youtube.com/watch?v=lZvhtfUlY8Y)
  - Linux 64: [arm-linux-gnueabihf](https://github.com/me-no-dev/RasPiArduino/releases/download/0.0.1/arm-linux-gnueabihf-linux64.tar.gz)
  - Linux 32: [arm-linux-gnueabihf](https://github.com/me-no-dev/RasPiArduino/releases/download/0.0.1/arm-linux-gnueabihf-linux32.tar.gz)
  - Mac OS X: [arm-linux-gnueabihf](https://github.com/me-no-dev/RasPiArduino/releases/download/0.0.1/arm-linux-gnueabihf-osx.tar.gz)
* Restart Arduino IDE and select the RaspberryPI from the list of boards
* Compile a sketch
* Select the RaspberryPi from the list of Ports (will show the IP address)
* Upload your sketch and see it go


### Instructions for the PI
* Install Raspbian Jessie on your RaspberryPI
* Gain root permissions
```bash
sudo su
```

* Enable password login for root
```bash
passwd
```
  - _enter the new root password twice_
  - Edit `/etc/ssh/sshd_config` and make sure that the following lines exist and are not commented
```bash
PermitRootLogin yes
PasswordAuthentication yes
```

* Disable Serial Console on boot by removing `console=/dev/ttyAMA0` from /boot/cmdline.txt (or through raspi-config)

* Disable Serial tty
```bash
systemctl disable serial-getty@ttyAMA0
```

* Disable loading sound kernel module
```
sed -i "s/dtparam=audio=on/#dtparam=audio=on/" /boot/config.txt
```

* Change the hostname for your Pi (optional) (also through raspi-config)
```bash
hostnamectl set-hostname piduino
```

* Setup WiFi (optional)
```bash
cat > /etc/wpa_supplicant/wpa_supplicant.conf <<EOL
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
network={
    ssid="your-ssid"
    psk="your-pass"
}
EOL
```

* Setup avahi service to allow updating the sketch from ArduinoIDE
```bash
cat > /etc/avahi/services/arduino.service <<EOL
<?xml version="1.0" standalone='no'?><!--*-nxml-*-->
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name replace-wildcards="yes">%h</name>
  <service>
    <type>_arduino._tcp</type>
    <port>22</port>
    <txt-record>board=bplus</txt-record>
  </service>
</service-group>
EOL

service avahi-daemon restart
```

* Install telnet and git
```bash
apt-get update
apt-get install telnet git
```

* Copy all files from tools/arpi_bins to /usr/local/bin
```bash
git clone https://github.com/me-no-dev/RasPiArduino.git piduino
chmod +x piduino/tools/arpi_bins/*
cp piduino/tools/arpi_bins/* /usr/local/bin
rm -rf piduino
```

* Create symbolic link for _run-avrdude_
```bash
ln -s /usr/local/bin/run-avrdude /usr/bin/run-avrdude
```

* Synchronize time and start sketch on boot (optional)
```bash
apt-get install ntpdate
cat > /etc/rc.local <<EOL
#!/bin/sh -e

_IP=\$(hostname -I) || true
if [ "\$_IP" ]; then
  printf "My IP address is %s\n" "\$_IP"
fi

# Sync Time
ntpdate-debian -u > /dev/null
# Start Sketch
/usr/local/bin/run-sketch > /dev/null

exit 0
EOL
```

* Prevent some RealTek USB WiFi from sleep (optional) (EU)
```bash
echo "options 8192cu rtw_power_mgnt=0 rtw_enusbss=1 rtw_ips_mode=1" > /etc/modprobe.d/8192cu.conf
echo "options r8188eu rtw_power_mgnt=0 rtw_enusbss=1 rtw_ips_mode=1" > /etc/modprobe.d/r8188eu.conf
```

* Disable screen blank (optional)
```bash
sed -i "s/BLANK_TIME=30/BLANK_TIME=0/" /etc/kbd/config
sed -i "s/POWERDOWN_TIME=30/POWERDOWN_TIME=0/" /etc/kbd/config
```

* Do not load I2C UART or SPI kernel drivers

* Reboot

* After this you can view a board named Raspberry Pi from the boards options in Arduino IDE

* Now you can write code in Arduino IDE and make the code work on Raspberry Pi!


## Code for PID lifo

```C++
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float Kp = 0; //set up the constants value
float Ki = 0;
float Kd = 0;
int P;
int I;
int D;

int lastError = 0;
boolean onoff = false;

//Increasing the maxspeed can damage the motors - at a value of 255 the 6V motors will receive 7,4 V 
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

//Set up the drive motor carrier pins
int mode = 8;
int aphase = 9;
int aenbl = 6;
int bphase = 5;
int benbl = 3;

//Set up the buttons pins
int buttoncalibrate = 17 //pin A3
int buttonstart = 2;

void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  //Set up the sensor array pins
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);
  qtr.setEmitterPin(7);//LEDON PIN

  pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  digitalWrite(mode, HIGH);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  boolean Ok = false;
  while (Ok == false) { //the loop won't start until the robot is calibrated
    if(digitalRead(buttoncalibrate) == HIGH) {
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
    }
  }
  forward_brake(0, 0);
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if(digitalRead(buttonstart) == HIGH) {
    onoff =! onoff;
    if(onoff = true) {
      delay(1000);//a delay when the robot starts
    }
    else {
      delay(50);
    }
  }
  if (onoff == true) {
    PID_control();
  }
  else {
    forward_brake(0,0);
  }
}
void forward_brake(int posa, int posb) {
  //set the appropriate values for aphase and bphase so that the robot goes straight
  digitalWrite(aphase, LOW);
  digitalWrite(bphase, HIGH);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}
void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  //Serial.print(motorspeeda);Serial.print(" ");Serial.println(motorspeedb);
  forward_brake(motorspeeda, motorspeedb);
}
```
