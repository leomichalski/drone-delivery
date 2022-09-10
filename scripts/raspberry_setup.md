## Raspberry Pi Manual Setup
This setup supposes the username is pi. 

### Connect the board to wifi

Poweroff the board properly. Remove the SD card. Connect the SD card to another computer. Go to the sd card folder named "boot".

Create a file called "wpa_supplicant.conf".
```
touch wpa_supplicant.conf
```

Write the following text inside the file if you're in Brazil (remember to change the wifi password and the wifi name). If you're not in Brazil, just change the 2-digit country code.
```
country=BR # Your 2-digit country code
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev

network={
    ssid="YOUR_NETWORK_NAME"
    psk="YOUR_PASSWORD"
    scan_ssid=1
    key_mgmt=WPA-PSK
}
```

### Be sure to add yourself in the tty and dialout groups via usermod to avoid to have to execute scripts as root

```
sudo usermod -a -G tty pi
sudo usermod -a -G dialout pi
```
