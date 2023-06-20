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

### Enable ssh

Put the SD card on a computer. Access the "boot" directory. Create an empty file named "ssh".
```
touch /path/to/folder/named/boot/ssh
```

Then, insert the SD card back into the Raspberry Pi and power on. Make sure your computer is connected to the same network as the Pi board. Discover the Raspberry Pi IP address. The following command lists the IP addresses in the network, one of them should be the Raspberry Pi IP address. 
```
nmap -sn $(hostname -I | awk '{ print $1 }')/24
```

There are other ways to find out the Raspberry Pi IP address. One of them is to open a terminal in the Raspberry Pi, then run:
```
hostname -I
# the first IP that appeared is probably the one that you want
```

Run the following command in the computer to SHH into the board. Substitute RASPBERRY_PI_IP with the board ip address.

```
ssh pi@RASPBERRY_PI_IP

# e.g
ssh pi@192.168.0.53
```

The last step will demand a password. The default password is "raspberry". Usually, it's an important security measure to change the default password. There's also an option to ssh into the Raspberry Pi without passwords. Further information: [raspberrypi.com/documentation/computers/remote-access.html](https://www.raspberrypi.com/documentation/computers/remote-access.html)

### Be sure to add yourself in the tty and dialout groups via usermod to avoid to have to execute scripts as root

```
sudo usermod -a -G tty pi
sudo usermod -a -G dialout pi
```
