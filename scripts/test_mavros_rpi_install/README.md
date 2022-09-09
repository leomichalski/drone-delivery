
### Setup
```
# install
sudo apt install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc

# uninstall
sudo apt remove --purge python3-opencv python3-matplotlib python3-pygame
pip3 uninstall pyserial, pymavlink, mavproxy
```

### Run
Tip: use tmux.

```
# terminal 1
./arducopter -S --model + --speedup 1 --defaults copter.parm -I0

# terminal 2
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551

# terminal 3
roslaunch mavros apm.launch fcu_url:="udp://127.0.0.1:14551@14555"
```
