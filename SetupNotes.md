# How to Setup CLR 

## Networking description

## UDEV Rules

We use udev rules due to USB dev changes when rebooting computers. It is expected in the URDF that there is a rule for the gripper, lift, and safety com port for the rail.

```
sudo touch /etc/udev/rules.d/<name_of_rule>.rules
```
Then create the relevant rule in that file.

### UDEV Rules
```
SUBSYSTEM=="tty", ATTRS{idProduct}=="6015", ATTRS{idVendor}=="0403", SYMLINK+="robotiq", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idProduct}=="6001", ATTRS{idVendor}=="0403", SYMLINK+="ewellix", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idProduct}=="0043", ATTRS{idVendor}=="2341", SYMLINK+="safety_com_port", MODE="0666"

```
