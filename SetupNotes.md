# How to Setup ChonkUR

## Networking description

## UDEV Rules

We use udev rules due to USB dev changes when rebooting computers. It is expected in the URDF that there is a rule for the gripper.

```
sudo touch /etc/udev/rules.d/<name_of_rule>.rules
```
Then create the relevant rule in that file.

### robotiq hand-e rule
```
SUBSYSTEMS=="usb", ATTRS{idProduct}=="6015", ATTRS{idVendor}=="0403", SYMLINK+="robotiq"
```
