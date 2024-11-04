# How to Setup CLR

## Networking description

## UDEV Rules

We use udev rules due to USB dev changes when rebooting computers. It is expected in the URDF that there is a rule for the gripper, lift, and safety com port for the rail.

```
sudo touch /etc/udev/rules.d/<name_of_rule>.rules
```
Then create the relevant rule in that file.

### UDEV Rules For Robotic Components
```
SUBSYSTEM=="tty", DEVPATH=="/devices/pci0000:00/0000:00:14.0/usb1/1-8/1-8.4/1-8.4:1.0/ttyUSB0/tty/ttyUSB0", ATTRS{idProduct}=="6010", ATTRS{idVendor}=="0403", SYMLINK+="robotiq_unused", MODE="0666"
SUBSYSTEM=="tty", DEVPATH=="/devices/pci0000:00/0000:00:14.0/usb1/1-8/1-8.4/1-8.4:1.1/ttyUSB1/tty/ttyUSB1", ATTRS{idProduct}=="6010", ATTRS{idVendor}=="0403", SYMLINK+="robotiq", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idProduct}=="6001", ATTRS{idVendor}=="0403", SYMLINK+="ewellix", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idProduct}=="0043", ATTRS{idVendor}=="2341", SYMLINK+="safety_com_port", MODE="0666"
```

### UDEV Rules For CLR Console
```
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="a0:ce:c8:79:81:15", NAME="imetro_axis"
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="00:50:b6:0b:3f:ae", NAME="imetro_er4"
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="40:a6:b7:ac:e7:10", NAME="clr_robot"
```

### UDEV Rules For CLR Controls
```
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="38:ca:84:4c:1d:ac", NAME="clr_robot"
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="a0:ce:c8:79:19:a0", NAME="imetro_er4"
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="a0:ce:c8:79:81:11", NAME="rail"
```

A diagram showing all of the components and their connections is located on the [CLR Confluence Page](https://bender.jsc.nasa.gov/confluence/x/wRFBDg)
