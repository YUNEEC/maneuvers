# maneuvers 
Execute flight maneuvers using droncecode_sdk and px4-autopilot.

## build
```bash
cd px4maneuvers
mkdir build && cd build
cmake ../src
```

If you want to build all maneuvers:
```bash 
make
```
If you want to build just a specific maneuver:
```
make [maneuver-name]
```
## start maneuver
First launch px4-autopilot. 
To start / launch a maneuver, type from within the build folder:
```bash
./maneuvers/[maneuver-name] udp://:14540
```
