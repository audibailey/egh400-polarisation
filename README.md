# EGH400 Reproducible Polarisation

### Perdana Bailey

## Intro
This is the repository for all my work regarding the software side of the polarisation experiment. This was made 
using ROS2 with a complicated setup of an RPi and RoboStack.

## Steps: RPi
1. Install the ArenaSDK
2. Clone this repository onto the Raspberry Pi into a ROS2 Workspace
3. Clone ds4_driver, lucid_vision_driver and vrpn_mocap into the `src` directory of this repository
4. Run `sudo apt update && rosdep update`
5. Run `rosdep install --from-paths src -y --ignore-src`
6. Run `colcon build` and hope it all builds
7. Use `bluetoothctl` to connect to a PS4 controller
8. Run `ros2 launch calibrate_process launch_project.py`

## Steps: Normal Computer (Does the processing)
1. Clone this repository into a ROS2 Workspace
2. Run `rosdep update`
3. Run `rosdep install --from-paths src -y --ignore-src`
4. Change `parent_path` in `calibrate_process.py`
5Run `ros2 launch calibrate_process calibrate_process`

## Troubleshooting
### Running the polarisation camera on Ethernet when RPi is using ROS2 for WIFI
ROS2 tries to transmit over what ever interface it wants and when the polarisation camera is attached, it will try 
and transmit it over the polarisation camera... This is obviously incorrect.

Steps to fix (run on RPi)
1. Run `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
2. Run `export interface=$(for dev in /sys/class/net/*; do [ -e "$dev"/wireless ] && echo ${dev##*/}; done)`
3. Run
   ```
   echo "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
    <CycloneDDS xmlns=\"https://cdds.io/config\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd\">
        <Domain id=\"any\">
            <General>
                <NetworkInterfaceAddress>$interface</NetworkInterfaceAddress>
            </General>
        </Domain>
    </CycloneDDS>" > /tmp/cyclonedds_interface.xml
   ```
4. Run `export CYCLONEDDS_URI="file:///tmp/cyclonedds_interface.xml"`

More information can be found on:
https://dds-demonstrators.readthedocs.io/en/latest/Teams/1.Hurricane/setupCycloneDDS.html


