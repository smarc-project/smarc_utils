# Set up and run Axis cams in the inner pool

## SAM's latop:
- Connect cameras to main unit and unit to samnet. Power both up
- Laptop in samnet as well
- Start WG VPN
- Make sure the http authentication method is Basic in the cams: follow this in Windows https://faq.axis.com/s/article/does-axis-device-manager-support-basic-authentication?language=en_US
- Check that the mosquitto broker daemon is running in laptop
- Run: roslaunch external_cams axis_mqtt_srvr.launch

## External laptop:
- Connect to eduroam
- Start WG VPN
- Run: roslaunch external_cams axis_mqtt_clt.launch
- You should be able to see the live stream from cam 1 in rviz for example
