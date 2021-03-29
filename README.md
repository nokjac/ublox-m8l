# ublox-m8l 
### It is a component to configure and start of uBlox EVK-M8L GNSS module on BeagleBoneBlack (BBB)

### Build:
**build gnss_m8l_i2c**
```
make
``` 

**build gnss_m8l_i2c_debug** (contains additional traces)
```
make debug
```

**build gnss_m8l_i2c_broadcast** (sends broadcast messages to the port 49100)
```
make broadcast
```

### Execute
call with parameter for selecting dedicated i2c channel (e.g. /dev/i2c-2 device):
```
gnss_m8l_i2c 2
```
