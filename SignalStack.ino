/*
REQUIREMENTS:
    -interface with RoveComm
        -RoveComm.begin()
    -receive motor commands
        -manual control in both directions
            -small value so no ramping is required and equipment doesn't break
            -digitalWrite high to one moco and low to other moco
        -possibly closed loop?
            -ramp speed up to small value
    -send GPS data
        -start serial communication
        -serial comm with GPS
        -RoveComm.write()
    -send compass data
        -I2C comm with GPS
        -RoveComm.write()
*/