# QUECTEL LC29X Global Navigation Satellite System (GNSS) DRIVER Library (WIP) #

Library for Interfacing with the LC29H Module


## LC29X PAIR Query & Response Information ##
This chapter explains PAIR messages (proprietary NMEA messages defined by the chipset supplier). “P” means proprietary message, “AIR” means the command defined by the chipset supplier.


### PAIR COMMANDS ###
Refer to Quectel LC29H documentation. Here are some examples below:

#### Packet Type: 002 PAIR_GNSS_SUBSYS_POWER_ON####
    Details:
        Powers on the GNSS system, including DSP/RF/PE/Clock, etc.
    Synopsis: 
        $PAIR002*<Checksum><CR><LF>
    Parameter: 
        None.
    Result: 
        Returns a PAIR_ACK message.
    Example: 
        $PAIR002*38 
        $PAIR001,002,0*39

#### Packet Type: 003 PAIR_GNSS_SUBSYS_POWER_OFF ####
    Details:
        Powers off the GNSS system, including DSP/RF/PE/Clock etc. After you send this command, CM4 can still receive commands (including PAIR commands that are not reliant on DSP).
    Type:
        Command.
    Synopsis:
        $PAIR003*<Checksum><CR><LF>
    Parameter:
        None.
    Result:
        Returns a PAIR_ACK message.
    Example:
        $PAIR003*39
        $PAIR001,003,0*38
