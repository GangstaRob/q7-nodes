
/* NSP **********************************************************/

/**
* Nanosatellite protocol
* used to calculate the Cyclic Redundancy Check (CRC), 
* which is based on the bytes within a message. 
*/
uint16_t NSP_calc_crc(uint8_t * bufp, unsigned int len);


/* Commands *******************************/

/**
* Sends initialize command to the RW. 
* This will start the program within the RW and 
* allow the user to send telemetry commands. A second call
* of this command will stop the program in the RW.
*/
static void RW_telecommand_init();


/**
* Sends ping command to the RW.
*/
static int RW_ping();


/**
* Performs RW test mode
*/
static void _self_test();




/* RW Telemetry commands *********************************************/

/**
* Converts bytes into floats and floats into bytes
*/
typedef union
{
    float value;
    uint8_t byte[sizeof(float)];
} RW_value_t;


/**
* Sets speed. Use: _set_speed(speed);
*/
void _set_speed(RW_value_t speed);


/**
* Stops RW (sets speed to 0)
*/
static void _stop_wheel();


/**
* Returns value from RW. Use: _get_speed(&speed);
*/
void _get_speed(RW_value_t* speed);


/**
* Sets torque. Use: _set_torque(torque);
*/
void _set_torque(RW_value_t torque);


/**
* Returns value from RW. Use: _get_torque(&torque);
*/
void _get_torque(RW_value_t* torque);


