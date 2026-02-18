    #include <cmath>

    //----------- CONFIGURATION -------------

    double loop_time = 0.01*1000; //convert to millis

    //physics constants
    #define THRUST_COEFF 0.0001
    #define QUADCOPTER_MASS 0.1
    #define G 9.81

    //pin numbers
    #define HARDWARE_SAFETY_PIN 13
    #define LEFT_PWM_PIN 5
    #define FRONT_PWM_PIN 6
    #define RIGHT_PWM_PIN 9
    #define REAR_PWM_PIN 10

    //motor control constants
    #define MIN_PULSE 1000
    #define MAX_PULSE 2000
    #define KV 1
    #define KA 0

    //safety constants:
    #define BRIDGE_STALE_MS 200
    #define BRIDGE_NO_DATA_MS 600
    #define IMU_STALE_MS 50
    #define IMU_NO_DATA_MS 200


    // Can comment out for test runs on different platforms
    // (which may not support these libraries/hardware)
    #define ENABLE_BRIDGE
    #define ENABLE_IMU
    #define ENABLE_MOTORS
    // #define ENABLE_SERIAL_DEBUGGING
    //TODO: set motors to hover velocity without requiring signal from linux processor
    // #define COMS_FREE_HOVER

    const bool ENABLE_GENTLE_CRASH = false;
    //TESTING TOOL. SET MOTORS TO HOVER WITH NO SIGNAL FROM LINUX, WHENEVER HARDWARE SAFETY IS ENABLED.
    const bool COMS_FREE_HOVER = false;


    //IMPORTANT: These values come from compute_mixer.py in the scripts folder
    // This matrix is used to convert the output of the LQR into motor thrusts
    const double LQR_MIXER[4][4] = {
        {0.250000,3.535534,0.000000,-0.250000},
        {0.250000,0.000000,3.535534,0.250000},
        {0.250000,-3.535534,-0.000000,-0.250000},
        {0.250000,-0.000000,-3.535534,0.250000},
    };

    //IMPORTANT: These values come from compute_k.py in the scripts folder
    //These matrixes define the constants for the control loop
    //These two arrays are the same but kneg is negated
    const double LQR_K[4][12] = {
        {-0.000000,-0.000000,6.716873,-0.000000,-0.000000,6.571935,0.000000,-0.000000,-0.000000,0.000000,-0.000000,-0.000000},
        {-0.000000,-1.309546,0.000000,0.000000,-1.638736,0.000000,6.204574,0.000000,0.000000,1.127978,0.000000,0.000000},
        {1.309546,-0.000000,0.000000,1.638736,-0.000000,0.000000,0.000000,6.204574,-0.000000,0.000000,1.127978,-0.000000},
        {-0.000000,-0.000000,0.000000,-0.000000,-0.000000,-0.000000,0.000000,-0.000000,3.126511,0.000000,-0.000000,1.978831},
    };

    const double LQR_KNEG[4][12] = {
        {0.000000,0.000000,-6.716873,0.000000,0.000000,-6.571935,-0.000000,0.000000,0.000000,-0.000000,0.000000,0.000000},
        {0.000000,1.309546,-0.000000,-0.000000,1.638736,-0.000000,-6.204574,-0.000000,-0.000000,-1.127978,-0.000000,-0.000000},
        {-1.309546,0.000000,-0.000000,-1.638736,0.000000,-0.000000,-0.000000,-6.204574,0.000000,-0.000000,-1.127978,0.000000},
        {0.000000,0.000000,-0.000000,0.000000,0.000000,0.000000,-0.000000,0.000000,-3.126511,-0.000000,0.000000,-1.978831},
    };







    // conditional includes
    #ifdef ENABLE_BRIDGE
    #include "Arduino_RouterBridge.h"
    #endif

    #ifdef ENABLE_IMU
    #include "REG.h"
    #include "wit_c_sdk.h"
    #endif

    #ifdef ENABLE_MOTORS
    #include <Servo.h>
    #endif

    #ifdef ENABLE_SERIAL_DEBUGGING
    #include <Serial.h>
    #endif


    unsigned long loop_start_time = 0;
    unsigned long bridge_recieve_timestamp = 0;









    //----------- STATUS MONITORING -------------
    /*
    Note - status indicators:
    - IMU (not connected, up-to-date tilted, up-to-date level, stale)
    - Loop health (Within loop time, <10ms over, over <100ms, over >200ms)
    - Linux bridge health (Good, Stale data, No data)
    - Armed (Armed, disarmed manually, disarmed due to fault)
    - 
    */

    enum IMUStatus{
        UP_TO_DATE,
        STALE_IMU,
        NO_SIGNAL
    };

    enum LoopStatus{
        NORMAL_LOOP,
        OVERRUN_10MS,
        OVERRUN_100MS,
        OVERRUN_200MS
    };

    enum BridgeStatus{
        NORMAL_BRIDGE,
        STALE_BRIDGE,
        NO_DATA_BRIDGE
    };

    enum SafetyStatus{
        DISARMED_MANUAL,
        DISARMED_FAULT,
        GENTLE_CRASH_FAULT,
        ARMED
    };

    IMUStatus imu_status = UP_TO_DATE;
    LoopStatus loop_status = NORMAL_LOOP;
    BridgeStatus bridge_status = NORMAL_BRIDGE;
    SafetyStatus safety_status = DISARMED_MANUAL;

    bool python_arm_flag = false;

    /* Order of state vectors:
        0: position x,
        1: position y,
        2: position z,
        3: velocity x,
        4: velocity y,
        5: velocity z,
        6: roll,
        7: pitch,
        8: yaw,
        9: roll rate,
        10: pitch rate,
        11: yaw rate
    */
    double reference_state[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    double current_state[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

    // {left, front, right, back}
    double motor_velocities[4] = {0,0,0,0};
    double previous_motor_velocities[4] = {0,0,0,0};

    #ifdef ENABLE_IMU
        #define ACC_UPDATE		0x01
        #define GYRO_UPDATE		0x02
        #define ANGLE_UPDATE	0x04
        #define MAG_UPDATE		0x08
        #define READ_UPDATE		0x80
        static volatile char s_cDataUpdate = 0, s_cCmd = 0xff; 

        static void CmdProcess(void);
        static void AutoScanSensor(void);
        static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
        static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
        static void Delayms(uint16_t ucMs);
        const uint32_t c_uiBaud[8] = {0,4800, 9600, 19200, 38400, 57600, 115200, 230400};
        int i;
        float fAcc[3], fGyro[3], fAngle[3];
        int t = millis();
    #endif

    #ifdef ENABLE_MOTORS
    Servo left_motor;
    Servo front_motor;
    Servo right_motor;
    Servo rear_motor;
    #endif

    //these are helper functions to approximate sin/cos much faster
    unsigned int isinTable16[] = {
        0, 1144, 2287, 3430, 4571, 5712, 6850, 7987, 9121, 10252, 11380, 
        12505, 13625, 14742, 15854, 16962, 18064, 19161, 20251, 21336, 22414, 
        23486, 24550, 25607, 26655, 27696, 28729, 29752, 30767, 31772, 32768, 

        33753, 34728, 35693, 36647, 37589, 38521, 39440, 40347, 41243, 42125, 
        42995, 43851, 44695, 45524, 46340, 47142, 47929, 48702, 49460, 50203, 
        50930, 51642, 52339, 53019, 53683, 54331, 54962, 55577, 56174, 56755, 

        57318, 57864, 58392, 58902, 59395, 59869, 60325, 60763, 61182, 61583, 
        61965, 62327, 62671, 62996, 63302, 63588, 63855, 64103, 64331, 64539, 
        64728, 64897, 65047, 65176, 65286, 65375, 65445, 65495, 65525, 65535, 
    };

    double isin(long x){
        bool pos = true;  // positive - keeps an eye on the sign.
        if (x < 0) {
            x = -x;
            pos = false;
        }
        if (x >= 360) x %= 360;
        if (x > 180) {
            x -= 180;
            pos = !pos;
        }
        if (x > 90) x = 180 - x;
        if (pos) return isinTable16[x] * 0.0000152590219;
        return isinTable16[x] * -0.0000152590219;
    }

    double icos(long x){
        return isin(x+90);
    }



    //convert thrust to velocity using magical thrust coefficient
    double thrustToVel(double thrust) {
        return std::sqrt(std::abs(thrust) / THRUST_COEFF) * std::copysign(1.0, thrust);
    }






    //----------- VECTOR/MATRIX HELPERS (FOR LQR)
    void subtract12(const double a[12], const double b[12], double out[12]) {
        for (int i = 0; i < 12; i++)
            out[i] = a[i] - b[i];
    }

    void add4(const double a[4], const double b[4], double out[4]) {
        for (int i = 0; i < 4; i++)
            out[i] = a[i] + b[i];
    }

    void negate4x12(const double in[4][12], double out[4][12]) {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 12; ++j)
                out[i][j] = -in[i][j];
    }

    void mat4x12_vec12(const double mat[4][12], const double vec[12], double out[4]) {
        for (int i = 0; i < 4; i++) {
            out[i] = 0.0;
            for (int j = 0; j < 12; j++)
                out[i] += mat[i][j] * vec[j];
        }
    }

    void mat4x4_vec4(const double mat[4][4], const double vec[4], double out[4]) {
        for (int i = 0; i < 4; i++) {
            out[i] = 0.0;
            for (int j = 0; j < 4; j++)
                out[i] += mat[i][j] * vec[j];
        }
    }

    //THE ACTUAL CONTROL MAGIC
    void lqrControlStep(const double current[12], const double reference[12], double out[4]) {
        double error[12];
        subtract12(current, reference, error);

        // error = current - reference
        // current[8] = yaw

        double yaw = -current[8];
        // double cy = cos(yaw);
        // double sy = sin(yaw);
        //APPROXIMATIONS FOR SPEED
        double cy = icos(static_cast<long>(yaw*57.295779513));
        double sy = isin(static_cast<long>(yaw*57.295779513));

        //---------------------------------------------
        // Rotate the POSITION error into local frame
        //---------------------------------------------

        double posErr_x =  error[0];
        double posErr_y =  error[1];
        double posErr_z =  error[2];

        // Only yaw affects x/y:
        double posErr_local_x = cy * posErr_x - sy * posErr_y;
        double posErr_local_y = sy * posErr_x + cy * posErr_y;
        double posErr_local_z = posErr_z;


        //---------------------------------------------
        // Rotate the VELOCITY error into local frame
        //---------------------------------------------

        double velErr_x = error[3];
        double velErr_y = error[4];
        double velErr_z = error[5];

        double velErr_local_x = cy * velErr_x - sy * velErr_y;
        double velErr_local_y = sy * velErr_x + cy * velErr_y;
        double velErr_local_z = velErr_z;


        //---------------------------------------------
        // Build the 12-element local error vector
        //---------------------------------------------

        double error_local[12] = {
            posErr_local_x,
            posErr_local_y,
            posErr_local_z,
            velErr_local_x,
            velErr_local_y,
            velErr_local_z,
            error[6],   // roll error  (unchanged)
            error[7],   // pitch error (unchanged)
            error[8],   // yaw error   (unchanged!) per your original code
            error[9],   // roll rate
            error[10],  // pitch rate
            error[11]   // yaw rate    (unchanged)
        };

        // double error_local[12];
        // getStateVector(posErr, velErr, error_local);

        // keep yaw and yaw rate unchanged
        error_local[8] = error[8];
        error_local[11] = error[11];

        // double Kneg[4][12];
        // negate4x12(LQR_K, Kneg);

        double feedback[4];
        mat4x12_vec12(LQR_KNEG, error_local, feedback);

        double steady_state[4] = {
            QUADCOPTER_MASS * G,
            0, 0, 0
        };

        add4(feedback, steady_state, out);
    }

    //Convert LQR output to motor velocities.
    //The control matrix (output from LQR) is:
    //{total thrust, roll torque, pitch torque, yaw torque}
    void applyMixer(const double control[4], double out[4]) {
        double thrusts[4];
        mat4x4_vec4(LQR_MIXER, control, thrusts);

        out[0] = thrustToVel(thrusts[1]);  // left
        out[1] = thrustToVel(thrusts[0]);  // front
        out[2] = thrustToVel(thrusts[3]);  // right
        out[3] = thrustToVel(thrusts[2]);   // back
    }








    //--------BRIDGE FUNCTIONS--------
    //These functions are exposed to the bridge
    //and are how this program recieves values
    //from the linux loop.
    void recieve_reference_pos(double pos_x,
                            double pos_y,
                            double pos_z,
                            double ang_pos_x,
                            double ang_pos_y,
                            double ang_pos_z) {
        reference_state[0]  = pos_x;
        reference_state[1]  = pos_y;
        reference_state[2]  = pos_z;
        reference_state[6]  = ang_pos_x;
        reference_state[7]  = ang_pos_y;
        reference_state[8]  = ang_pos_z;
        bridge_recieve_timestamp = millis();
    }

    void recieve_reference_vel(
                            double vel_x,
                            double vel_y,
                            double vel_z,
                            double ang_vel_x,
                            double ang_vel_y,
                            double ang_vel_z) {
        reference_state[3]  = vel_x;
        reference_state[4]  = vel_y;
        reference_state[5]  = vel_z;

        reference_state[9]  = ang_vel_x;
        reference_state[10] = ang_vel_y;
        reference_state[11] = ang_vel_z;
    }


    void recieve_current_state(
                            double cur_pos_x,
                            double cur_pos_y,
                            double cur_pos_z,
                            double cur_vel_x,
                            double cur_vel_y,
                            double cur_vel_z) {
        current_state[0]  = cur_pos_x;
        current_state[1]  = cur_pos_y;
        current_state[2]  = cur_pos_z;

        current_state[3]  = cur_vel_x;
        current_state[4]  = cur_vel_y;
        current_state[5]  = cur_vel_z;
    }

    void recieve_arm(bool arm) {
        python_arm_flag = arm;
    }


    //So I don't have to worry about forgetting which index is which:
    double getLeft() {
        return motor_velocities[0];
    }

    double getFront() {
        return motor_velocities[1];
    }

    double getRight() {
        return motor_velocities[2];
    }

    double getBack() {
        return motor_velocities[3];
    }


    bool ledstate = false;


    void setup() {
        pinMode(LED_BUILTIN, OUTPUT);
        pinMode(HARDWARE_SAFETY_PIN, INPUT);

        //https://www.youtube.com/watch?v=vWpq636f1Z4
        //Higher bridge baud rate requires changing /etc/systemd/system/arduino-bridge
        #ifdef ENABLE_BRIDGE
        Bridge.begin(460800);

        //single letter names for speed
        Bridge.provide("p", recieve_reference_pos);
        Bridge.provide("v", recieve_reference_vel);
        Bridge.provide("s", recieve_current_state);
        Bridge.provide("a", recieve_arm);
        #endif

        #ifdef ENABLE_IMU
        WitInit(WIT_PROTOCOL_NORMAL, 0x50);
        WitSerialWriteRegister(SensorUartSend);
        WitRegisterCallBack(SensorDataUpdata);
        WitDelayMsRegister(Delayms);
        AutoScanSensor();
        #endif

        #ifdef ENABLE_MOTORS
        left_motor.attach(LEFT_PWM_PIN, MIN_PULSE, MAX_PULSE);
        front_motor.attach(FRONT_PWM_PIN, MIN_PULSE, MAX_PULSE);
        right_motor.attach(RIGHT_PWM_PIN, MIN_PULSE, MAX_PULSE);
        rear_motor.attach(REAR_PWM_PIN, MIN_PULSE, MAX_PULSE);
        #endif

        #ifdef ENABLE_SERIAL_DEBUGGING
        Serial.begin(9600);
        #endif

        loop_start_time = millis();
    }


    void loop() {

        //SAFETY CHECKS:
        bool loop_ok = loop_status != OVERRUN_200MS;
        #ifdef ENABLE_BRIDGE
            unsigned long bridge_delay = millis() - bridge_recieve_timestamp;
            if(bridge_delay > BRIDGE_NO_DATA_MS) bridge_status = NO_DATA_BRIDGE;
            else if (bridge_delay > BRIDGE_STALE_MS) bridge_status = STALE_BRIDGE;
            else bridge_status = NORMAL_BRIDGE;

            bool bridge_ok = bridge_status != NO_DATA_BRIDGE;
        #else
            unsigned long bridge_delay = 0;
            bool bridge_ok = true;
        #endif
        #ifdef ENABLE_IMU
            // unsigned long IMU_delay = millis() - imu_state.timestamp_ms;
            // if(IMU_delay > IMU_NO_DATA_MS) imu_status = NO_SIGNAL;
            // else if(IMU_delay > IMU_STALE_MS) imu_status = STALE_IMU;
            // else imu_status = STALE_IMU;

            // bool imu_ok = imu_status != NO_SIGNAL;
        #else
            unsigned long IMU_delay = 0;
            bool imu_ok = true;
        #endif

        //decide arming status:
        if(COMS_FREE_HOVER){
            if(digitalRead(HARDWARE_SAFETY_PIN) == HIGH) safety_status = ARMED;
            else safety_status = DISARMED_MANUAL;
        } else {
            if(!imu_ok || !loop_ok){
                safety_status = DISARMED_FAULT;
            } else if (!bridge_ok) {
                if(safety_status == ARMED && ENABLE_GENTLE_CRASH) safety_status = GENTLE_CRASH_FAULT;
                else safety_status = DISARMED_FAULT;
            }else if (digitalRead(HARDWARE_SAFETY_PIN) == HIGH && python_arm_flag){
                safety_status = ARMED;
            } else {
                safety_status = DISARMED_MANUAL;
            }
        }


        // Get IMU data, store in current state
        #ifdef ENABLE_IMU
            // if(!imu.update()) {
            //     imu_status = NO_SIGNAL;
            // } if {
            while (Serial1.available()) {
                WitSerialDataIn(Serial1.read());
            }
            // }

            if(s_cDataUpdate) {
                ledstate = !ledstate;
                for(i = 0; i < 3; i++){
                    fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
                    fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
                    fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
                }
                // if(imu_state.is_new) imu_status = UP_TO_DATE;

                current_state[6] = fAngle[0];
                current_state[7] = fAngle[1];
                current_state[8] = fAngle[2];
                current_state[9] = fGyro[0];
                current_state[10] = fGyro[1];
                current_state[11] = fGyro[2];

                #ifdef ENABLE_BRIDGE
                    Bridge.call(
                        "p",
                        current_state[8],
                        current_state[7],
                        current_state[6],
                        current_state[11],
                        current_state[10],
                        current_state[9]
                    );
                #endif
            }

        #endif

        if(safety_status == GENTLE_CRASH_FAULT) {
            current_state[0] = 0;
            current_state[1] = 0;
            current_state[2] = 0;
            current_state[3] = 0;
            current_state[4] = 0;
            current_state[5] = 0;
            reference_state[0] = 0;
            reference_state[1] = 0;
            reference_state[2] = -0.1;
            reference_state[3] = 0;
            reference_state[4] = 0;
            reference_state[5] = 0;
        }


        //run the control loop
        double LQR_result[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
        lqrControlStep(current_state, reference_state, LQR_result);
        applyMixer(LQR_result, motor_velocities);

        //spin le motors:
        #ifdef ENABLE_MOTORS
        if(safety_status == ARMED || safety_status == GENTLE_CRASH_FAULT) {
            double left = KV * getLeft();
            double front = KV * getFront();
            double right = KV * getRight();
            double rear = KV * getBack();

            left += (motor_velocities[0] - previous_motor_velocities[0]) * (millis() - loop_start_time) * KA;
            front += (motor_velocities[1] - previous_motor_velocities[1]) * (millis() - loop_start_time) * KA;
            right += (motor_velocities[2] - previous_motor_velocities[2]) * (millis() - loop_start_time) * KA;
            rear += (motor_velocities[3] - previous_motor_velocities[3]) * (millis() - loop_start_time) * KA;

            left_motor.write(left);
            front_motor.write(front);
            right_motor.write(right);
            rear_motor.write(rear);
        } else {
            left_motor.write(0);
            front_motor.write(0);
            right_motor.write(0);
            rear_motor.write(0);
        }
        #endif


        // Transmit data to linux loop
        #ifdef ENABLE_BRIDGE
        Bridge.call(
            "r",
            getLeft(),
            getFront(),
            getRight(),
            getBack()
        );
        #endif




        //maintain loop time:
        unsigned long actual_loop_time = 0;
        while(actual_loop_time < loop_time){
            actual_loop_time = millis() - loop_start_time;
        }
        loop_start_time = millis();

        //Telemetry:
        #ifdef ENABLE_SERIAL_DEBUGGING
            Serial.println("Loop complete in " + actual_loop_time + "ms.")
            if(safety_status == ARMED) Serial.println("Copter Was ARMED")
            else if(safety_status == DISARMED_MANUAL) Serial.println("Copter Was MANUALLY DISARMED")
            else if(safety_status == DISARMED_FAULT) Serial.println("Copter Was DISARMED due to a FAULT")
            else if(safety_status == GENTLE_CRASH_FAULT) Serial.println("Copter is tyring to CRASH GENTLY due to a FAULT")
            if(imu_status == STALE_IMU) Serial.println("FAULT: IMU data stale (not fatal)");
            else if(imu_status == NO_SIGNAL) Serial.println("FAULT: NO IMU Data for " + IMU_delay + "ms (fatal)");
            if(bridge_status == STALE_BRIDGE) Serial.println("FAULT: Bridge data is stale (not fatal)");
            if(bridge_status == NO_DATA_BRIDGE) Serial.println("FAULT: No bridge data for " + bridge_delay + "ms (fatal)")
        #endif

        //heartbeat
        // ledstate = !ledstate;
        digitalWrite(LED_BUILTIN, ledstate ? LOW : HIGH);
    }



















    //IMU GARBAGE
#ifdef ENABLE_IMU
    void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}
}
static void ShowHelp(void)
{
	Monitor.print("\r\n************************	 WIT_SDK_DEMO	************************");
	Monitor.print("\r\n************************          HELP           ************************\r\n");
	Monitor.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	Monitor.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	Monitor.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	Monitor.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	Monitor.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	Monitor.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	Monitor.print("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
  Monitor.print("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
  Monitor.print("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
  Monitor.print("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
  Monitor.print("UART SEND:h\\r\\n   help.\r\n");
	Monitor.print("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	if(WitStartAccCali() != WIT_HAL_OK) Monitor.print("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	if(WitStartMagCali() != WIT_HAL_OK) Monitor.print("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	if(WitStopMagCali() != WIT_HAL_OK) Monitor.print("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Monitor.print("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Monitor.print("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Monitor.print("\r\nSet Baud Error\r\n");
              else 
              {
                Serial1.begin(c_uiBaud[WIT_BAUD_115200]);
                Monitor.print(" 115200 Baud rate modified successfully\r\n");
              }
			break;
		case 'b':	if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Monitor.print("\r\nSet Baud Error\r\n");
              else 
              {
                Serial1.begin(c_uiBaud[WIT_BAUD_9600]); 
                Monitor.print(" 9600 Baud rate modified successfully\r\n");
              }
			break;
		case 'r': if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)  Monitor.print("\r\nSet Baud Error\r\n");
			        else Monitor.print("\r\nSet Baud Success\r\n");
			break;
		case 'R':	if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Monitor.print("\r\nSet Baud Error\r\n");
              else Monitor.print("\r\nSet Baud Success\r\n");
			break;
    case 'C': if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) Monitor.print("\r\nSet RSW Error\r\n");
      break;
    case 'c': if(WitSetContent(RSW_ACC) != WIT_HAL_OK) Monitor.print("\r\nSet RSW Error\r\n");
      break;
		case 'h':	ShowHelp();
			break;
		default :break;
	}
	s_cCmd = 0xff;
}
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  Serial1.write(p_data, uiSize);
  Serial1.flush();
}
static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
	{
		Serial1.begin(c_uiBaud[i]);
    Serial1.flush();
		iRetry = 2;
		s_cDataUpdate = 0;
		do
		{
			WitReadReg(AX, 3);
			delay(200);
      while (Serial1.available())
      {
        WitSerialDataIn(Serial1.read());
      }
			if(s_cDataUpdate != 0)
			{
				Monitor.print(c_uiBaud[i]);
				Monitor.print(" baud find sensor\r\n\r\n");
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	Monitor.print("can not find sensor\r\n");
	Monitor.print("please check your connection\r\n");
}
    //2906 lines w/o headers, 3589 w headers.
#endif