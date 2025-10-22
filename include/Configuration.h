/**
 * @brief All the constants needed to characterize the quadcopter physics and controls
 */

#define LOOP_TIME 0.01 //main loop time in seconds

//physical dimensions:
#define QUADCOPTER_ROTOR_DISTANCE 0.2 //quadcopter rotor center distance, in meters

#define QUADCOPTER_MASS 1 //mass in kg
#define QUADCOPTER_MOI 1 //moment of intertia of the quadcopter

#define FRONT_LEFT_SPINS_CCW false //allows inverting assumed rotor spin directions to make it more flexible


//Thrust calculation:
#define THRUST_COEFF 0.0000011 //constant used to calculate rotor thrust. units: N per Rad/S.
//thrust calculation based on the formula: Thrust = THRUST_COEFF * AIR_DENSITY * ROTOR_AREA * (rotor velocity * ROTOR_RADIUS)^2
//which can be simplified to Thrust = THRUST_COEFF * rotor velocity^2 (where thrust coeff is empirically measured)

#define ROTOR_DRAG_COEFF 0.000000011 //This is used to calculate the angular force produced by the rotors about the Z axis.
//again the formula is DRAG_COEFF * rotor velocity^2

//super simple drag formula F = -kv. Simple, but might require manual tuning.
#define LINEAR_DRAG_COEFF_XY 0.25
#define LINEAR_DRAG_COEFF_Z 0.25
#define ANGULAR_DRAG_COEFF_XY 0.005
#define ANGULAR_DRAG_COEFF_Z 0.005

/*
RANDOM NOTES:
Possible encoder/magnet pairing:
https://www.apexmagnets.com/magnets/3mm-x-2mm-disc-neodymium-magnet?srsltid=ARcRdnooPzPf5m3-a-cQOnyCKTR5fxU1tlA2biy94N--J_RvzCmAhl6O
https://www.amazon.com/AS5048A-Accuracy-Magnetic-Peripheral-Interface/dp/B08LW1K6F7
*/