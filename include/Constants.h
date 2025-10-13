//physical dimensions:
#define QUADCOPTER_ROTOR_DISTANCE 0.2 //quadcopter rotor center distance, in meters

#define QUADCOPTER_MASS 1 //mass in kg
#define QUADCOPTER_MOI 1 //moment of intertia of the quadcopter

#define FRONT_LEFT_SPINS_CCW false //allows inverting assumed rotor spin directions to make it more flexible


//Thrust calculation:
#define THRUST_COEFF 1 //constant used to calculate rotor thrust. units: N per Rad/S.
//thrust calculation based on the formula: Thrust = THRUST_COEFF * AIR_DENSITY * ROTOR_AREA * (rotor velocity * ROTOR_RADIUS)^2
//which can be simplified to Thrust = THRUST_COEFF * rotor velocity^2 (where thrust coeff is empirically measured)

#define ROTOR_DRAG_COEFF 0.1 //This is used to calculate the angular force produced by the rotors about the Z axis.
//again the formula is DRAG_COEFF * rotor velocity^2

//super simple drag formula F = -kv. Simple, but might require manual tuning.
#define LINEAR_DRAG_COEFF_XY 10
#define LINEAR_DRAG_COEFF_Z 10
#define ANGULAR_DRAG_COEFF_XY 10
#define ANGULAR_DRAG_COEFF_Z 10