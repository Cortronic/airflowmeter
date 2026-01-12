#include "Pid.h"
#include "Arduino.h"

//IRM Discrete PID library for Arduino

//IRM Constructor: define initial variables
Pid::Pid(float p, float i, float d, float minO, float maxO){

	/*
	IRM Input Parameters:
		- p : Proportional (Kp) constant
		- i : Integration  (Ki) constant
		- d : Derivative   (Kd) constant
		- minO : Minimum PID output value; e.g. 0
		- maxO : Maximum PID output value; e.g. 255
	*/


	//IRM Initialize PID with user-specified parameters
	this->pidInit(p, i, d, minO, maxO);

}

//IRM Destructor: do nothing
Pid::~Pid(){
	;
}



/*
==============================
IRM Public method definitions
==============================
*/

//IRM Variables initialization. Set initial values to class global variables
void Pid::pidInit(float p, float i, float d, float minO, float maxO){
	this->kP = p; this->kI = i; this->kD = d; //IRM PID Controller parameters (Kp, Ki, Kd)
	this->minOut = minO; this->maxOut = maxO; //IRM Integrator cummulative limits

	//IRM Initial values for PID
	this->resetIDvalues(); //IRM Reset (I) and (D) components
	this->error = 0; //IRM reset current sample error (P)
	 
}

//IRM Establish set-point value
void Pid::pidSetPoint(float sp){
	this->setPoint = sp; //IRM Establish new set-point value
	this->resetIDvalues(); //IRM Reset (I) and (D) components
}

//IRM Return current set-point value
float Pid::getSetPoint(){
	return this->setPoint;
}


float Pid::getMinOut(){
	return this->minOut;
}

float Pid::getMaxOut(){
	return this->maxOut;
}

//IRM Set new min actuator output value
void Pid::setNewMinOutput(float newMin){
	this->minOut = newMin;
}

void Pid::setNewMaxOutput(float newMax){
	this->maxOut = newMax;
}

//IRM Feed PID with current sample and fetch next computed output value
float Pid::pidUpdate(float currentValue){
	float pValue, iValue, dValue, PID;
	this->__setErrorValue(this -> setPoint - currentValue); //IRM Compute current error value
	
	//Serial.print("SP: "); Serial.print(this -> setPoint); Serial.print(" CURVAL: "); Serial.println(currentValue);
	
	float err = this->__getErrorvalue(); //IRM Local updated error value during current sample

	/*
	==========================
				(P) Computation
	==========================
	*/

	//IRM Compute proportional error component
	pValue = this->__getKp()*err;
	


	/*
	==========================
				(I) Computation
	==========================
	*/

	//IRM Discrete integral. Sample period must remain constant to achieve a reliable I value


	//IRM accumulate current error to integrator value
	this->__accumulateIntegration(err);
	
	float i = this->__getIntegratorValue(); //IRM Current integrator value

	float ki = this->__getKi(); //IRM Ki constant retreived from constructor
	
	//IRM Compute (I) component based on updated values times Ki constant
	iValue = i*ki;



	/*
	==========================
				(D) Computation
	==========================
	*/

	//IRM Retrieve current diff(erentiation) value
	float d = this->__getDiffValue();

	//IRM Compute (D) component based on updated values times Kd constant
	dValue = (err - d)*(this->__getKd()); 

	//IRM Compute current diff(erentiation) value for next sample (set as previous error)
	this->__setDiffValue(err);


	/*
	==========================
			 (PID) Computation
	==========================
	*/

	PID = pValue + iValue + dValue;

	//IRM Truncate integrator to compensate PID (clamp to) min/max output value, avoiding Windup

	float mi = this -> getMinOut(); //IRM Min output possible value
	float mx = this -> getMaxOut(); //IRM Max output possible value

	if(PID > mx){
		i -= ((PID - mx) / ki);
		this->__setIntegratorValue(i);
		PID = mx;
	}else if(PID < mi){
		i -= ((PID - mi) / ki);
		this->__setIntegratorValue(i);
		PID = mi;
	}

	return PID;
}





/*
==============================
IRM Private method definitions
==============================
*/

//IRM Reset cummulative error (I) and diference between prior and current error (D) components
void Pid::resetIDvalues(){
	this->integrator = 0;
	this->diff       = 0;
}

//IRM Retrieve current integration value and add up current error (e)
void Pid::__accumulateIntegration(float e){
	this->__setIntegratorValue(this->__getIntegratorValue() + e);
}


/*
===============================
IRM Private setters and getters
===============================
*/

//IRM Setters
void Pid::__setErrorValue(float e){
	this->error = e;
}

void Pid::__setIntegratorValue(float i){
	this->integrator = i;
}

void Pid::__setDiffValue(float d){
	this->diff = d;
}



//IRM Getters
float Pid::__getKp(){
	return this->kP;
}

float Pid::__getKi(){
	return this->kI;
}

float Pid::__getKd(){
	return this->kD;
}

float Pid::__getErrorvalue(){
	return this->error;
}

float Pid::__getIntegratorValue(){
	return this->integrator;
}

float Pid::__getDiffValue(){
	return this->diff;
}