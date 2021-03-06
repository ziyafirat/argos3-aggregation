#ifndef FOOTBOT_AGGREGATION_H
#define FOOTBOT_AGGREGATION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot motor ground sensor */
//#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

#include <argos3/plugins/robots/foot-bot/simulator/footbot_motor_ground_rotzonly_sensor.h>

#include <iostream>
#include <string>
#include <math.h>

#define COMPLETE_TURN 10*round(2*3.14*14/(2*m_fWheelVelocity))
#define STATE_WALK 0
#define STATE_STAY 1
#define STATE_LEAVE 2

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace std;
/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotAggregation: public CCI_Controller {

private:

	/**
	 * The path of the output file.
	 */
	std::string m_strOutFile;

	/**
	 * The stream associated to the output file.
	 */
	std::ofstream m_cOutFile;

public:

	/* Class constructor. */
	CFootBotAggregation();

	/* Class destructor. */
	virtual ~CFootBotAggregation() {
	}

	/* Random turn  */
	int random_turn_lenght();
	int random_turn_lenght(double);

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_diffusion_controller> section.
	 */
	virtual void Init(TConfigurationNode& t_node);

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	virtual void ControlStep();

	virtual void Move();

	virtual void MoveStep();

	virtual void ChangeState(unsigned short int newState);

	virtual void UpdateState(unsigned short int newState);

	virtual unsigned int CountNeighbours();

	virtual float ComputeProba(unsigned int n);

	virtual void Walk();

	virtual void Stay();

	virtual void Leave();

	virtual void WalkStep();

	virtual void StayStep();

	virtual void LeaveStep();

	virtual void RandomTurn();

	virtual int CheckSpot();

	virtual string GetState();

	virtual string GetStateStep();

	//virtual string IntToString ( int number );
	//virtual unsigned int CountSpotRobot();

	virtual int LastMove();

	virtual int InformedRobot(int spot);

	virtual void speak(bool activate, int channel = 0);
	virtual bool hear(unsigned short int w);

	virtual unsigned short int GetWord();
	virtual string text();
	virtual vector<unsigned short int> GetLexicon();

	/*
	 * This function resets the controller to its state right after the
	 * Init().
	 * It is called when you press the reset button in the GUI.
	 * In this example controller there is no need for resetting anything,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	virtual void Reset();

	/*
	 * Called to cleanup what done by Init() when the experiment finishes.
	 * In this example controller there is no need for clean anything up,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	virtual void Destroy() {
	}

protected:

	/* Pointer to the differential steering actuator */
	CCI_DifferentialSteeringActuator* m_pcWheels;
	/* Pointer to the foot-bot proximity sensor */
	CCI_FootBotProximitySensor* m_pcProximity;
	/* Pointer to the range and bearing actuator */
	CCI_RangeAndBearingActuator* m_pcRABA;
	/* Pointer to the range and bearing sensor */
	CCI_RangeAndBearingSensor* m_pcRABS;

	/* Pointer to the foot-bot motor ground sensor */
	//CCI_FootBotMotorGroundSensor* m_pcGround;
	/* Pointer to the foot-bot motor ground sensor */
	CFootBotMotorGroundRotZOnlySensor* m_pcGroundZ;

	int maxNeighborsSeen;
	int countMaxNeighbors;
	int counter;
	int numOfTimeStepTurning;
	int left;
	int right;
	float computeProbVal;
	float computeProbUniform;
	int lastMove;
	int avoidTurns;
	int leaveTurns;
	int stayTurns;
	int walkTurns;
	int spotTurns;
	int spotFlag;
	int exploratoryFlag;
	int numOfNeighboursWhileJoining;
	int numOfNeighboursWhileOnSite;
	int spotInfoSite;
	string spotOut;
	unsigned short int state;

	unsigned short int stateStep;

	unsigned short int probaRule;
	/* The random number generator */
	CRandom::CRNG* m_pcRNG;

	/*
	 * The following variables are used as parameters for the
	 * algorithm. You can set their value in the <parameters> section
	 * of the XML configuration file, under the
	 * <controllers><footbot_diffusion_controller> section.
	 */

	/* Maximum tolerance for the angle between
	 * the robot heading direction and
	 * the closest obstacle detected. */
	CDegrees m_cAlpha;
	/* Maximum tolerance for the proximity reading between
	 * the robot and the closest obstacle.
	 * The proximity reading is 0 when nothing is detected
	 * and grows exponentially to 1 when the obstacle is
	 * touching the robot.
	 */
	Real m_fDelta;
	/* Wheel speed. */
	Real m_fWheelVelocity;

	/* Base staying proba */
	Real a, b;
	int k;
	string logstr;
	unsigned int minDist, m_fStayTurns, m_fLeaveTurns, m_fWalkTurns;
	int blackSpotCounter;
	int clockCounter;
	int spotInf;
	//int goStraight;
	int goBlackPoint;
	int waitBlackPoint;
	int obstacleFlag;
	unsigned int goStraight, walkInsideSpot, leaveInsideSpot, waitInsideSpot,
			informedSpot, numInformedRobot, numInformedRobotBlack,
			numInformedRobotWhite,swarmSize;

	/* Angle tolerance range to go straight.
	 * It is set to [-alpha,alpha]. */
	CRange<CRadians> m_cGoStraightAngleRange;

	double rho;
	int differentialDrive;

	int robotNum;
	unsigned short int currentWord;
	vector<unsigned short int> lexicon;

};

#endif
