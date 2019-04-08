/* Include the controller definition */
#include "footbot_aggregation.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <limits>
#include <iostream>
#include <string>
#include <cstring>
#include <cerrno>

/* PI value definition */
#define PI (3.141592653589793238462643383279502884L)

/* TWO_PI value definition */
#define TWO_PI (6.2831853071795864769252867665590057683943L)

/****************************************/
/****************************************/

CFootBotAggregation::CFootBotAggregation() :
		m_pcWheels(NULL), m_pcProximity(NULL), m_cAlpha(90.0f), m_fDelta(0.5f), m_fWheelVelocity(
				2.5f), minDist(100), a(0.14f), b(0), k(15), numOfTimeStepTurning(
				0), maxNeighborsSeen(0), countMaxNeighbors(0), counter(0), left(
				0), right(0), goStraight(50), obstacleFlag(0), rho(0.5), blackSpotCounter(
				0), goBlackPoint(0), waitBlackPoint(0), differentialDrive(18), walkInsideSpot(
				150), leaveInsideSpot(800), currentWord(0), waitInsideSpot(200), m_fStayTurns(
				50), m_fLeaveTurns(50), m_fWalkTurns(50), spotOut(""), robotNum(
				0), numInformedRobot(10), numInformedRobotBlack(10), numInformedRobotWhite(
				10), numOfNeighboursWhileJoining(-1), numOfNeighboursWhileOnSite(
				-1), spotInfoSite(-1), computeProbVal(-1), logstr(), computeProbUniform(
				-1), informedSpot(0), spotInf(-1), clockCounter(0), exploratoryFlag(
				0), m_pcRABA(
		NULL), m_pcRABS(
		NULL), m_pcRNG(NULL), m_pcGroundZ(NULL), state(0), stateStep(0), avoidTurns(
				0), stayTurns(0), leaveTurns(0), walkTurns(1), spotTurns(0), spotFlag(
				0), probaRule(2), lastMove(0), m_cGoStraightAngleRange(
				-ToRadians(m_cAlpha), ToRadians(m_cAlpha)) {
}

/****************************************/
/****************************************/

void CFootBotAggregation::Init(TConfigurationNode& t_node) {

	/*
	 * Get sensor/actuator handles
	 *
	 * The passed string (ex. "differential_steering") corresponds to the
	 * XML tag of the device whose handle we want to have. For a list of
	 * allowed values, type at the command prompt:
	 *
	 * $ argos3 -q actuators
	 *
	 * to have a list of all the possible actuators, or
	 *
	 * $ argos3 -q sensors
	 *
	 * to have a list of all the possible sensors.
	 *
	 * NOTE: ARGoS creates and initializes actuators and sensors
	 * internally, on the basis of the lists provided the configuration
	 * file at the <controllers><footbot_aggregation><actuators> and
	 * <controllers><footbot_aggregation><sensors> sections. If you forgot to
	 * list a device in the XML and then you request it here, an error
	 * occurs.
	 */
	m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>(
			"differential_steering");
	m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
	m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
	m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");

//	m_pcGround = GetSensor<CCI_FootBotMotorGroundSensor>(
//			"footbot_motor_ground");

//	m_pcGround = GetSensor<CFootBotMotorGroundRotZOnlySensor::CCI_FootBotMotorGroundSensor>(
//			"footbot_motor_ground");

	m_pcGroundZ = GetSensor<CFootBotMotorGroundRotZOnlySensor>(
			"footbot_motor_ground");

//	const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads2 =
//								m_pcGroundZ->GetReadings();
//
//	const CFootBotMotorGroundRotZOnlySensor::TReadings& tGroundReads2 =
//									m_pcGroundZ->m_bAddNoise();
//

//	const CFootBotMotorGroundRotZOnlySensor::TReadings& tGroundNoise =
//						m_pcGroundZ->GetReadings();
//
//	m_pcGroundZ->
//	bool sdf= m_pcGroundZ->m_bAddNoise;

	/*
	 * os3-examples
	 *  the configuration file
	 */
	m_cAlpha = CDegrees(90.0f);
	m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
	m_fDelta = 0.5;
	m_fWheelVelocity = 10;
	GetNodeAttribute(t_node, "minDist", minDist);
	GetNodeAttribute(t_node, "aParam", a);
	GetNodeAttribute(t_node, "bParam", b);
	GetNodeAttribute(t_node, "kParam", k);
	GetNodeAttribute(t_node, "leaveTurns", m_fLeaveTurns);
	GetNodeAttribute(t_node, "stayTurns", m_fStayTurns);
	GetNodeAttribute(t_node, "walkTurns", m_fWalkTurns);
	GetNodeAttribute(t_node, "rule", probaRule);
	GetNodeAttribute(t_node, "goStraight", goStraight);
	GetNodeAttribute(t_node, "walkInsideSpot", walkInsideSpot);
	GetNodeAttribute(t_node, "leaveInsideSpot", leaveInsideSpot);
	GetNodeAttribute(t_node, "waitInsideSpot", waitInsideSpot);
	GetNodeAttribute(t_node, "informedSpot", informedSpot);
	GetNodeAttribute(t_node, "numInformedRobot", numInformedRobot);
	GetNodeAttribute(t_node, "numInformedRobotBlack", numInformedRobotBlack);
	GetNodeAttribute(t_node, "numInformedRobotWhite", numInformedRobotWhite);
	GetNodeAttribute(t_node, "countMaxNeighbors", countMaxNeighbors);
	GetNodeAttribute(t_node, "output", m_strOutFile);

//	string fileName = "Robot_" + GetId() + ".txt";
//	m_strOutFile = m_strOutFile + "/" + fileName;
//	/* Open the file for text writing */
//	m_cOutFile.open(m_strOutFile.c_str(),
//			std::ofstream::in | std::ofstream::out | std::ofstream::app);
//	if (m_cOutFile.fail()) {
//		THROW_ARGOSEXCEPTION(
//				"Error opening file \"" << m_strOutFile << "\": " << ::strerror(errno));
//	}

//	const CCI_RangeAndBearingSensor::TReadings& tPackets =
//			m_pcRABS->GetReadings();
//numInformedRobot = tPackets.size() * numInformedRobot/100;
//	for (size_t i = 0; i < tPackets.size(); ++i) {
//		m_pcRABA->SetData(1, i);
//		//hear(i);
////		if (i < 10) {
////			m_pcRABA->SetData(1, 0);
////		} else {
////			m_pcRABA->SetData(1, 1);
////		}
//	}
	/* Create a random number generator. We use the 'argos' category so
	 that creation, reset, seeding and cleanup are managed by ARGoS. */
	m_pcRNG = CRandom::CreateRNG("argos");
	Reset();
}

void CFootBotAggregation::Reset() {
	lastMove = 0;
	avoidTurns = 0;
	leaveTurns = 0;
	stayTurns = 0;
	walkTurns = 1;
	spotTurns = 0;
	spotFlag = 0;
	counter = 0;
	numOfTimeStepTurning = 0;
	left = 0;
	right = 0;
	stateStep = 0;
	exploratoryFlag = 0;
	clockCounter = 0;
	lexicon.clear();
	m_pcRABA->ClearData();
	UpdateState(STATE_WALK);
	/* Close the output file */
//	m_cOutFile.close();
//	if (m_cOutFile.fail()) {
//		THROW_ARGOSEXCEPTION(
//				"Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
//	}
//	/* Open the file for text writing */
//
//	m_cOutFile.open(m_strOutFile.c_str(),
//			std::ofstream::in | std::ofstream::out | std::ofstream::app);
//	if (m_cOutFile.fail()) {
//		THROW_ARGOSEXCEPTION(
//				"Error opening file \"" << m_strOutFile << "\": " << ::strerror(errno));
//	}
}

/****************************************/
/****************************************/

void CFootBotAggregation::ControlStep() {

	if (!avoidTurns) {
		clockCounter++;
//		m_cOutFile << clockCounter << "	" << numOfNeighboursWhileJoining << "	"
//				<< numOfNeighboursWhileOnSite << "	" << spotInfoSite << "	"
//				<< computeProbUniform << "	" << computeProbVal << endl;
		switch (stateStep) {
		case STATE_WALK:
			WalkStep();
			break;
		case STATE_STAY:
			StayStep();
			break;
		case STATE_LEAVE:
			LeaveStep();
			break;
		}
	} else {
		--avoidTurns;
//		clockCounter++;
//		m_cOutFile << clockCounter << "	" << numOfNeighboursWhileJoining << "	"
//				<< numOfNeighboursWhileOnSite << "	" << spotInfoSite << "	"
//				<< computeProbUniform << "	" << computeProbVal << endl;

	}
}

string CFootBotAggregation::GetStateStep() {
	switch (stateStep) {
	case STATE_WALK:
		return "WALK";
		break;
	case STATE_STAY:
		return "STAY";
		break;
	case STATE_LEAVE:
		return "LEAVE";
		break;
	}
	return "0";
}

/******************************************/
/******************************************/

void CFootBotAggregation::WalkStep() {

	if (CheckSpot() == STATE_WALK) {

		blackSpotCounter = 0;
		goBlackPoint = 0;
		waitBlackPoint = 0;
		spotTurns = 0;
		spotFlag = 0;
		spotOut = "";
		walkInsideSpot = goStraight; // 500; // m_pcRNG->Uniform(CRange<Real>(300.0, 800.0));
		// Go straight  times - step.
		if (counter < goStraight && left == 0 && right == 0) {
			/* Go straight */
			UpdateState(STATE_WALK);
			counter++;
		} else {
			RandomTurn();
		}

	} else {
		UpdateState(STATE_STAY);

//		float p = ComputeProba(CountNeighbours());
//		float randUniform = m_pcRNG->Uniform(CRange<Real>(0.0, 1.0));
//		computeProbUniform = randUniform;
//		computeProbVal=p;
//
//		if (randUniform < p) {
//			UpdateState(STATE_STAY);
//
//		}
	}

}

void CFootBotAggregation::StayStep() {

	if (goBlackPoint < walkInsideSpot) {
		MoveStep();
		goBlackPoint++;

		if (goBlackPoint > 40 && spotFlag == 0) {

			//test

			//test ends
			/* Read stuff from the ground sensor */
			const CFootBotMotorGroundRotZOnlySensor::TReadings& tGroundReads =
					m_pcGroundZ->GetReadings();

//			const CCI_RangeAndBearingSensor::TReadings& tPackets =
//					m_pcRABS->GetReadings();

			if ((Abs(tGroundReads[0].Value) > 0.1f
					&& Abs(tGroundReads[0].Value) < 0.9f)
					|| (Abs(tGroundReads[1].Value) > 0.1f
							&& Abs(tGroundReads[1].Value) < 0.9f)
					|| (Abs(tGroundReads[2].Value) > 0.1f
							&& Abs(tGroundReads[2].Value) < 0.9f)
					|| (Abs(tGroundReads[3].Value) > 0.1f
							&& Abs(tGroundReads[3].Value) < 0.9f)) {

//				UpdateState(STATE_STAY);
				//goBlackPoint = walkInsideSpot;

//				m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
//						avoidTurns = m_pcRNG->Uniform(
//								CRange<UInt32>(0.25 * COMPLETE_TURN, 0.75 * COMPLETE_TURN));

//				m_pcWheels->SetLinearVelocity(-m_fWheelVelocity,
//						m_fWheelVelocity);

				if (spotTurns < 20) {

					/* Turn right, depending on the sign of the angle  .  */
					m_pcWheels->SetLinearVelocity(m_fWheelVelocity,
							-m_fWheelVelocity);
					spotTurns++;
					//spotFlag = 0;

				}

				walkInsideSpot = goBlackPoint + goBlackPoint / 2;

//				for (int i = 0; i < 10; i++) {
//				MoveStep();
//			}

			} else if (spotTurns > 0 && spotTurns < 20) {
				/* Turn right, depending on the sign of the angle   */
				m_pcWheels->SetLinearVelocity(m_fWheelVelocity,
						-m_fWheelVelocity);
				spotTurns++;
			}

		} else if (CheckSpot() == STATE_WALK) {
			UpdateState(STATE_WALK);
			spotTurns = 0;
		}
//		if (spotOut == "WaitInside") {
//			goBlackPoint = walkInsideSpot;
//			UpdateState(STATE_STAY);
//		}

	} else {
		spotTurns = 0;
		if (numOfNeighboursWhileOnSite < 0) {
//			waitBlackPoint++;
//			UpdateState(STATE_STAY);
//			LOGERR << "computeProbVal:" << computeProbVal
//					<< " computeProbUniform:" << computeProbUniform
//					<< " numOfNeighboursWhileOnSite:" << " waitOnsiteCount:"
//					<< waitBlackPoint << " waitInsideSpot:" << waitInsideSpot
//					<< " robot:" << GetId() << std::endl;

			//join
			stateStep = STATE_WALK;
			float p = ComputeProba(CountNeighbours());
			float randUniform = m_pcRNG->Uniform(CRange<Real>(0.0, 1.0));
			computeProbUniform = randUniform;
			computeProbVal = p;

			if (randUniform < p) {
				UpdateState(STATE_STAY);

			}
		} else {
			waitBlackPoint = 0;
			//spotFlag=1;

			//

			float p = ComputeProba(CountNeighbours());
			computeProbVal = p;
			float randUniform = m_pcRNG->Uniform(CRange<Real>(0.0, 1.0));
			computeProbUniform = randUniform;

			if (randUniform < p) {

				UpdateState(STATE_LEAVE);

			} else {
				UpdateState(STATE_STAY);

			}

//		LOGERR << "--computeProbVal:" << computeProbVal
//							<< " computeProbUniform:" << computeProbUniform
//							<< " numOfNeighboursWhileOnSite:" << " goSite:"
//							<< goBlackPoint << numOfNeighboursWhileOnSite << " robot:"
//							<< GetId() << std::endl;
		}
	}

}

void CFootBotAggregation::LeaveStep() {

	if (CheckSpot() == STATE_STAY) { //(blackSpotCounter < leaveInsideSpot) {
		MoveStep();
		//blackSpotCounter++;
	} else
		stateStep = STATE_WALK;	//ChangeState(STATE_WALK);
}

void CFootBotAggregation::UpdateState(unsigned short int newState) {

	m_pcRABA->SetData(0, newState);
	stateStep = newState;
	switch (newState) {
	case STATE_WALK:
		MoveStep();
		break;
	case STATE_STAY:
		if (CheckSpot() == STATE_STAY)
			m_pcWheels->SetLinearVelocity(0, 0);
		else
			UpdateState(STATE_LEAVE);
		//speak(true);
		break;
	case STATE_LEAVE:
		spotFlag = 1;
		numOfNeighboursWhileJoining = -1;
		numOfNeighboursWhileOnSite = -1;
		spotInfoSite = -1;
		MoveStep();

		break;
	}
}

void CFootBotAggregation::RandomTurn() {
// random turn
	if (counter == goStraight && left == 0 && right == 0) {
		counter = 0;
		numOfTimeStepTurning = 0;
		double ran = rand() / double(RAND_MAX);
		if (ran < 0.5) {
			numOfTimeStepTurning = random_turn_lenght(ran);
			left = 1;
		} else {
			numOfTimeStepTurning = random_turn_lenght(ran);
			right = 1;
		}
	}

	if (counter < numOfTimeStepTurning && left == 1) {

		/* Turn left, depending on the sign of the angle   */
		m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
		counter++;
	} else if (counter < numOfTimeStepTurning && right == 1) {

		/* Turn right, depending on the sign of the angle   */
		m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
		counter++;
	} else if (counter == numOfTimeStepTurning) {
		counter = 0;
		numOfTimeStepTurning = 0;
		left = 0;
		right = 0;
		stateStep = 0;
	}

}

void CFootBotAggregation::MoveStep() {
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads =
			m_pcProximity->GetReadings();

	bool obstacle = false;
// Check obstacles in the [-pi/2,pi/2] range
	for (size_t i = 0; i < tProxReads.size(); ++i) {
		if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(
				tProxReads[i].Angle)) {
			if (tProxReads[i].Value > m_fDelta) {
				obstacle = true;
			}
		}
	}

	if (obstacle) {
		m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
		avoidTurns = m_pcRNG->Uniform(
				CRange<UInt32>(0.25 * COMPLETE_TURN, 0.75 * COMPLETE_TURN));

		if (spotFlag == 0) {
			spotTurns = 0;
		}

		if (goBlackPoint > 40) {
			goBlackPoint = walkInsideSpot;
		}

//		if(obstacleFlag=-0)
		//m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
//		else{
//
//		}
//		counter = 0;
//		numOfTimeStepTurning = differentialDrive;
//		double ran = rand() / double(RAND_MAX);
//		if (ran < 0.5)
//			left = 1;
//		else
//			right = 1;
//		stateStep = 0;
//		RandomTurn();

	} else {
		m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
		//spotTurns=0;
	}
}

int CFootBotAggregation::CheckSpot() {

//	const CCI_RangeAndBearingSensor::TReadings& tPackets =
//				m_pcRABS->GetReadings();
	/* Read stuff from the ground sensor */
	CFootBotMotorGroundRotZOnlySensor::TReadings tGroundReads =
			m_pcGroundZ->GetReadings();

	float noise_level = 0.05;
	tGroundReads[0].Value += m_pcRNG->Gaussian(0, noise_level);
	tGroundReads[1].Value += m_pcRNG->Gaussian(0, noise_level);
	tGroundReads[2].Value += m_pcRNG->Gaussian(0, noise_level);
	tGroundReads[3].Value += m_pcRNG->Gaussian(0, noise_level);

	if (Abs(tGroundReads[0].Value) < 0.09f || Abs(tGroundReads[1].Value) < 0.09f
			|| Abs(tGroundReads[2].Value) < 0.09f
			|| Abs(tGroundReads[3].Value) < 0.09f) {
		spotInf = 0;

		return InformedRobot(1);

	} else if (Abs(tGroundReads[0].Value - 1.0f) < 0.09f
			|| Abs(tGroundReads[1].Value - 1.0f) < 0.09f
			|| Abs(tGroundReads[2].Value - 1.0f) < 0.09f
			|| Abs(tGroundReads[3].Value - 1.0f) < 0.09f) {
		spotInf = 1;

		return InformedRobot(0);

	} else {
		return STATE_WALK;
	}

}

int CFootBotAggregation::InformedRobot(int spot) {
	//informedSpot = 0:white-  1:black
	int avoidSpot = informedSpot;
	int value = atoi(GetId().c_str());
	if (avoidSpot == spot) {
		if (value >= numInformedRobotWhite && value < numInformedRobot) {
			//LOGERR << "avoid white " << value << std::endl;
			return STATE_LEAVE;
		} else {
			return STATE_STAY;
		}
	} else {
		if (value < numInformedRobotWhite) {
			//LOGERR << "avoid black " << value << std::endl;
			return STATE_LEAVE;
		} else {
			return STATE_STAY;
		}
	}
}

//int CFootBotAggregation::InformedRobot(int spot) {
//	int infSpot = informedSpot;
//	if (infSpot == spot) {
//		int value = atoi(GetId().c_str());
//		if (value < numInformedRobot) {
//			return STATE_LEAVE;
//		} else {
////			if (spotOut == "inside") {
////				spotOut = "WaitInside";
////			}
//			return STATE_STAY;
//		}
//	} else {
////		if (spotOut == "inside") {
////			spotOut = "WaitInside";
////		}
//
//		return STATE_STAY;
//	}
//}

/****************************************/
/****************************************/

void CFootBotAggregation::speak(bool activate, int channel) {
	if (activate) {
		if (lexicon.size() == 0)
			lexicon.push_back(m_pcRNG->Uniform(CRange<UInt32>(1, 256)));
		currentWord = lexicon[m_pcRNG->Uniform(
				CRange<UInt32>(0, lexicon.size()))];
		m_pcRABA->SetData(channel, currentWord);
	} else
		m_pcRABA->SetData(channel, 0);
}

bool CFootBotAggregation::hear(unsigned short int w) {
	bool inside = false;

	if (w > 0) {
		for (vector<unsigned short int>::iterator it = lexicon.begin();
				it != lexicon.end() && !inside; ++it) {
			if (w == *it) {
				inside = true;
			}
		}

		if (inside) {
			currentWord = w;
			if (lexicon.size() > 1) {
				lexicon.clear();
				lexicon.push_back(w);
			}
		} else if (not inside)
			lexicon.push_back(w);
	}

	return inside;
}

unsigned int CFootBotAggregation::CountNeighbours() {
	const CCI_RangeAndBearingSensor::TReadings& tPackets =
			m_pcRABS->GetReadings();
	unsigned int counter = 1;
	for (size_t i = 0; i < tPackets.size(); ++i) {

		if (tPackets[i].Range < minDist) {
//			LOGERR << "RangeandBearng:" << tPackets[i].Range << " robot "
//							<< GetId() << std::endl;
			if (probaRule != 4 and tPackets[i].Data[0] == STATE_STAY) {
				//hear(tPackets[i].Data[0]);
				++counter;
			} else {
				++counter;

			}

		}
	}
	if (counter > maxNeighborsSeen) {
		maxNeighborsSeen = counter;
	}
	//LOGERR << "Max neighbors " << maxNeighborsSeen << std::endl;

	return counter;
}

//unsigned int CFootBotAggregation::CountSpotRobot() {
//	CFootBotEntity& footbotEntity = *any_cast<CFootBotEntity*>(
//						it->second);
//
//	CFootBotAggregation& controller =
//						static_cast<CFootBotAggregation&>(footbotEntity.GetControllableEntity().Get());
//
//
//	unsigned int counter = 1;
//
//	if (state == "STAY") {
//					Real Robot_X =
//							footbotEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
//					Real Robot_Y =
//							footbotEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
//					positions.push_back(make_pair(Robot_X, Robot_Y));
//
//	//			m_cOutFile << "    " << Robot_X << " \t         " << Robot_Y
//	//					<< endl;
//
//					cFootbotPosition.Set(Robot_X, Robot_Y);
//
//
//					Real fDistanceSpotBlack =
//							(m_cCoordBlackSpot - cFootbotPosition).Length();
//					if (fDistanceSpotBlack <= m_fRadius && totalSpots>=2) {
//						blackSpotCount += 1;
//						whiteSpotCount4 += 1;
//					}
//	}
//	//LOGERR << "Max neighbors " << maxNeighborsSeen << std::endl;
//
//	return counter;
//}

float CFootBotAggregation::ComputeProba(unsigned int n) {

	switch (probaRule) {
	case 1: //linear
		return n * a;
		break;

	case 5:   //functions sub prop new eq
		--n;
		switch (stateStep) {
		case STATE_WALK: //P_join

			if (numOfNeighboursWhileJoining < 0) { //while non informed robot join, calculate number of neighbors
				numOfNeighboursWhileJoining = n;  // n is counted neighbors
				numOfNeighboursWhileOnSite = n;
				spotInfoSite = spotInf;
			}
			return 1;
			break;
		case STATE_STAY: //1-P_leave

			numOfNeighboursWhileOnSite = n;
			int value = atoi(GetId().c_str());       // get robot ID

//			clockCounter++;
			//0 is black and 1 is white.
			//m_cOutFile << clockCounter << "	" << n << "	" << spotInf << endl;

//			LOGERR << "clockCounter:" << clockCounter << "	neigh:" << n
//					<< " spotInf:" << spotInf << " robot:" << GetId()
//					<< " numInfR:" << numInformedRobot << std::endl;

			//if (value >= numInformedRobot && numInformedRobot > 0) { // if robot non informed robot and total informed > 0 use new method

			//if (value >= numInformedRobot) { // if robot non informed robot


				float Res1 = std::numeric_limits<float>::infinity();

				int nnres = Abs(n - numOfNeighboursWhileJoining);
				float kkres = (-b) * (k - nnres);
				Res1 = Exp(-b * (k - Abs(n - numOfNeighboursWhileJoining)));
				computeProbVal = Res1;


//				LOGERR << "clockCounter:" << clockCounter << "	neigh:" << n
//						<< " nnres:" << nnres << " robot:" << GetId()
//						<< " kkres:" << kkres << " Res1:" << Res1 << std::endl;

				float one = 1.0f;

				return Min(one, Res1);

//			} else {
//				float Ress = std::numeric_limits<float>::infinity();
//				Ress = exp(-b * n);
//				computeProbVal = Ress;
//				return Ress;
//			}
			break;
		}
		break;
	case 7:   //functions sub prop new eq
		--n;
		switch (stateStep) {
		case STATE_WALK: //P_join

			if (numOfNeighboursWhileJoining < 0) { //while non informed robot join, calculate number of neighbors
				numOfNeighboursWhileJoining = n;  // n is counted neighbors
				numOfNeighboursWhileOnSite = n;
				spotInfoSite = spotInf;
			}
			return 1;
			break;
		case STATE_STAY: //1-P_leave

			numOfNeighboursWhileOnSite = n;
			int value = atoi(GetId().c_str());       // get robot ID

//			clockCounter++;
			//0 is black and 1 is white.
			//m_cOutFile << clockCounter << "	" << n << "	" << spotInf << endl;

//			LOGERR << "clockCounter:" << clockCounter << "	neigh:" << n
//					<< " spotInf:" << spotInf << " robot:" << GetId()
//					<< " numInfR:" << numInformedRobot << std::endl;

			//if (value >= numInformedRobot && numInformedRobot > 0) { // if robot non informed robot and total informed > 0 use new method
			if (value >= numInformedRobot) { // if robot non informed robot
				Real Res1 = Exp(
						-b * (k - Abs(n - numOfNeighboursWhileJoining)));

				Real one = 1.0f;

				return Min(one, Res1);

			} else {
				return Exp(-2.2 * n);
			}
			break;
		}
		break;

	case 6: //functions sub prop
		--n;
		switch (stateStep) {
		case STATE_WALK: //P_join

			if (numOfNeighboursWhileJoining < 0) { //while non informed robot join, calculate number of neighbors
				numOfNeighboursWhileJoining = n;  // n is counted neighbors
			}
			return 1;
			break;
		case STATE_STAY: //1-P_leave

			float pLeaveNonInformed[8] =
					{ 0.0, 0.0000002051, 0.0000167017, 0.0001507331,
							0.0013603680, 0.0122773399, 0.1108031584, 1.0 }; // array for non informed return values
			int value = atoi(GetId().c_str());       // get robot ID

			if (value >= numInformedRobot) { // if robot non informed robot and total informed > 0 use new method

				int X = numOfNeighboursWhileJoining; // numOfNeighboursWhileJoining is equal to X

				if (n == X - 0 || n == X + 0) // X + or -0 robots p(leave) -> 0.0000000000
					return pLeaveNonInformed[0];
				else if (n == X - 1 || n == X + 1) // X + or -1 robots p(leave) -> 0.0000002051
					return pLeaveNonInformed[1];
				else if (n == X - 2 || n == X + 2) // X + or -2 robots p(leave) -> 0.0000167017
					return pLeaveNonInformed[2];
				else if (n == X - 3 || n == X + 3) // X + or -3 robots p(leave) -> 0.0001507331
					return pLeaveNonInformed[3];
				else if (n == X - 4 || n == X + 4) // X + or -4 robots p(leave) -> 0.0013603680
					return pLeaveNonInformed[4];
				else if (n == X - 5 || n == X + 5) // X + or -5 robots p(leave) -> 0.0122773399
					return pLeaveNonInformed[5];
				else if (n == X - 6 || n == X + 6) // X + or -6 robots p(leave) -> 0.1108031584
					return pLeaveNonInformed[6];
				else if (n == X - 7 || n == X + 7) // X + or -7 robots p(leave) -> 1.0000000000
					return pLeaveNonInformed[7];
				else
					// X + or -  > 7 robots p(leave) -> 1.0000000000
					return pLeaveNonInformed[7];
			} else {

				return Exp(-b * n);
			}
			break;
		}
		break;
	case 2: //functions sub prop
		--n;
		switch (stateStep) {
		case STATE_WALK: //P_join

			return 0.03 + 0.48 * (1 - Exp(-a * n));

			break;
		case STATE_STAY: //1-P_leave
			return Exp(-b * n);

			break;
		}
		break;
	case 4: //campo
		--n;
		switch (stateStep) {
		case STATE_WALK: //P_join
//			double Miu=0.001;
		//double sss = countMaxNeighbors;
			return 0.001 * (1 - n / countMaxNeighbors);
			//return 1;

			break;
		case STATE_STAY: //1-P_leave
			double theta = 0.01;
			double rho = 1667.0;
			double S_local = countMaxNeighbors;
			return theta / (1 + rho * (n / S_local) * (n / S_local));

			break;
		}
		break;
	case 3: //correll and martinolli
		--n; //don't count the bot itself
		float pJoin[5] = { 0.03, 0.42, 0.5, 0.51, 0.51 };
		float pLeave[5] = { 1, 0.02, 0.00236, 0.0014, 0.000766 };
		if (n > 4)
			n = 4;
		switch (stateStep) {
		case STATE_WALK:
			return pJoin[n];
			break;
		case STATE_STAY:
			return pLeave[n];
			break;
		}
		break;

	}
	return 0;
}

unsigned short int CFootBotAggregation::GetWord() {
	unsigned short int w = 0; //means no convergence

	if (lexicon.size() >= 1)
		w = lexicon.front();

	return w;
}

string CFootBotAggregation::text() {
	return ToString(GetWord());
}

vector<unsigned short int> CFootBotAggregation::GetLexicon() {
	return lexicon;
}

/****************************************/
/****************************************/
/****************************************/
/****************************************/
/****************************************/
/****************************************/
/****************************************/
/****************************************/

void CFootBotAggregation::Move() {
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads =
			m_pcProximity->GetReadings();

	bool obstacle = false;
// Check obstacles in the [-pi/2,pi/2] range
	for (size_t i = 0; i < tProxReads.size(); ++i) {
		if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(
				tProxReads[i].Angle)) {
			if (tProxReads[i].Value > m_fDelta) {
				obstacle = true;
			}
		}
	}

	if (obstacle) {
		m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
//avoidTurns = m_pcRNG->Uniform(CRange<UInt32>(0.25*COMPLETE_TURN,0.75*COMPLETE_TURN));
		avoidTurns = random_turn_lenght();
	} else {
		m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
	}
	lastMove = 0;
}

void CFootBotAggregation::ChangeState(unsigned short int newState) {

	m_pcRABA->SetData(0, newState);
	state = newState;
	switch (newState) {
	case STATE_WALK:
		Move();
		walkTurns = m_fWalkTurns;
		break;
	case STATE_STAY:
		m_pcWheels->SetLinearVelocity(0, 0);
		stayTurns = m_fStayTurns;
		break;
	case STATE_LEAVE:
		Move();
		leaveTurns = m_fLeaveTurns;
		numOfNeighboursWhileJoining = -1;

		break;
	}
}

void CFootBotAggregation::Walk() {
	--walkTurns;
	if (walkTurns == 0) {
		float p = ComputeProba(CountNeighbours());
//cout<<p<<endl;
// if(m_pcRNG->Uniform(CRange<Real>(0.0f,1.0f)) < p)
		if (random_turn_lenght() < p)
			ChangeState(STATE_STAY);
		else
			ChangeState(STATE_WALK);
	} else if (state == STATE_WALK)
		Move();
}

void CFootBotAggregation::Stay() {
	--stayTurns;
	if (stayTurns == 0) {
		float p = ComputeProba(CountNeighbours());
//		LOGERR << "ComputeProbaSSS:" << p << " spotInf:" << spotInf << " robot:"
//					<< GetId() << std::endl;
//if(m_pcRNG->Uniform(CRange<Real>(0.0f,1.0f)) < p)

		if (random_turn_lenght() < p)
			ChangeState(STATE_STAY);
		else
			ChangeState(STATE_LEAVE);
	}

}

void CFootBotAggregation::Leave() {
	--leaveTurns;
	if (leaveTurns == 0)
		ChangeState(STATE_WALK);
	else
		Move();
}

string CFootBotAggregation::GetState() {
	switch (state) {
	case STATE_WALK:
		return "WALK";
		break;
	case STATE_STAY:
		return "STAY";
		break;
	case STATE_LEAVE:
		return "LEAVE";
		break;
	}
	return "0";
}

int CFootBotAggregation::LastMove() {
	return lastMove;
}

/****************************************/
/****************************************/

/*
 * This Method return random turn lenght
 */

int CFootBotAggregation::random_turn_lenght(double ran) {

//double ran = rand() / double(RAND_MAX);
	/*Random number between 0,differentialDrive 180 degree  */
	double c = ((2.0 * rho) / (1.0 + (rho * rho)));
	double V = cos(ran * TWO_PI);
	double sigma = acos((V + c) / (1 + (c * V))); //% [0, PI];
	return (int) (rint)(COMPLETE_TURN * sigma) / PI;
}

int CFootBotAggregation::random_turn_lenght() {

	double ran = rand() / double(RAND_MAX);
	/*Random number between 0,differentialDrive 180 degree  */
	double c = ((2.0 * rho) / (1.0 + (rho * rho)));
	double V = cos(ran * TWO_PI);
	double sigma = acos((V + c) / (1 + (c * V))); //% [0, PI];
	return (int) (rint)(COMPLETE_TURN * sigma) / PI;
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotAggregation, "footbot_aggregation_controller")
