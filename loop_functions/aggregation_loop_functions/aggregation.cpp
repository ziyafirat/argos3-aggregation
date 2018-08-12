#include "aggregation.h"

#include <algorithm>
#include <cstring>
#include <cerrno>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
/****************************************/
/****************************************/

// add log in loop function  some time-steps. use Init and PostStep method,
// add table list for different spot neigthbour if 1 ; 0.2 , if 3 ; 0.4 , if 6 ; 0.6 etc.
//
// 
static const Real BOT_RADIUS = 0.14f;

/****************************************/
/****************************************/

CAggregation::CAggregation() :
		timeStopCond(), m_fRadius(), minDist(), m_fMinObjectY(), m_fMaxObjectY(), outsideSpotCount(
				0), whiteSpotCount(0), blackSpotCount(0), blackSpotVector(), whiteSpotVector() {
}

/****************************************/
/****************************************/

CAggregation::~CAggregation() {
	/* Nothing to do */
}

/****************************************/
/****************************************/

void CAggregation::Init(TConfigurationNode& t_tree) {
	/* Get output file name from XML tree */

	GetNodeAttribute(t_tree, "output", m_strOutFile);
	GetNodeAttribute(t_tree, "radiusSpot", m_fRadius);
	GetNodeAttribute(t_tree, "blackSpotSize", blackSpotVector);
	GetNodeAttribute(t_tree, "whiteSpotSize", whiteSpotVector);
	GetNodeAttributeOrDefault(t_tree, "minDist", minDist, minDist);
	GetNodeAttributeOrDefault(t_tree, "timeStopCond", timeStopCond,
			timeStopCond);

//2 spots
//	m_cCoordBlackSpot = CVector2(-blackSpotVector, 0);
//	m_cCoordWhiteSpot = CVector2(whiteSpotVector, 0);

//	m_fRadius = 0.6;

//3 spots
	m_cCoordBlackSpot = CVector2(-blackSpotVector, 0);
	m_cCoordWhiteSpot = CVector2(whiteSpotVector, whiteSpotVector);
	m_cCoordWhiteSpot2 = CVector2(whiteSpotVector, -whiteSpotVector);

//4 spots
//	m_cCoordBlackSpot = CVector2(-blackSpotVector, 4);
//	m_cCoordBlackSpot2 = CVector2(-blackSpotVector, -4);
//	m_cCoordWhiteSpot = CVector2(whiteSpotVector, 4);
//	m_cCoordWhiteSpot2 = CVector2(whiteSpotVector, -4);

	blackSpotCount = 0;
	whiteSpotCount = 0;
	outsideSpotCount = 0;

	/* Open the file for text writing */
	m_cOutFile.open(m_strOutFile.c_str(),
			std::ofstream::in | std::ofstream::out | std::ofstream::app);
	if (m_cOutFile.fail()) {
		THROW_ARGOSEXCEPTION(
				"Error opening file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}

	////////////////////////////////////////////////////////////////////////////////// CREATION AND POSITIONING OF THE ARENA WALLS////////////////////////////////////////////////////////////////////////////////
	CVector3 arena_size = GetSpace().GetArenaSize();
	float m_fArenaRadius = Min(arena_size[0], arena_size[1]) / 2;
	int m_unNumArenaWalls = 50;
	CRadians wall_angle = CRadians::TWO_PI / m_unNumArenaWalls;
	CVector3 wall_size(0.05,
			2.0 * m_fArenaRadius * Tan(CRadians::PI / m_unNumArenaWalls), 0.1);
	ostringstream entity_id;
//	m_cOutFile << clock << "	arenaradius: " << m_fArenaRadius << "	" << m_fArenaRadius
//			<< endl;
	for (UInt32 i = 0; i < m_unNumArenaWalls; i++) {
		entity_id.str("");
		entity_id << "wall_" << i;
		CRadians wall_rotation = wall_angle * i;

		CVector3 wall_position(m_fArenaRadius * Cos(wall_rotation),
				m_fArenaRadius * Sin(wall_rotation), 0);
		CQuaternion wall_orientation;
		wall_orientation.FromEulerAngles(wall_rotation, CRadians::ZERO,
				CRadians::ZERO);

		CBoxEntity* box = new CBoxEntity(entity_id.str(), wall_position,
				wall_orientation, false, wall_size, (Real) 1.0);
		AddEntity(*box);
	}

//	m_cOutFile  << "clock	" << "informedRate	"<< "blackSpotCount	"
//			<< "whiteSpotCount	" << "outsideSpotCount	" << endl;
	/////////////////////////////////////

//	   /* Get the map of all foot-bots from the space */
//	   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
//	   /* Go through them */
//	   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
//	       it != tFBMap.end();
//	       ++it) {
//	      /* Create a pointer to the current foot-bot */
//	      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
//	      /* Create a waypoint vector */
//	      m_tWaypoints[pcFB] = std::vector<CVector3>();
//	      /* Add the initial position of the foot-bot */
//	      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
//	   }
//
//
//	 /* Initialize sources configuration */
//	    size_t i;
//	    UInt32 unTrials = 0;
//	    bool bDone = false;
//	    do {
//	      for(i = 0; i < 20; ++i){
//	        m_fSourceGrayLevel[i] = m_pcRNG->Uniform(m_cFloorLevelRange);
//	        LOG << "Floor color for source " << i << ": " << m_fSourceGrayLevel[i] << std::endl;
//	        if(m_fSourceGrayLevel[i] <= m_fBestSource){
//	          m_fBestSource = m_fSourceGrayLevel[i];
//	          m_unBestSource = i;
//	        }
//	      }
//	      if ( fabs(m_fSourceGrayLevel[0] - m_fSourceGrayLevel[1]) > 0.05 ){
//	        bDone = true;
//	      }
//	    } while (!bDone && unTrials < 100);
//
//	    /* Write the header of the output file */
//	    m_cOutFile << "Best source = " << m_unBestSource << std::endl;
//	    m_cOutFile << "#Clock\t";
//	    for(i = 0; i < NUM_SOURCES; ++i){
//	      m_cOutFile << "Source" << i << "\t";
//	    }
//	    m_cOutFile << "Total" << "\t" << std::endl;
//
//	    /* Display best source */
//	    LOG << "Best source is " << m_unBestSource << " with color " << m_cSourceColors[m_unBestSource] << std::endl;
//
//	    /* Add the two cylinder objects */
//	    AddObjects();

}

/****************************************/
/****************************************/

void CAggregation::Reset() {
	/* Close the output file */
	m_cOutFile.close();
	if (m_cOutFile.fail()) {
		THROW_ARGOSEXCEPTION(
				"Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}
	/* Open the file for text writing */
	m_cOutFile.open(m_strOutFile.c_str(),
			std::ofstream::in | std::ofstream::out | std::ofstream::app);
	if (m_cOutFile.fail()) {
		THROW_ARGOSEXCEPTION(
				"Error opening file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}
}

/****************************************/
/****************************************/

void CAggregation::Destroy() {
	/* Close the output file */
	m_cOutFile.close();
	if (m_cOutFile.fail()) {
		THROW_ARGOSEXCEPTION(
				"Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}
}

/****************************************/
/****************************************/

void CAggregation::PreStep() {
	/* Nothing to do */
}

/****************************************/
/****************************************/

list<pair<float, float> > CAggregation::findCluster(
		list<pair<float, float> >::iterator seed,
		list<pair<float, float> >& pos) {
	list<pair<float, float> > cluster;
	pair<float, float> s = *seed;
	cluster.push_back(s);
	pos.erase(seed);

	list<pair<float, float> > temp;
	float d = minDist * 0.01;
	if (pos.size() > 0) {
		bool hasNeighbours = true;
		while (hasNeighbours) {
			hasNeighbours = false;
			for (list<pair<float, float> >::iterator it = pos.begin();
					it != pos.end(); ++it) {
				if (sqrt(
						pow((it->first - s.first), 2)
								+ pow((it->second - s.second), 2)) < d) {
					hasNeighbours = true;
					temp = findCluster(it, pos);
					for (list<pair<float, float> >::iterator it2 = temp.begin();
							it2 != temp.end(); ++it2)
						cluster.push_back(*it2);
					break;
				}
			}
		}
	}

	return cluster;
}

//void CAggregation::CreateRandomInformed() {
//	CCIBehaviorController sdsa
//        gsl_ran_shuffle(randNum, permutation->data, m_unNumberOfFootBots, sizeof(size_t));
//        LOG << "Number of robots: " << m_unNumberOfFootBots << std::endl;
//        LOG << "Informed ratio: " << m_fInformedRatio << std::endl;
//        LOG << "Number of informed: " << m_unNumberOfInformed << std::endl;
//        for (UInt8 i = 0; i < m_unNumberOfInformed; i++) {
//                UInt8 unRandomFBIndex = gsl_permutation_get(permutation, i);
//                std::ostringstream sFootBotNameStream;
//                sFootBotNameStream << "fb_" << unRandomFBIndex;
//                std::string sFbName = sFootBotNameStream.str();
//                CBTFootbotFlockingRootBehavior* pcRootBehavior = m_mFBs_Behaviors.find(sFbName)->second;
//                pcRootBehavior->SetInformed(true);
//                //LOG << "Setting goal direction to " << m_cGoalDirection << std::endl;
//                pcRootBehavior->SetInformationDirection(m_cGoalDirection);
//        }
//}

double CAggregation::std2D(list<pair<float, float> > pos) {
	pair<float, float> accPos = make_pair(0, 0);
	for (list<pair<float, float> >::iterator it = pos.begin(); it != pos.end();
			++it) {
		accPos.first += it->first;
		accPos.second += it->second;
	}
	pair<float, float> meanPos = make_pair(accPos.first / pos.size(),
			accPos.second / pos.size());

	double accStd = 0;
	for (list<pair<float, float> >::iterator it = pos.begin(); it != pos.end();
			++it) {
		accStd += pow((it->first - meanPos.first), 2)
				+ pow((it->second - meanPos.second), 2);
	}

	return accStd / (pos.size() * 4 * pow(BOT_RADIUS, 2));
}

float CAggregation::connectivity(list<pair<float, float> > pos) {
	pair<float, float> accPos = make_pair(0, 0);
	float tot;
	float d = minDist * 0.01;

	for (list<pair<float, float> >::iterator it = pos.begin(); it != pos.end();
			++it) {
		for (list<pair<float, float> >::iterator it2 = pos.begin();
				it2 != pos.end(); ++it2) {
			if (sqrt(
					pow((it->first - it2->first), 2)
							+ pow((it->second - it2->second), 2)) < d) {
				++tot;
			}
		}
	}

	return tot / pos.size();
}

int CAggregation::clustersInfo(list<pair<float, float> > pos,
		vector<int>& sizes, vector<double>& stds) {
	pair<float, float> seed;
	list<pair<float, float> > cluster;
	while (pos.size() > 0) {
		cluster = findCluster(pos.begin(), pos);
		if (cluster.size() >= 2) {
			sizes.push_back(cluster.size());
			stds.push_back(std2D(cluster));
		}
	}

	return sizes.size();
}

void CAggregation::PostStep() {
	int clock = GetSpace().GetSimulationClock();
	if (clock % 100 == 0) {
		list<pair<float, float> > positions;
		set<unsigned short int> words;
		vector<unsigned short int> lexi;
		CVector2 cFootbotPosition(0, 0);
//		m_cOutFile << "Time-Step:" << clock
//				<< "\n    Positions-X \t Positions-Y \n";
		CSpace::TMapPerType& cFBMap = GetSpace().GetEntitiesByType("foot-bot");
		blackSpotCount = 0;
		whiteSpotCount = 0;
		outsideSpotCount = 0;
		for (CSpace::TMapPerType::iterator it = cFBMap.begin();
				it != cFBMap.end(); ++it) {
			CFootBotEntity& footbotEntity = *any_cast<CFootBotEntity*>(
					it->second);
			CFootBotAggregation& controller =
					static_cast<CFootBotAggregation&>(footbotEntity.GetControllableEntity().GetController());
			string state = controller.GetStateStep();
			if (state == "STAY") {
				Real Robot_X =
						footbotEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
				Real Robot_Y =
						footbotEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
				positions.push_back(make_pair(Robot_X, Robot_Y));

//			m_cOutFile << "    " << Robot_X << " \t         " << Robot_Y
//					<< endl;

				cFootbotPosition.Set(Robot_X, Robot_Y);

				Real fDistanceSpotBlack =
						(m_cCoordBlackSpot - cFootbotPosition).Length();
				if (fDistanceSpotBlack <= m_fRadius) {
					blackSpotCount += 1;

				}

				Real fDistanceSpotWhite =
						(m_cCoordWhiteSpot - cFootbotPosition).Length();
				if (fDistanceSpotWhite <= m_fRadius) {
					whiteSpotCount += 1;

				}

				Real fDistanceSpotWhite2 = (m_cCoordWhiteSpot2
						- cFootbotPosition).Length();
				if (fDistanceSpotWhite2 <= m_fRadius) {
					whiteSpotCount += 1;

				}

				if (fDistanceSpotBlack > m_fRadius
						&& fDistanceSpotWhite > m_fRadius) {
					outsideSpotCount += 1;

				}
			}

			//int spotCount= controller.CountNeighbours();
			//m_cOutFile << " spotCount:" << spotCount << " " << endl;

//			lexi = controller.CountNeighbours();
//			for (int i = 0; i < lexi.size(); ++i)
//				words.insert(lexi[i]);
//
		}

		m_cOutFile << clock << "	" << blackSpotCount / cFBMap.size() << "	"
				<< whiteSpotCount / cFBMap.size() << endl;


//		m_cOutFile << " \t    TimeStep: " << clock
//				<< "     BlackSpot: " << blackSpotCount
//				<< "     WhiteSpot: " << whiteSpotCount
//				<< "     OutsideSpot: " << outsideSpotCount << endl;

		//m_cOutFile << "   whiteSpotCount: " << whiteSpotCount << endl;

		//m_cOutFile << "   m_fObjectiveFunction: " << outsideSpotCount << endl;

		vector<int> sizes;
		vector<double> stds;
		clustersInfo(positions, sizes, stds);

		int sizeAcc = 0;
		double stdAcc = 0;
		int nb = sizes.size();
		for (int i = 0; i < nb; ++i) {
			sizeAcc += sizes[i];
			stdAcc += stds[i];
		}

//		m_cOutFile << connectivity(positions) << " " << words.size() << " "
//				<< sizes.size() << " " << (float) sizeAcc / nb << " "
//				<< stdAcc / nb;
//		for (int i = 0; i < nb; ++i)
//			m_cOutFile << " " << sizes[i];
//		m_cOutFile << endl;
	}
}

bool CAggregation::IsExperimentFinished() {
	if (timeStopCond > 0) {
		bool stabilised = true;
		CSpace::TMapPerType& cFBMap = GetSpace().GetEntitiesByType("foot-bot");
		for (CSpace::TMapPerType::iterator it = cFBMap.begin();
				it != cFBMap.end() and stabilised; ++it) {
			CFootBotEntity& footbotEntity = *any_cast<CFootBotEntity*>(
					it->second);
			CFootBotAggregation& footbot =
					static_cast<CFootBotAggregation&>(footbotEntity.GetControllableEntity().GetController());
			if (footbot.LastMove() < timeStopCond)
				stabilised = false;
		}
		return stabilised;
	}
	return false;
}

argos::CColor CAggregation::GetFloorColor(
		const argos::CVector2& c_position_on_plane) {

//2 spots
//	CVector2 vCurrentPoint(c_position_on_plane.GetX(),
//			c_position_on_plane.GetY());
//	Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
//	if (d <= m_fRadius) {
//		return CColor::BLACK;
//	}
//	d = (vCurrentPoint - m_cCoordWhiteSpot).Length();
//	if (d <= m_fRadius) {
//		return CColor::WHITE;
//	}
//
//	return CColor::GRAY50;

//3 spots
	CVector2 vCurrentPoint(c_position_on_plane.GetX(),
			c_position_on_plane.GetY());
	Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
	if (d <= m_fRadius) {
		return CColor::BLACK;
	}
	d = (vCurrentPoint - m_cCoordWhiteSpot).Length();
	if (d <= m_fRadius) {
		return CColor::WHITE;
	}
	d = (vCurrentPoint - m_cCoordWhiteSpot2).Length();
	if (d <= (m_fRadius)) {
		return CColor::WHITE;
	}

	return CColor::GRAY50;

// 4 spots
//	CVector2 vCurrentPoint(c_position_on_plane.GetX(),
//			c_position_on_plane.GetY());
//	Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
//	if (d <= m_fRadius) {
//		return CColor::BLACK;
//	}
//	d = (m_cCoordBlackSpot2 - vCurrentPoint).Length();
//	if (d <= m_fRadius) {
//		return CColor::WHITE;
//	}
//	d = (vCurrentPoint - m_cCoordWhiteSpot).Length();
//	if (d <= m_fRadius) {
//		return CColor::WHITE;
//	}
//	d = (vCurrentPoint - m_cCoordWhiteSpot2).Length();
//	if (d <= (m_fRadius)) {
//		return CColor::WHITE;
//	}
//
//	return CColor::GRAY50;
}

/****************************************/
/****************************************/

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CAggregation, "aggregation_loop_functions");
