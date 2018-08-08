#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
//#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include "controllers/footbot_aggregation/footbot_aggregation.h"
#include "argos3/plugins/simulator/entities/box_entity.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <utility>
#include <list>
#include <math.h>


using namespace argos;

class CAggregation : public CLoopFunctions {

public:


	typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
	   TWaypointMap m_tWaypoints;
   /**
    * Class constructor
    */
   CAggregation();

   /** 
    * Class destructor
    */
   virtual ~CAggregation();

   /**
    * Initializes the experiment.
    * It is executed once at the beginning of the experiment, i.e., when ARGoS is launched.
    * @param t_tree The parsed XML tree corresponding to the <loop_functions> section.
    */
   virtual void Init(TConfigurationNode& t_tree);

   /**
    * Resets the experiment to the state it was right after Init() was called.
    * It is executed every time you press the 'reset' button in the GUI.
    */
   virtual void Reset();

   /**
    * Undoes whatever Init() did.
    * It is executed once when ARGoS has finished the experiment.
    */
   virtual void Destroy();

   /**
    * Performs actions right before a simulation step is executed.
    */
   virtual void PreStep();
   
   list< pair<float,float> > findCluster(list< pair<float,float> >::iterator seed, list< pair<float,float> >& pos);
   double std2D(list< pair<float,float> > pos);
   float connectivity(list< pair<float,float> > pos);
   int clustersInfo(list< pair<float,float> > pos, vector<int>& sizes, vector<double>& stds);

   virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);

   /**
    * Performs actions right after a simulation step is executed.
    */
   virtual void PostStep();
   
   /**
    * Determines the end of the experiment.
    */
   virtual bool IsExperimentFinished();

private:


private:   /**
    * The path of the output file.
    */
   std::string m_strOutFile;

   /**
    * The stream associated to the output file.
    */
   std::ofstream m_cOutFile;

   /**
    * This vector contains a list of positions of objects in the construction area
    */
   std::vector<CVector3> m_vecConstructionObjectsInArea;

   /**
    * Minimum and maximum Y coordinate for the objects in the construction area
    */
   Real m_fMinObjectY, m_fMaxObjectY;
   
   Real m_fRadius;
   CVector2 m_cCoordBlackSpot;
   CVector2 m_cCoordBlackSpot2;
   CVector2 m_cCoordWhiteSpot;
   CVector2 m_cCoordWhiteSpot2;
   unsigned int minDist;
   unsigned int timeStopCond;

   Real blackSpotCount;
   Real whiteSpotCount;
   Real outsideSpotCount;
   Real blackSpotVector;
   Real whiteSpotVector;

};
