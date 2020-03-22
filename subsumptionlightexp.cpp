
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>

/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "random.h"
#include "programmedarena.h"

#include "subsumptionlightexp.h"

/******************** Sensors ******************/
#include "contactsensor.h"
#include "epuckproximitysensor.h"
#include "lightsensor.h"
#include "bluelightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "realbluelightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "subsumptionlightcontroller.h"

using namespace std;

/*Create Arena */
static char* pchHeightMap = 
"%%%%%%%%%%%%%%%%%%%%"
"%##################%"
"%######%%%#########%"
"%##################%"
"%##################%"
"%##################%"
"%############%%%###%"
"%##################%"
"%##%%%%############%"
"%##################%"
"%##################%"
"%##########%%%%%###%"
"%##################%"
"%##################%"
"%##################%"
"%##################%"
"%###%%%#####%%%%###%"
"%##################%"
"%##################%"
"%%%%%%%%%%%%%%%%%%%%";


extern gsl_rng* rng;
extern long int rngSeed;

/*******************************************************************************/
///*******************************************************************************/
//
CSubsumptionLightExp::CSubsumptionLightExp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{
	
	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{	

		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);
		
		

		m_nRunTime = 10000;;
		m_pcvRobotPositions = new dVector2[m_nRobotsNumber];
		m_fRobotOrientations = new double[m_nRobotsNumber];
		for ( int i = 0 ; i < m_nRobotsNumber ; i++)
		{
			m_pcvRobotPositions[i].x 	= -0.0;
			m_pcvRobotPositions[i].y 	= -0.0;
			m_fRobotOrientations[i] 	= 0.0;
		}
	
		m_fLightSensorRange = 0.8; // meters
		m_fBlueLightSensorRange = 0.8;
		m_nNumberOfLightObject = 8;
		m_nNumberOfBlueLightObject = 3;
		m_pcvLightObjects = new dVector2[m_nNumberOfLightObject];

		m_pcvBlueLightObjects = new dVector2[m_nNumberOfBlueLightObject];

		m_pcvLightObjects[0].x = 0.5;
		m_pcvLightObjects[0].y = -0.75;
		m_pcvLightObjects[1].x = -0.5;
		m_pcvLightObjects[1].y = -0.5;
		m_pcvLightObjects[2].x = -0.5;
		m_pcvLightObjects[2].y = 0.5;
		m_pcvLightObjects[3].x = 0.5;
		m_pcvLightObjects[3].y = 0.5;
		m_pcvLightObjects[4].x = 0.90;
		m_pcvLightObjects[4].y = 0.0;
		//AZULES
		m_pcvLightObjects[5].x   = 0.1;
		m_pcvLightObjects[5].y = 0.2;

		m_pcvLightObjects[6].x   = 0.4;
		m_pcvLightObjects[6].y = 0.6;

		m_pcvLightObjects[7].x   = 0.9;
		m_pcvLightObjects[7].y = -0.3;		


		m_nNumberOfGroundArea = 0;
		m_vGroundAreaCenter = new dVector2[m_nNumberOfGroundArea];
		m_fGroundAreaExternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaInternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaColor = new double[m_nNumberOfGroundArea];
	}
	/* Else, extract info from the file */
	else
	{
		ifstream pfile(paramsFile);
		if(!pfile) {
			cerr << "Can't find parameters file " << endl;
			exit(0);
		}	

		/* EXTRA */
		
		/* Get number of robots */
		m_nRobotsNumber = getInt('=',pfile);
		/* Set numer of robots */
		SetNumberOfEpucks(m_nRobotsNumber);
		/* For each robot get position and orientation */
		m_pcvRobotPositions = new dVector2[m_nRobotsNumber];
		m_fRobotOrientations = new double[m_nRobotsNumber];
		for ( int i = 0 ; i < m_nRobotsNumber ; i++)
		{
			m_pcvRobotPositions[i].x 	= getDouble('=',pfile);
			m_pcvRobotPositions[i].y 	= getDouble('=',pfile);
			m_fRobotOrientations[i] 	= getDouble('=',pfile);
		}
		
		/* Get write to file flag */
		m_nWriteToFile 	= getInt('=',pfile);
		/* Get Run Time */
		m_nRunTime = getInt('=',pfile);

		/* ENVIRONMENT */
		
		/* Lights */
		/* Get Light Objects Number */
		m_nNumberOfLightObject = getInt('=',pfile);
  		m_nNumberOfBlueLightObject = getInt('=',pfile);
		/* Create Objects */
		m_pcvLightObjects = new dVector2[m_nNumberOfLightObject];
		
		for ( int i = 0 ; i < m_nNumberOfLightObject; i++){
			/* Get X position */
			m_pcvLightObjects[i].x = getDouble('=',pfile);
			
			/* Get Y Position */
			m_pcvLightObjects[i].y = getDouble('=',pfile);
			
		}


		/*m_pcvBlueLightObjects = new dVector2[m_nNumberOfBlueLightObject];
		for ( int i = 0 ; i < m_nNumberOfBlueLightObject; i++){

		m_pcvBlueLightObjects[i].x = getDouble('=',pfile);
		m_pcvBlueLightObjects[i].y = getDouble('=',pfile);

		}*/
		/* Ground Areas */
		/* Get GroundArea Objects */
		m_nNumberOfGroundArea = getInt('=',pfile);
		m_vGroundAreaCenter = new dVector2[m_nNumberOfGroundArea];
		m_fGroundAreaExternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaInternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaColor = new double[m_nNumberOfGroundArea];

		for ( int i = 0 ; i < m_nNumberOfGroundArea ; i++)
		{
			m_vGroundAreaCenter[i].x = getDouble('=',pfile);
			m_vGroundAreaCenter[i].y = getDouble('=',pfile);
			m_fGroundAreaInternalRadius[i] = 0.0;
			m_fGroundAreaExternalRadius[i] = getDouble('=',pfile);
			m_fGroundAreaColor[i] = getDouble('=',pfile);

		}
		
		
		/* SENSORS */
		/* Get Light Range */

		m_fLightSensorRange = getDouble('=',pfile); 

		m_fBlueLightSensorRange = getDouble('=',pfile); 
		/* Get Battery load range */
		m_fBatterySensorRange = getDouble('=',pfile);
		/* Get batttery charge coef */
		m_fBatteryChargeCoef = getDouble('=',pfile);
		/* Get batttery charge coef */
		m_fBatteryDischargeCoef = getDouble('=',pfile);
		
		/* MORPHOLOGY */
	}
}

/******************************************************************************/
/******************************************************************************/

CSubsumptionLightExp::~CSubsumptionLightExp ( void )
{
	delete [] m_pcvLightObjects;
	delete [] m_vGroundAreaCenter;
	delete [] m_fGroundAreaExternalRadius;
	delete [] m_fGroundAreaInternalRadius;
	delete [] m_fGroundAreaColor;
}

	/******************************************************************************/
/******************************************************************************/
CArena* CSubsumptionLightExp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	/* Create and add Light Object */
	char pchTemp[128];
	CLightObject* pcLightObject = NULL;
	CBlueLightObject* pcBlueLightObject = NULL;
	for( int i = 0 ; i < m_nNumberOfLightObject ; i++){
		if( i <5 ){sprintf(pchTemp, "LightObject%d", i);
		CLightObject* pcLightObject = new CLightObject (pchTemp);
		pcLightObject->SetCenter(m_pcvLightObjects[i]);
		pcArena->AddLightObject(pcLightObject);}
		if(i >= 5){

		sprintf(pchTemp, "BlueLightObject%d", i);
		CBlueLightObject* pcBlueLightObject = new CBlueLightObject (pchTemp);
		pcBlueLightObject->SetCenter(m_pcvLightObjects[i]);
		pcArena->AddBlueLightObject(pcBlueLightObject);
	}


	}
	
	/*CBlueLightObject* pcBlueLightObject = NULL;
	for( int i = 0 ; i < m_nNumberOfBlueLightObject ; i++){
		
	}*/
	
	





	/* Create GroundArea */
	char sGroundAreaName[100]="epuck";


	for ( int i = 0 ; i < m_nNumberOfGroundArea ; i++)
	{
		//Create GroundArea
		sprintf(sGroundAreaName,"groundArea%d",i);
		CGroundArea* groundArea = new CGroundArea(sGroundAreaName);
		groundArea->SetCenter(m_vGroundAreaCenter[i]);
		groundArea->SetExtRadius(m_fGroundAreaExternalRadius[i]);
		groundArea->SetIntRadius(m_fGroundAreaInternalRadius[i]);
		groundArea->SetColor(m_fGroundAreaColor[i]);
		groundArea->SetHeight(0.20);
		pcArena->AddGroundArea(groundArea);
	}

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CSubsumptionLightExp::AddActuators(CEpuck* pc_epuck)
{
	/* Create and Add Wheels */
	char pchTemp[128];
	sprintf(pchTemp, "actuators_%s", pc_epuck->GetName());
	CActuator* pcActuator = NULL;
	pcActuator = new CWheelsActuator(pchTemp, pc_epuck);
	pc_epuck->AddActuator(pcActuator);
}

/******************************************************************************/
/******************************************************************************/

void CSubsumptionLightExp::AddSensors(CEpuck* pc_epuck)
{
	//
	/* Create and add Proximity Sensor */
	CSensor* pcProxSensor = NULL;
	pcProxSensor = new CEpuckProximitySensor(252);
	pc_epuck->AddSensor(pcProxSensor);

	//Light Sensor
	CSensor* pcLightSensor = NULL;
	pcLightSensor = new CLightSensor("Light Sensor", m_fLightSensorRange);
	pc_epuck->AddSensor(pcLightSensor);
	
	//Contact Sensor
	CSensor* pcContactSensor = NULL;
	pcContactSensor = new CContactSensor("Contact Sensor");
	pc_epuck->AddSensor(pcContactSensor);
	
	//Ground Sensor
	CSensor* pcGroundSensor = NULL;
	pcGroundSensor = new CGroundSensor("Ground Sensor");
	pc_epuck->AddSensor(pcGroundSensor);
	
	//Ground Memory Sensor
	CSensor* pcGroundMemorySensor = NULL;
	pcGroundMemorySensor = new CGroundMemorySensor("Ground Memory Sensor");
	pc_epuck->AddSensor(pcGroundMemorySensor);
	
	//Battery Sensor
	CSensor* pcBatterySensor = NULL;
	pcBatterySensor = new CBatterySensor("Battery Sensor", m_fBatterySensorRange, m_fBatteryChargeCoef, m_fBatteryDischargeCoef);
	pc_epuck->AddSensor(pcBatterySensor);
	/* Create and add Light Sensor */
	CSensor* pcBlueLightSensor = NULL;
	pcBlueLightSensor = new CBlueLightSensor("Blue Light Sensor", m_fLightSensorRange);
	pc_epuck->AddSensor(pcBlueLightSensor);
}

/******************************************************************************/
/******************************************************************************/

void CSubsumptionLightExp::SetController(CEpuck* pc_epuck)
{
	char pchTemp[128];
	sprintf(pchTemp, "Iri1");
	CController* pcController = new CSubsumptionLightController(pchTemp, pc_epuck, m_nWriteToFile);
	pc_epuck->SetControllerType( CONTROLLER_SUBSUMPTION_LIGHT );
	pc_epuck->SetController(pcController);

	
}

/******************************************************************************/
/******************************************************************************/

void CSubsumptionLightExp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, m_pcvRobotPositions[i].x, m_pcvRobotPositions[i].y, m_fRobotOrientations[i]);
		pc_simulator->AddEpuck(pcEpuck);
		pc_simulator->SetTimeLimit(m_nRunTime);
	}

	Reset();
}


/******************************************************************************/
/******************************************************************************/

void CSubsumptionLightExp::Reset ( void )
{
}
