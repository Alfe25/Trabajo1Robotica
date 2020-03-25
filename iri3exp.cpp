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

#include "iri3exp.h"

/******************** Sensors ******************/
#include "contactsensor.h"
#include "epuckproximitysensor.h"
#include "lightsensor.h"
#include "bluelightsensor.h"
#include "lightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "redbatterysensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "iri3controller.h"

using namespace std;

/*Create Arena */
static char* pchHeightMap = 
"%%%%%%%%%%%%%%%%%%%%"
"%##################%"
"%##################%"
"%##################%"
"%####%%%###########%"
"%##################%"
"%############%%%###%"
"%##################%"
"%####%%%###########%"
"%##################%"
"%##################%"
"%##########%%%%%###%"
"%##################%"
"%##################%"
"%##################%"
"%###########%%%####%"
"%####%%############%"
"%##################%"
"%##################%"
"%%%%%%%%%%%%%%%%%%%%";


extern gsl_rng* rng;
extern long int rngSeed;

/*******************************************************************************/
///*******************************************************************************/
//
CIri3Exp::CIri3Exp(const char* pch_name, const char* paramsFile) :
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
			m_pcvRobotPositions[i].x 	= 0.5;
			m_pcvRobotPositions[i].y 	= -0.9;
			m_fRobotOrientations[i] 	= 0.0;
		}
	
		m_fLightSensorRange = 0.8; // meters
		m_fBlueLightSensorRange = 1.5;
		m_fRedLightSensorRange = 4;
		m_nNumberOfLightObject = 8;

		//Parámetros de bateria
		m_fRedBatterySensorRange=0.5;
		m_fRedBatteryChargeCoef=0.01;
		m_fRedBatteryDischargeCoef=0.0005;
		
		m_pcvLightObjects = new dVector2[m_nNumberOfLightObject];


		m_pcvLightObjects[0].x = 0.0;
		m_pcvLightObjects[0].y = -1.1;
		m_pcvLightObjects[1].x = -0.95;
		m_pcvLightObjects[1].y = -0.2;
		m_pcvLightObjects[2].x = -0.5;
		m_pcvLightObjects[2].y = 0.5;
		m_pcvLightObjects[3].x = 0.25;
		m_pcvLightObjects[3].y = 0.5;
		m_pcvLightObjects[4].x = 0.90;
		m_pcvLightObjects[4].y = 0.0;

		//AZULES
		m_pcvLightObjects[5].x = 0.95;
		m_pcvLightObjects[5].y = 0.95;

		m_pcvLightObjects[6].x = -0.95;
		m_pcvLightObjects[6].y = -0.0;

		//Rojo
		m_pcvLightObjects[7].x = -0.95;  
		m_pcvLightObjects[7].y = 0.95;	

		m_nNumberOfGroundArea = 2;
		m_vGroundAreaCenter = new dVector2[m_nNumberOfGroundArea];
		m_vGroundAreaCenter[0].x = 0.9;
		m_vGroundAreaCenter[0].y = -0.9;
		m_vGroundAreaCenter[1].x = -0.9;
		m_vGroundAreaCenter[1].y = 0.85;

		m_fGroundAreaExternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaExternalRadius[0]=0.5;
		m_fGroundAreaExternalRadius[1]=0.5;

		m_fGroundAreaInternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaInternalRadius[0]=1;
		m_fGroundAreaInternalRadius[1]=1;

		m_fGroundAreaColor = new double[m_nNumberOfGroundArea];
		m_fGroundAreaColor[0]=(0.0);
		m_fGroundAreaColor[1]=(0.5);
		
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
  		
		/* Create Objects */
		m_pcvLightObjects = new dVector2[m_nNumberOfLightObject];
		
		for ( int i = 0 ; i < m_nNumberOfLightObject; i++){
			/* Get X position */
			m_pcvLightObjects[i].x = getDouble('=',pfile);
			
			/* Get Y Position */
			m_pcvLightObjects[i].y = getDouble('=',pfile);
			
		}

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
		m_fRedLightSensorRange = getDouble('=',pfile); 
		m_fBlueLightSensorRange = getDouble('=',pfile); 
		/* Get Battery load range */
		m_fRedBatterySensorRange = getDouble('=',pfile);
		/* Get batttery charge coef */
		m_fRedBatteryChargeCoef = getDouble('=',pfile);
		/* Get batttery charge coef */
		m_fRedBatteryDischargeCoef = getDouble('=',pfile);
		
		/* MORPHOLOGY */
	}
}

/******************************************************************************/
/******************************************************************************/

CIri3Exp::~CIri3Exp ( void )
{
	delete [] m_pcvLightObjects;
	delete [] m_vGroundAreaCenter;
	delete [] m_fGroundAreaExternalRadius;
	delete [] m_fGroundAreaInternalRadius;
	delete [] m_fGroundAreaColor;
}

/******************************************************************************/
/******************************************************************************/
CArena* CIri3Exp::CreateArena()
{
	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	/* Create and add Light Object */
	char pchTemp[128];
	CLightObject* pcLightObject = NULL;
	CBlueLightObject* pcBlueLightObject = NULL;
	CRedLightObject* pcRedLightObject = NULL;
	for( int i = 0 ; i < m_nNumberOfLightObject ; i++){
		if( i < 5 ){
			sprintf(pchTemp, "LightObject%d", i);
			CLightObject* pcLightObject = new CLightObject (pchTemp);
			pcLightObject->SetCenter(m_pcvLightObjects[i]);
			pcArena->AddLightObject(pcLightObject);}
		if(i >= 5 && i <= 6){
			sprintf(pchTemp, "BlueLightObject%d", i);
			CBlueLightObject* pcBlueLightObject = new CBlueLightObject (pchTemp);
			pcBlueLightObject->SetCenter(m_pcvLightObjects[i]);
			pcArena->AddBlueLightObject(pcBlueLightObject);}
	/* LUZ ROJA */
		if(i > 6) {

			sprintf(pchTemp, "RedLightObject%d", i);
			CRedLightObject* pcRedLightObject = new CRedLightObject (pchTemp);
			pcRedLightObject->SetCenter(m_pcvLightObjects[i]);
			pcArena->AddRedLightObject(pcRedLightObject);
	}


	}


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

void CIri3Exp::AddActuators(CEpuck* pc_epuck)
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

void CIri3Exp::AddSensors(CEpuck* pc_epuck)
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
	CSensor* pcRedBatterySensor = NULL;
	pcRedBatterySensor = new CRedBatterySensor("Battery Sensor", m_fRedBatterySensorRange, m_fRedBatteryChargeCoef, m_fRedBatteryDischargeCoef);
	pc_epuck->AddSensor(pcRedBatterySensor);

	/* Create and add Blue Light Sensor */
	CSensor* pcBlueLightSensor = NULL;
	pcBlueLightSensor = new CBlueLightSensor("Blue Light Sensor", m_fBlueLightSensorRange);
	pc_epuck->AddSensor(pcBlueLightSensor);

	/* Create and add Red Light Sensor */
	CSensor* pcRedLightSensor = NULL;
	pcRedLightSensor = new CRealRedLightSensor("Red Light Sensor", m_fRedLightSensorRange);
	pc_epuck->AddSensor(pcRedLightSensor);
}

/******************************************************************************/
/******************************************************************************/

void CIri3Exp::SetController(CEpuck* pc_epuck)
{
	char pchTemp[128];
	sprintf(pchTemp, "Iri3");
	CController* pcController = new CIri3Controller(pchTemp, pc_epuck, m_nWriteToFile);
	pc_epuck->SetControllerType( CONTROLLER_IRI3 );
	pc_epuck->SetController(pcController);

	
}

/******************************************************************************/
/******************************************************************************/

void CIri3Exp::CreateAndAddEpucks(CSimulator* pc_simulator)
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

void CIri3Exp::Reset ( void )
{
}
