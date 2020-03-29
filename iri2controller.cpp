/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "lightsensor.h"
#include "bluelightsensor.h"
#include "redlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "redbatterysensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri2controller.h"


/******************************************************************************/
/******************************************************************************/

extern gsl_rng* rng;
extern long int rngSeed;
/******************************************************************************/
/******************************************************************************/

using namespace std;
/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS 6

#define ESQUIVAR_PRIORITY 0
#define GASOLINA_PRIORITY 1
#define FUEGO_PRIORITY 2
#define REFUERZO_PRIORITY 3
#define GATO_PRIORITY	4
#define SERVICIO_PRIORITY 5


#define PROXIMITY_THRESHOLD 0.5
#define BATTERY_THRESHOLD 0.5

#define SPEED 1000.0


/******************************************************************************/
/******************************************************************************/
CIri2Controller::CIri2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;	
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);
	/* Set light Switch Sensor */
	m_seLight = (CLightSensor*) m_pcEpuck->GetSensor(SENSOR_LIGHT);
	/* Set blue light Switch Sensor */
        m_seBlueLight =(CBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_BLUE_LIGHT);
	/* Set red light sensor*/
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);

	/* Initilize Variables */

	/* Velocidad de las ruedas */
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;
        /* Número  de luces amarillas */
	m_NLight = 0.0;
	/* Número  de luces azules */
	m_NBlueLight = 0.0;
	/* Zona gris = 1.0, desactiva Zona negra = 0.0 */
	memory=0.0;
	memoryanterior = 0.0;
        /* Veces que deja el objeto en la zona negra */
	totalrecogidos = 0.0;
	/* Transporta un objeto ahora: 1.0 si, 0.0 no*/
	objetorecogido = 0.0;
	/* Bateria */
	bateria=1.0;

	/* Create TABLE for the COORDINATOR */
	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}
}

/******************************************************************************/
/******************************************************************************/

CIri2Controller::~CIri2Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}
}

/******************************************************************************/
	/***AQUI EMPIEZA EL CODIGO A PROGRAMAR***/

void CIri2Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

	
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}
	/* Release Inhibitors */
	fBattInhibitor = 1.0;
	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);
	/* Metodos llamados */
	Esquivar ( ESQUIVAR_PRIORITY );
  	Gasolina ( GASOLINA_PRIORITY );
	Fuego(FUEGO_PRIORITY);
	Refuerzo(REFUERZO_PRIORITY);
	Gato( GATO_PRIORITY );	
	Servicio ( SERVICIO_PRIORITY );
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::Coordinator ( void )
{
	int nBehavior;
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
	{
		if ( m_fActivationTable[nBehavior][2] == 1.0 )
		{	
			break;
		}
	}

	m_fLeftSpeed = m_fActivationTable[nBehavior][0];
	m_fRightSpeed = m_fActivationTable[nBehavior][1];
	
  printf("Comp:%d Batt:%2.4f Fuegos:%1.0f Accidentes:%1.0f Gato:%1.0f Gatos rescatados:%1.0f \n", nBehavior,bateria, m_NLight, m_NBlueLight, objetorecogido , totalrecogidos );
	printf("\n");	

  if (m_nWriteToFile ) 
	{
		// INIT: WRITE TO FILES 
		// Write coordinator ouputs 
		FILE* fileOutput = fopen("outputFiles/coordinator2", "a");
		fprintf(fileOutput,"%2.4f %d  \n", m_fTime, nBehavior);
		fclose(fileOutput);
		// END WRITE TO FILES 
	}
}

/******************************************************************************/
/******************************************************************************/

/**FUNCION AÑADIDA**/

//Atiende a las luces azules y las apaga

void CIri2Controller::Refuerzo( unsigned int un_priority ){
	
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	totalBlueLight = 0;
	TOTBlueLight = 0;
	totalBlueLight = bluelight[0]+bluelight[7];
	TOTBlueLight = bluelight[0]+bluelight[1]+bluelight[2]+bluelight[3]+bluelight[4]+bluelight[5]+bluelight[6]+bluelight[7];
	memory=groundMemory[0];
	
	if (fBattInhibitor == 1.0 ){
			
			if ( totalBlueLight >= 0.8)
			{
				m_seBlueLight->SwitchNearestLight(0);
				m_NBlueLight=m_NBlueLight+1;
				m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);
				m_fActivationTable[un_priority][2] = 1.0;
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
			}
			
			/* GO Light */
			if ( (bluelight[0] * bluelight[7] == 0.0) && (TOTBlueLight != 0))
			{
				
				/* Calc light intensity at the left and right */
				double bluelightLeft = bluelight[0] + bluelight[1] + bluelight[2] + bluelight[3]; //sensores por la izquierda
				double bluelightRight= bluelight[4] + bluelight[5] + bluelight[6] + bluelight[7]; // sensores por la derecha

				/* If light on the left */
				if ( bluelightLeft > bluelightRight )
				{
					//if(){
						
					//}
					/* Turn left */
					//m_acWheels->SetSpeed(-500,500); 	// gira a la izquierda
					m_fActivationTable[un_priority][2] = 1.0;
					m_fActivationTable[un_priority][0] = -SPEED;
					m_fActivationTable[un_priority][1] = SPEED;
					
				}
				else if(bluelightLeft < bluelightRight)
				{
					/* Turn right */
					//m_acWheels->SetSpeed(500,-500);		// gira a la derecha
					m_fActivationTable[un_priority][2] = 1.0;
					m_fActivationTable[un_priority][0] = SPEED;
					m_fActivationTable[un_priority][1] = -SPEED;
				}
				else{	//recto
					m_fActivationTable[un_priority][2] = 1.0;
					m_fActivationTable[un_priority][0] = SPEED;
					m_fActivationTable[un_priority][1] = SPEED;
				}
			}
				
			if((bluelight[0] * bluelight[7] != 0.0) && (TOTBlueLight != 0)){
				m_fActivationTable[un_priority][2] = 1.0;
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
			}
		
	}
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/luzazul2", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, bluelight[0], bluelight[1], bluelight[2], bluelight[3], bluelight[4], bluelight[5], bluelight[6], bluelight[7]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}


/******************************************************************************/
/******************************************************************************/

/**FUNCION AÑADIDA**/

//Se dirige hacia las luces amarillas y las apaga

void CIri2Controller::Fuego( unsigned int un_priority ){

	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);

	totalLight = 0;
	TOTLight = 0;
	
  	totalLight = light[0]+light[7];
	TOTLight = light[0]+light[1]+light[2]+light[3]+light[4]+light[5]+light[6]+light[7];
	memory=groundMemory[0];

	if (fBattInhibitor == 1.0 ){

			if ( totalLight >= 0.95)
			{
				m_seLight->SwitchNearestLight(0);
				m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
				m_fActivationTable[un_priority][2] = 1.0;
				m_NLight = m_NLight +1;
			}
			
			/* GO Light */
			if ( (light[0] * light[7] == 0.0) && (TOTLight != 0))
			{
				
				/* Calc light intensity at the left and right */
				double lightLeft 	= light[0] + light[1] + light[2] + light[3]; //sensores por la izquierda
				double lightRight = light[4] + light[5] + light[6] + light[7];		// sensores por la derecha

				/* If light on the left */
				if ( lightLeft > lightRight )
				{
					/* Turn left */
					//m_acWheels->SetSpeed(-500,500); 	// gira a la izquierda
					m_fActivationTable[un_priority][2] = 1.0;
					m_fActivationTable[un_priority][0] = -SPEED;
					m_fActivationTable[un_priority][1] = SPEED;
					
				}
				else{
					/* Turn right */
										// gira a la derecha
					m_fActivationTable[un_priority][2] = 1.0;
					m_fActivationTable[un_priority][0] = SPEED;
					m_fActivationTable[un_priority][1] = -SPEED;
				}
			}							//recto
			if((light[0] * light[7] != 0.0) && (TOTLight != 0)){
				m_fActivationTable[un_priority][2] = 1.0;
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
			}			
		
	}
	/* INIT WRITE TO FILE */
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/luzamarilla2", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}

}

/******************************************************************************/
/******************************************************************************/

/**FUNCION AÑADIDA**/

//Detecta el choque y se aleja de esa dirección

void CIri2Controller::Esquivar ( unsigned int un_priority )
{
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);
	double* battery = m_seRedBattery->GetSensorReading(m_pcEpuck);
	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;
	memory=groundMemory[0];
	bateria=battery[0];

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += prox[i] * cos ( proxDirections[i] );
		vRepelent.y += prox[i] * sin ( proxDirections[i] );

		if ( prox[i] > fMaxProx )
			fMaxProx = prox[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;
	
	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);


		double fCLinear = 1.0;
		double fCAngular = 1.0;
		double fC1 = SPEED / M_PI;
		
		/* Calc Linear Speed */
		double fVLinear = SPEED * fCLinear * ( cos ( fRepelent / 2) );

		/*Calc Angular Speed */
		double fVAngular = fRepelent;

		m_fActivationTable[un_priority][0] = fVLinear - fC1 * fVAngular;
		m_fActivationTable[un_priority][1] = fVLinear + fC1 * fVAngular;
		m_fActivationTable[un_priority][2] = 1.0;
	}

	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/esquivar2", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

/**FUNCION AÑADIDA**/

//Circula en linea recta a lo largo de la arena.

void CIri2Controller::Servicio ( unsigned int un_priority )
{	
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	memory=groundMemory[0];
	if (fBattInhibitor == 1.0){	
		m_fActivationTable[un_priority][0] = SPEED;
		m_fActivationTable[un_priority][1] = SPEED;
		
	}	
	m_fActivationTable[un_priority][2] = 1.0;

	
	if (m_nWriteToFile ) 
	{
		// INIT: WRITE TO FILES 
		// Write level of competence ouputs 
		FILE* fileOutput = fopen("outputFiles/servicio2", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		// END WRITE TO FILES 
	}
	
}
		
/******************************************************************************/
/******************************************************************************/

/**FUNCION AÑADIDA**/

//Recarga bateria cuando se acerca a la luz roja y si baja del su threshold se dirige alli.

void CIri2Controller::Gasolina ( unsigned int un_priority )
{
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	/* Leer Battery Sensores */
	double* battery = m_seRedBattery->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Luz */
	double* light = m_seRedLight->GetSensorReading(m_pcEpuck);

	/* Calc light intensity at the left and right */
	double lightLeft 	= light[0] + light[1] + light[2] + light[3];
	double lightRight = light[4] + light[5] + light[6] + light[7];
	memory=groundMemory[0];
	bateria=battery[0];

	/* If battery below a BATTERY_THRESHOLD */
	if ( battery[0] < BATTERY_THRESHOLD ){
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
		fBattInhibitor = 0.0;
		/* If not pointing to the light */
		if ( light[0] * light[7] == 0.0 )
		{
			/* Activate level of competence */
			m_fActivationTable[un_priority][2] = 1.0;

			/* If light on the left */
			if ( lightLeft > lightRight ){
				/* Turn left */
				m_fActivationTable[un_priority][0] = -SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
					
			}else{
				/* Turn right */
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = -SPEED;
			}
		}else if(light[0] * light[7] != 0.0){
			m_fActivationTable[un_priority][2] = 1.0;
			m_fActivationTable[un_priority][0] = SPEED;
			m_fActivationTable[un_priority][1] = SPEED;
		}
	}else{
		fBattInhibitor = 1.0;	
	}	

	if (m_nWriteToFile ){
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/gasolina2", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		/* END WRITE TO FILE */
	}
}
/******************************************************************************/
/******************************************************************************/

/**FUNCION AÑADIDA**/

//Detecta si el suelo es gris o negro con el fin de sumar objetos

void CIri2Controller::Gato ( unsigned int un_priority )
{
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	
	// If with a virtual puck 
	if(fBattInhibitor==1.0){
		if(memory == 1.0 && memoryanterior == 0.0){
		objetorecogido = objetorecogido + 1 ;
		memoryanterior = 1.0;
		}
		if ( memory == 1.0 ){
			// Set Leds to BLUE
			m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);	
			m_fActivationTable[un_priority][0] = SPEED;
			m_fActivationTable[un_priority][1] = SPEED;			
			m_fActivationTable[un_priority][2] = 1.0; 1.0;					
		}
		if(memory == 0.0 && memoryanterior == 1.0){
			objetorecogido = objetorecogido - 1 ;
			memoryanterior = 0.0;
			totalrecogidos = totalrecogidos +1;
		}	
	
	}
	
	//FILE
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/gato2", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f  ", m_fTime, fBattInhibitor, groundMemory[0]);
		
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

