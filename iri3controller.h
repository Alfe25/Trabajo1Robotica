#ifndef IRI3CONTROLLER_H_
#define IRI3CONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"


/******************************************************************************/
/******************************************************************************/

class CIri3Controller : public CController
{
public:

    CIri3Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);

    ~CIri3Controller();

    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
		/* ROBOT */
    CEpuck* m_pcEpuck;
   
	 	/* SENSORS */
		CWheelsActuator* m_acWheels;
    		CEpuckProximitySensor* m_seProx;
		CLightSensor* m_seLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CRedBatterySensor* m_seRedBattery;   
		CBlueLightSensor* m_seBlueLight;
		CRealRedLightSensor* m_seRedLight;
		
		/* Global Variables */
		double 	m_fLeftSpeed;
		double 	m_fRightSpeed;
		double** m_fActivationTable;
		double 	m_fTime;
		double	m_NLight;
		double	m_NBlueLight;
		double fBattInhibitor;
		double totalBlueLight;
		double totalLight;
		double TOTBlueLight;
		double TOTLight;
		double memoryanterior;
		double objetorecogido;
		double totalrecogidos;
		double memory;
		int m_nWriteToFile;
		double bateria;

		/* Functions */

		void ExecuteBehaviors ( void );
		void Coordinator ( void );
		void Fuego( unsigned int un_priority );
		void Refuerzo( unsigned int un_priority );
		void Gato ( unsigned int un_priority );
		void Esquivar ( unsigned int un_priority );
		void Gasolina ( unsigned int un_priority );
		void Servicio ( unsigned int un_priority );
};

#endif
