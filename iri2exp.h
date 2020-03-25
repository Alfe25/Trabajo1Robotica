#ifndef IRI2EXP_H
#define IRI2EXP_H

/******************************************************************************/
/******************************************************************************/

#include "experiment.h"
/******************************************************************************/
/******************************************************************************/

class CIri2Exp : public CExperiment
{
public:
    CIri2Exp ( const char* pch_name , const char* paramsFile );
		~CIri2Exp ( void );
protected:
    // Overwritten from the superclasses:
    CArena* CreateArena();
	void AddActuators(CEpuck* pc_epuck);
    void AddSensors(CEpuck* pc_epuck);
    void SetController(CEpuck* pc_epuck);
    void CreateAndAddEpucks(CSimulator* pc_simulator);
   	void Reset(); 
private:
		/* VARIABLES*/
		/* Extra */
		int m_nRobotsNumber;
		int m_nWriteToFile;
		dVector2* m_pcvRobotPositions;
		double* m_fRobotOrientations;
		int m_nRunTime;

		/* Environment */
		int m_nNumberOfLightObject;
		dVector2 *m_pcvLightObjects;
		int m_nNumberOfGroundArea;
		dVector2* m_vGroundAreaCenter;
		double* m_fGroundAreaExternalRadius;
		double * m_fGroundAreaInternalRadius;
		double * m_fGroundAreaColor;

		/* Sensors */
		float m_fLightSensorRange;
		float m_fBlueLightSensorRange;
		float m_fRedLightSensorRange;
		double m_fRedBatterySensorRange;
		double m_fRedBatteryChargeCoef;
		double m_fRedBatteryDischargeCoef;

};

/******************************************************************************/
/******************************************************************************/

#endif
