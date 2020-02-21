/*
 * Astromobile.h
 * Implémentation du squelette de la classe Car
 * Créer par: William Gauvin
 * */

#ifndef SRC_ASTROMOBILE_H_
#define SRC_ASTROMOBILE_H_
#include <iostream>
#include <stdio.h>			/* printf */
#include <stdlib.h> 		/* EXIT_SUCCESS */
#include <stdint.h>     	/* int32_t uint32_t */
#include <pthread.h>    	/* pthread_t pthread_create pthread_join */
#include <semaphore.h> 		/* sem_t sem_init sem_wait sem_post */
#include <errno.h>      	/* errno */
#include <signal.h>     	/* struct sigevent */
#include <sys/neutrino.h>	/* ChannelCreate ConnectAttach MsgReceive */
#include <sys/syspage.h>
#include <sys/netmgr.h>     /* ND_LOCAL_NODE */
#include <time.h>           /* struct itimerspec struct timespec timer_create tier_settime clock_gettime */
#include <vector>
#include <string.h>
#include <mqueue.h>
#include <math.h>
#include <cmath>
#include <random>
#include <cstring>
#include <queue>
#include <stack>
#include <set>
#include "genMap.h"

using namespace std;

//Énumération pour la vitesse
enum E_Vitesse { Fast, Slow, Stop };

//Define utile
#define INIT_NIV_BAT 100
#define DISTANCE_MAX_LAT 2000
#define DISTANCE_MAX_LON 6000
#define TIME_TO_CHARGE_M 20
#define VITESSE_MAX_KMH 50
#define BAT_AUTONOMY_KM 50
#define FAST_SPEED 50.0
#define SLOW_SPEED 30.0
#define PERIOD_TASK_1 100
#define PERIOD_TASK_0 PERIOD_TASK_1/10
#define PERIOD_TASK_IMAGE 400
#define PI 3.141592
#define FAST_SPEED_M_S FAST_SPEED*1000/3600
#define SLOW_SPEED_M_S SLOW_SPEED*1000/3600
#define BATT_LVL_HIGH 80
#define BATT_LVL_LOW 10
#define SPEED_UP_SIM 10
#define AUT_50 50
#define AUT_30 80
#define SLEEP_TIME 100
#define TASK_PULSE_CODE _PULSE_CODE_MINAVAIL
#define IMAGE_TASK_ID -1
#define Small_Rock 10
#define Medium_Rock 20
#define Large_Rock 30

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;
//Cell pour A*
struct cell{
	int parent_i, parent_j;
	double f, g, h;
};
//Fonction tracePath
void tracePath(vector<vector<cell>> cellDetails, pair<int, int> dest, int ID);
//Fonction isValid
bool isValid(int row, int col, int, int);
//Classe du véhicules
class Car{
	public:
		//Get ID
		int Get_Car_ID();

		//Get Batterie lvl
		double Get_Batt_Lvl();

		//Set batterie lvl TODO NE PAS OUBLIER DE METTRE BAT LVL = BATLVL + SLOPE POUR DECREMENTER ET AUGMENTER LE + ou - EST CHANGER
		//DANS LE PASSAGE DE SLOPE!!
		void Set_Batt_Lvl(double slope);

		//Set Station recharge enable if true == enable if false == disable
		void Set_Station_recharge_Enable(bool);

		//Get Station recharge status
		bool Get_Station_Recharge_Status();

		//Get Position Reel
		coord_t Get_PosReal();

		//Set Position Reel
		void Set_PosReal(coord_t);

		//Get la vitesse
		E_Vitesse Get_Vitesse();

		//Set la vitesse
		void Set_Vitesse(E_Vitesse);

		//Get WayPoint
		coord_t Get_WP();

		//Set Waypoint
		void Set_WP(coord_t);

		//Set orientation
		void Set_Orientation(double);

		//Get Orientation
		double Get_Orientation();

		//Get Sleep timer
		int Get_Sleep_Timer();

		//Set Sleep Timer TODO INCREMENT OF 1
		void Set_Sleep_Timer();

		//Get Sleep Status
		bool Get_Sleep_Status();

		//Set Sleep Status
		void Set_Sleep_Status(bool);

		//Set Final Dest Flag
		void Set_Dest_Final_Reach(bool);

		//Get Final Dest Flag
		bool Get_Dest_Final_Reach();

		//Get Destination Final
		coord_t Get_Dest_Final();

		//Set Final Destination
		void Set_Dest_Final(coord_t);
		//Get Initialisation status
		bool Get_Init();

		//Set Init status
		void Set_Init(bool);

		//Reset Sleep Timer
		void Reset_Sleep_Timer();

		//Get WP Reach
		bool Get_WP_Reach();

		//Set WP Reach
		void Set_WP_Reach(bool);

		//Initialisation
		void Initialisation_Car(int i);

		//Print les informations du véhicule
		void Print_Info();

		//Distance parcourrue
		double distance;

		//Fonction qui regarde s'il y a un obstacle sur le chemin
		bool isBlocked(coord_t nPos);

		//Fonction A*
		int aStarSearch(coord_t nPos);

		//Vecteur d'osbtacle
		vector<pair<double, double>> obstacle;

		//variable utiliser pour A*
		bool Use_AStar;
		bool Ongoing_AStar;

		//Queue pour le chemin de A*
		queue<Pair> Path;

	private:
        //Indique si la destination finale a été atteinte
        bool Dest_Final_Reach;

        //Indique si le waypoint a été atteint
        bool Dest_WP_Reach = false;

        //Indique si le véhicule doit se faire chargé
        bool Station_Recharge_Enable = false;

        //Indique si on est dans la 1er passe (soit l'initialisation)
        bool Init_0;

        //Coordonné X & Y pour la position réelle
        coord_t Pos_Real;

        //Coordonné X & Y pour la destination finale
        coord_t Dest;

        //Coordonné X & Y pour le Waypoint
        coord_t WP;

        //Indicateur de vitesse de l'automobile
        E_Vitesse Vitesse;

        //Indicateur du niveau de batterie
        double Batterie_lvl;

        //Orientation du véhicule
        double Orientation;

        //Car ID
        int ID;

        //SLeep Timer
        int Sleep_Timer;

        //Sleep Status
        bool Sleep_Status;

};
#endif /* SRC_ASTROMOBILE_H_ */
