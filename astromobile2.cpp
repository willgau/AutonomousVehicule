/*
 * Astromobile2.cpp
 * Implémentation complète des fonctions de la voitures
 * Créer par: William Gauvin
 * */
#include "astromobile.h"

//Define utile
#define Low_Density IMG_W_PX*0.001
#define Medium_Density IMG_W_PX*0.025
#define High_Density IMG_W_PX*0.01
#define NB_OF_THREAD 5

//Vecteur temporaire pour storer les obstacles
vector<pair<double, double>> myv;

//Densité des obstacles
enum E_Rock_Density{LOW,MEDIUM,HIGH};

//Dimension des obstacles
int Rock_Dimension[3] = { Small_Rock, Medium_Rock, Large_Rock };

//Image
PathMap pm;

//Vecteur de toutes les voitures créer
vector<Car> All_Car;

//Vecteur pour les periods des tâches
vector<int> v_Task_Period = { PERIOD_TASK_0, PERIOD_TASK_1, 2000,PERIOD_TASK_0, PERIOD_TASK_0};

//Mutex de protection
pthread_mutex_t m_Protect_Location = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t m_Protect_Speed = PTHREAD_MUTEX_INITIALIZER;

//Semaphore
sem_t task0_sync;

//sem_t Sem_Alarm_Batt;
sem_t* Sem_Alarm_Batt[8];

//sem_t Protect_Car_ID;
sem_t* Protect_Car_ID[8];
//Alarme et type d'alarme
enum Type_Alarme{Alarme_10, Alarme_80}Alarme;


/* Thread arguments structure */
typedef struct thread_arg {
                sem_t* semaphore; /* Synchronization semaphore pointer */
                uint32_t id; /* Task id */
                uint32_t starttime; /* Global start time */
                int32_t chid; /* Task channel id */
} thread_args_t;

/* Pulse structure */
typedef union pulse_msg{
        struct _pulse pulse;
} pulse_msg_t;


//Vecteur pour la création des threads
//***********************************************
vector<sem_t> v_task_sync;
vector<pthread_t> v_task;
vector<pthread_t> v_task_Alarm;
vector<pthread_t> v_pulse_handler;
vector<sigevent> v_task_event;
vector<itimerspec> v_task_itime;
vector<timer_t> v_task_timer;
vector<thread_args_t> v_task_args;
vector<thread_args_t> v_task_args_Alarm;
//*********************************************
/*
Fonction qui compare deux coordonnés et retourne si elles sont équivalentes

Input:
  Coord_t A;  (Position X & Y)
  Coord_t B;  (Position X & Y)
Output:
  Boolean (True si les positions sont équivalentes, Faux sinon)

Commentaire:
  Les positions sont des doubles qui sont cast en int pour enlever la partie décimal;
  sinon les positions ne seront jamais égales (ex: 1.9999999 != 2.0)
*/
bool Compare_Pos(coord_t A, coord_t B) {
        if(int(A.x) >= int(B.x) - SPEED_UP_SIM && int(A.x) <= int(B.x)+ SPEED_UP_SIM)
                if(int(A.y) >= int(B.y) - SPEED_UP_SIM && int(A.y) <= int(B.y)+ SPEED_UP_SIM)
                        return true;

        return false;
}
/*
Fonction qui compare met dans une queue les positions fournis par l'algorithme A*

Input:
  vector<vector<cell>> cellDetails;  Vecteur 2D de cellules
  pair<int, int> dest;  pair de positions finales
  int ID; ID du véhicule

Commentaire:
Implémentation de A* inspirer du site https://www.geeksforgeeks.org/a-search-algorithm/
*/
void tracePath(vector<vector<cell>> cellDetails, pair<int, int> dest, int ID)
{
	int row = int(dest.first);
	int col = int(dest.second);

	while (!(cellDetails[row][col].parent_i == row
		&& cellDetails[row][col].parent_j == col))
	{
		All_Car[ID].Path.push(make_pair(All_Car[ID].Get_PosReal().x + row, All_Car[ID].Get_PosReal().y + col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
		pm.setPx( All_Car[ID].Get_PosReal().x + row, All_Car[ID].Get_PosReal().y + col, {255,255,255});
	}

	All_Car[ID].Path.push(make_pair(All_Car[ID].Get_PosReal().x + row,All_Car[ID].Get_PosReal().y + col));

}

/*
Fonction qui retourne vrai si les coordonnées sont dans la plage permise

Input:
  int row; la ligne
  int col; la colonne
  int H_MAX; la ligne maximale permise
  int W_MAX; la colonne maximale permise
Output:
  boolean true ou false
*/
bool isValid(int row, int col, int H_MAX, int W_MAX)
{

    return (row >= 0) && (row < H_MAX) &&
           (col >= 0) && (col < W_MAX);
}
/*
Fonction qui fait le dumping de l'image

Input:
  Argument du thread

Commentaire:
  Affinité avec le CPU 8
*/
void* Image_Dumping(void* args){

	struct timespec tp;
    uint32_t task_id;
    uint32_t starttime;
	sem_t* sync_sem;
    uint32_t elapsed_time;
	/* Get the arguments */
	sync_sem = ((thread_args_t*)args)->semaphore;
	int32_t task_chid = ((thread_args_t*)args)->chid;
	task_id = ((thread_args_t*)args)->id;

	//Put on single core (8e)
	int data = 0b0;
	if(task_id == IMAGE_TASK_ID){
		data = 0b10000000;
		ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
	}
	//delay to dump
	int TIME_TO_PRINT = 0;

	 while(1<2) {
		 if(0 == sem_wait(sync_sem)) {
			 /* Get the current time */
			 if(0 == clock_gettime(CLOCK_REALTIME, &tp)) {
				 elapsed_time = tp.tv_sec - starttime;

				 if(TIME_TO_PRINT >= 100){
					 cout << "Image Dumping"<< endl;
					 pm.dumpImage("test.bmp");
					 cout << "End Dumping"<< endl;
					 TIME_TO_PRINT = 0;
				 }
				 TIME_TO_PRINT++;
		     }
		     else {
		    	 /* Print error */
		    	 printf("Task %d could not get time: %d\n",task_id, errno);
		     }
		 }
		 else {
			 printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		 }
	 }
}
/*
Fonction qui gère l'alarme de la batterie

Input:
  Argument du thread

Commentaire:
  Gère les alarmes pour nimporte quel véhicule
*/
void* Alarm_Batt(void* args){

	uint32_t task_id = ((thread_args_t*)args)->id;

	while(true) {

	    if(0 == sem_wait(Sem_Alarm_Batt[task_id])) {

	    	//Alarme du niveau 10%
	    	if(Alarme == Alarme_10){
	    		All_Car[task_id].Set_Station_recharge_Enable(true);
	    	}
	    	//Alarme du niveau 80%
	    	else if(Alarme == Alarme_80){
	    		All_Car[task_id].Set_Station_recharge_Enable(false);

	    	}
	    	sem_post(Protect_Car_ID[task_id]);
	    }
	}
}
/*
Fonction qui gère le niveau de la batterie

Input:
  Argument du thread

Commentaire:
  Gère les batteries pour les véhicules
*/
void* Control_Batt(void* args) {

	 struct timespec tp;
     uint32_t task_id;
     uint32_t starttime;
	 sem_t* sync_sem;
     uint32_t elapsed_time;
	 /* Get the arguments */
	 sync_sem = ((thread_args_t*)args)->semaphore;
	 int32_t task_chid = ((thread_args_t*)args)->chid;
	 task_id = ((thread_args_t*)args)->id;

	 //Core affinity
     int data = 0b0;
     switch (task_id){
     case 0:
     	data = 0b00000001;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 1:
     	data = 0b00000010;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 2:
     	data = 0b00000100;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 3:
     	data = 0b00001000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 4:
     	data = 0b00010000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 5:
     	data = 0b001000000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 6:
     	data = 0b01000000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     default:break;
     }

	 while(1<2) {
		 if(0 == sem_wait(sync_sem)) {
			 // Get the current time
		     	 if(0 == clock_gettime(CLOCK_REALTIME, &tp)) {
		     		 elapsed_time = tp.tv_sec - starttime;
		     			pthread_mutex_lock(&m_Protect_Speed);
		     			//Indication par vitesse
		     			 switch (All_Car[task_id].Get_Vitesse()){
		                     	 case Fast:
		                             	 All_Car[task_id].Set_Batt_Lvl(-0.05*SPEED_UP_SIM/10);
		                             	 break;
		                     	 case Slow :
		                     		 	 All_Car[task_id].Set_Batt_Lvl(-0.005*SPEED_UP_SIM/10);
		                             	 break;
		                     	 case Stop :
		                             	 if(All_Car[task_id].Get_Station_Recharge_Status()){
		                             		 All_Car[task_id].Set_Batt_Lvl(0.05*SPEED_UP_SIM/10);

		                             		 //Gestion de l'alarme a 80%
		                                     	 if(All_Car[task_id].Get_Batt_Lvl() >= BATT_LVL_HIGH){
		                                             	 Alarme = Alarme_80;
		                                             	 sem_post(Sem_Alarm_Batt[task_id]);
		                                             	 sem_wait(Protect_Car_ID[task_id]);
		                                     	 }
		                             	 }
		                             	 break;
		                     	 default : break;
		             	 }
		     			pthread_mutex_unlock(&m_Protect_Speed);

		     			//Gestion de l'alarme a 10%
		     	        if(All_Car[task_id].Get_Batt_Lvl() <= BATT_LVL_LOW && !All_Car[task_id].Get_Station_Recharge_Status()){
		     	        		Alarme = Alarme_10;
		     	        		sem_post(Sem_Alarm_Batt[task_id]);
		     	        		sem_wait(Protect_Car_ID[task_id]);
		     	        }
		     	 }
		     	 else {

		     		 printf("Task %d could not get time: %d\n",task_id, errno);
         	 }
		 }
		 else {
			 printf("Task %d could not wait semaphore: %d\n", task_id, errno);
         }
 }
}
/*
Fonction qui gère la position réel

Input:
  int ID; ID de la voiture

Commentaire:
  L'appel de A* se fait ici,
  Calcul des positions réelles et des orientations sont réalisés ici
*/
void PosReel(int ID){


		//Protection des valeurs de positions courantes
		pthread_mutex_lock(&m_Protect_Location);
		coord_t coord_WP = All_Car[ID].Get_WP();
		coord_t coord_R  = All_Car[ID].Get_PosReal();
		pthread_mutex_unlock(&m_Protect_Location);

		//Calcul d'orientation
		if(coord_WP.x - coord_R.x > 0 )
			All_Car[ID].Set_Orientation(atan((coord_WP.y - coord_R.y) / (coord_WP.x - coord_R.x)));
		else if(coord_WP.y - coord_R.y >= 0)
			All_Car[ID].Set_Orientation((atan((coord_WP.y - coord_R.y) / (coord_WP.x - coord_R.x))) + PI);
		else if(coord_WP.y - coord_R.y < 0)
			All_Car[ID].Set_Orientation((atan((coord_WP.y - coord_R.y) / (coord_WP.x - coord_R.x))) - PI);


		//Calcule de la position réelle
		pthread_mutex_lock(&m_Protect_Speed);
        if (All_Car[ID].Get_Vitesse() == Fast){
        	coord_R.x =  coord_R.x + FAST_SPEED_M_S * SPEED_UP_SIM * PERIOD_TASK_1/1000 * cos(All_Car[ID].Get_Orientation());
        	coord_R.y =  coord_R.y + FAST_SPEED_M_S * SPEED_UP_SIM * PERIOD_TASK_1/1000 * sin(All_Car[ID].Get_Orientation());
        }
        else if (All_Car[ID].Get_Vitesse() == Slow){
        	coord_R.x =  coord_R.x + SLOW_SPEED_M_S * SPEED_UP_SIM * PERIOD_TASK_1/1000 * cos(All_Car[ID].Get_Orientation());
        	coord_R.y =  coord_R.y + SLOW_SPEED_M_S * SPEED_UP_SIM * PERIOD_TASK_1/1000 * sin(All_Car[ID].Get_Orientation());
        }
        pthread_mutex_unlock(&m_Protect_Speed);

        //Utilisation de A*, on regarde s'il y a un obstacle entre la position courante et le WP (une seule fois par WP)
        if (!All_Car[ID].Get_Dest_Final_Reach() && !All_Car[ID].Ongoing_AStar){
        	All_Car[ID].Use_AStar = All_Car[ID].aStarSearch({coord_WP.x ,
        		coord_WP.y});
        	All_Car[ID].Ongoing_AStar = true;
        }

        //Utilisation ou non des valeurs de A*
        //Implementation boolean inverse, erreur de nomenclature et de retour de fonction
        //Ici si Use_Astar est vrai, c'est qu'il n'a pas trouver d'obstacle entre lui et son WP
        if(All_Car[ID].Use_AStar){
        	pthread_mutex_lock(&m_Protect_Location);
        	All_Car[ID].Set_PosReal(coord_R);
        	pthread_mutex_unlock(&m_Protect_Location);
        }
        else{
        	//Les positions de A* sont utilisées
        	pthread_mutex_lock(&m_Protect_Location);
        	All_Car[ID].Set_PosReal({All_Car[ID].Path.front().first,All_Car[ID].Path.front().second});
        	pthread_mutex_unlock(&m_Protect_Location);
        	pm.setPx(coord_R.x,coord_R.y,{0,255,0});
        	All_Car[ID].Ongoing_AStar = true;
        	All_Car[ID].Path.pop();

        	//Jusqu'a ce que la queue soit vide
        	if (All_Car[ID].Path.empty()){
        		All_Car[ID].Use_AStar = true;
        		All_Car[ID].Ongoing_AStar = false;
        	}
        }
 }
/*
Fonction qui gère la position des voitures

Input:
  Argument du thread

Commentaire:
  Appel la fonction position réel
*/
void* Control_Position(void* args){
coord_t c;
	struct timespec tp;
	 sem_t* sync_sem;
     uint32_t task_id;
     uint32_t starttime;
     uint32_t elapsed_time;
	 int32_t task_chid;
	 /* Get the arguments */
	 sync_sem = ((thread_args_t*)args)->semaphore;
	 task_chid = ((thread_args_t*)args)->chid;
	 task_id = ((thread_args_t*)args)->id;

	 //Core affinity
     int data = 0b0;
     switch (task_id){
     case 0:
     	data = 0b00000001;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 1:
     	data = 0b00000010;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 2:
     	data = 0b00000100;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 3:
     	data = 0b00001000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 4:
     	data = 0b00010000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 5:
     	data = 0b001000000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 6:
     	data = 0b01000000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     default:break;
     }

	 while(1<2) {
		 if(0 == sem_wait(sync_sem)) {
			 /* Get the current time */
		     	 if(0 == clock_gettime(CLOCK_REALTIME, &tp)) {
		     		 elapsed_time = tp.tv_sec - starttime;
		     		 PosReel(task_id);
		     	 }
		     	 else {
		     		 /* Print error */
		     		 printf("Task %d could not get time: %d\n",task_id, errno);
        	 }
		 }
		 else {
			 printf("Task %d could not wait semaphore: %d\n", task_id, errno);
        }
	 }
}
/*
Fonction qui controle la navigation

Input:
  Argument du thread

Commentaire:
  Gère les vitesses, les sleeps, calculs les WP, destination et closest station
*/
void* Controle_Navigation(void* args) {
	 struct timespec tp;
     uint32_t task_id;
     uint32_t starttime;
     uint32_t elapsed_time;
	 sem_t* sync_sem;
	 int32_t task_chid;

	 /* Get the arguments */
	 sync_sem = ((thread_args_t*)args)->semaphore;
	 task_chid = ((thread_args_t*)args)->chid;
	 task_id = ((thread_args_t*)args)->id;

	 //Core affinity
     int data = 0b0;
     switch (task_id){
     case 0:
     	data = 0b00000001;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 1:
     	data = 0b00000010;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 2:
     	data = 0b00000100;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 3:
     	data = 0b00001000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 4:
     	data = 0b00010000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 5:
     	data = 0b001000000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     case 6:
     	data = 0b01000000;
     	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
     	break;
     default:break;
     }

     //Variable temporaire
     coord_t PosR;
     coord_t Dest;
     coord_t WP;

	 while(1<2) {
		 if(0 == sem_wait(sync_sem)) {
		     	 if(0 == clock_gettime(CLOCK_REALTIME, &tp)) {
		     		 elapsed_time = tp.tv_sec - starttime;

		     			//Extract data from class
		     			pthread_mutex_lock(&m_Protect_Location);
		     			PosR = All_Car[task_id].Get_PosReal();
		     			Dest = All_Car[task_id].Get_Dest_Final();
		     			WP = All_Car[task_id].Get_WP();
		     			pthread_mutex_unlock(&m_Protect_Location);

		     			//If car not sleeping
		     			if(!All_Car[task_id].Get_Sleep_Status()){
		     				//If destination final is reach, get new final destination
		     				if(All_Car[task_id].Get_Dest_Final_Reach()){
		     					All_Car[task_id].Set_Dest_Final_Reach(false);
                                pm.genDest(PosR, Dest, All_Car[task_id].Get_Car_ID());
                                pm.genWp(PosR, Dest, WP);
                                All_Car[task_id].Ongoing_AStar = false;

                                //if it's not in initialisation mode
                                if(!All_Car[task_id].Get_Init()){
                                	All_Car[task_id].Set_Sleep_Status(true);
                                	All_Car[task_id].Reset_Sleep_Timer();
                                	cout << "Sleep start"<< endl;
                                	pthread_mutex_lock(&m_Protect_Speed);
                                	All_Car[task_id].Set_Vitesse(Stop);
                                	pthread_mutex_unlock(&m_Protect_Speed);


                                }
                                else{
                                	All_Car[task_id].Set_Init(false);
                                	pthread_mutex_lock(&m_Protect_Speed);
                                	All_Car[task_id].Set_Vitesse(Fast);
                                	pthread_mutex_unlock(&m_Protect_Speed);
                                }
		     				}
		     				//WP is reached
		     				else if(All_Car[task_id].Get_WP_Reach()){
		     					if(!All_Car[task_id].Get_Station_Recharge_Status()){
		     						pm.genWp(PosR,Dest,WP);
		     						All_Car[task_id].Ongoing_AStar = false;
		     						All_Car[task_id].Set_WP_Reach(false);
		     						pthread_mutex_lock(&m_Protect_Speed);
		     						All_Car[task_id].Set_Vitesse(Fast);
		     						pthread_mutex_unlock(&m_Protect_Speed);
		     					}

		     				}
		     				//Compare current position with WP
		     				if(Compare_Pos(PosR,WP)){
		     					All_Car[task_id].Set_WP_Reach(true);
		     					//Is recharge is enable
		     					if(All_Car[task_id].Get_Station_Recharge_Status()){
		     						pthread_mutex_lock(&m_Protect_Speed);
		     						All_Car[task_id].Set_Vitesse(Stop);
		     						pthread_mutex_unlock(&m_Protect_Speed);
		     					}
		     					//Compare current position with destination
		     					if(Compare_Pos(PosR,Dest)){
		     						All_Car[task_id].Set_WP_Reach(false);
		     						All_Car[task_id].Set_Dest_Final_Reach(true);
		     						pthread_mutex_lock(&m_Protect_Speed);
		     						All_Car[task_id].Set_Vitesse(Stop);
		     						pthread_mutex_unlock(&m_Protect_Speed);
		     					}
		     				}
		     				//Recharge mode is enable, get closest station
		     				else if(All_Car[task_id].Get_Station_Recharge_Status() && All_Car[task_id].Get_Vitesse() == Fast){
		     						pm.getClosestStation(PosR,WP, All_Car[task_id].Get_Car_ID());
		     						pthread_mutex_lock(&m_Protect_Speed);
		     						All_Car[task_id].Set_Vitesse(Slow);
		     						pthread_mutex_unlock(&m_Protect_Speed);

		     				}

			     			pthread_mutex_lock(&m_Protect_Location);
			     			All_Car[task_id].Set_Dest_Final(Dest);
			     			All_Car[task_id].Set_WP(WP);
			     			pthread_mutex_unlock(&m_Protect_Location);

		     			}
		     			//Car is sleeping
		     			else{
		     				All_Car[task_id].Set_Sleep_Timer();
		     				if(All_Car[task_id].Get_Sleep_Timer() >= SLEEP_TIME){
		     					All_Car[task_id].Set_Sleep_Status(false);
		     				    All_Car[task_id].Set_Vitesse(Fast);
		     				    cout << "Sleep end" << endl;
		     				}
		     			}
		     	 }
		     	 else {

		     		 printf("Task %d could not get time: %d\n",task_id, errno);
		     	 }
		 }
		 else {
			 printf("Task %d could not wait semaphore: %d\n", task_id, errno);
       }
	 }
}
/******************************************************************************
 * Timer initialization routine
 * The function will initialize a timer given the parameters.
 *****************************************************************************/
int32_t init_timer(struct sigevent* event, struct itimerspec* itime,
                                        timer_t* timer, const int32_t chanel_id,const uint32_t period) {

        int32_t error;
        int32_t period_s;
        int32_t period_ns;
        /* Set event as pulse and attach to channel */
        event->sigev_notify = SIGEV_PULSE;
        event->sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, chanel_id,_NTO_SIDE_CHANNEL, 0);
        /* Set basic priority and set event code */
        event->sigev_priority = 0;
        event->sigev_code = TASK_PULSE_CODE;
        /* Create timer and associate to event */
        error = timer_create(CLOCK_MONOTONIC, event, timer);
        if(0 != error) {
                printf("Error creating timer\n");
                return error;
        }
        /* Set the itime structure */
        period_s = period / 1000;
        period_ns = (1000000 * period) - (period_s * 1000000000);
        itime->it_value.tv_sec = period_s;
        itime->it_value.tv_nsec = period_ns;
        itime->it_interval.tv_sec = period_s;
        itime->it_interval.tv_nsec = period_ns;
        /* Set the timer period */
        return timer_settime(*timer, 0, itime, NULL);
}

/******************************************************************************
 * Task pulse handler routine
 * Handles a pulse and release the semaphore.
 *****************************************************************************/
void* task_pulse_handler(void* args) {
        sem_t* sync_sem;
        int32_t rcvid;
        pulse_msg_t msg;
        int32_t task_chid;
        /* Get the arguments */
        sync_sem = ((thread_args_t*)args)->semaphore;
        task_chid = ((thread_args_t*)args)->chid;
        uint32_t task_id = ((thread_args_t*)args)->id;

        //Core affinity
        int data = 0b0;
        switch (task_id){
        case 0:
        	data = 0b00000001;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 1:
        	data = 0b00000010;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 2:
        	data = 0b00000100;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 3:
        	data = 0b00001000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 4:
        	data = 0b00010000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 5:
        	data = 0b001000000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 6:
        	data = 0b01000000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case IMAGE_TASK_ID: //For image dumping
        	data = 0b10000000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;

        default:break;
        }
        while(1<2) {
                /* Get the pulse message */
                rcvid = MsgReceive(task_chid, &msg, sizeof(pulse_msg_t), NULL);
                if (0 == rcvid) {
                        if (TASK_PULSE_CODE == msg.pulse.code) {
                                /* Release semaphore */
                                if(0 != sem_post(sync_sem)) {
                                        /* Print error */
                                        printf("Could not post semaphore:%d\n", errno);
                                }
                        }
                        else {
                                /* Print error */
                                printf("Unknown message received: %d\n",rcvid);
                        }
                }
                else {
                        /* Print error */
                        printf("Message receive failed: %d (%d)\n", rcvid,errno);
                }
        }
}


/*
Fonction qui gère la caméra

Input:
  Argument du thread

Commentaire:
  Appel la fonction Take_Photo apres 10m de parcourue
*/
void* Controle_Camera(void* args) {

		//Position temporaire pour la photo
    	double PosX = 0;
    	double PosY = 0;
        struct timespec tp;
        sem_t* sync_sem;
        uint32_t task_id;
        uint32_t starttime;
        uint32_t elapsed_time;
        /* Get the arguments */
        sync_sem = ((thread_args_t*)args)->semaphore;
        task_id = ((thread_args_t*)args)->id;
        starttime = ((thread_args_t*)args)->starttime;

        //Core affinity
        double d = 0;
        int data = 0b0;
        switch (task_id){
        case 0:
        	data = 0b00000001;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 1:
        	data = 0b00000010;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 2:
        	data = 0b00000100;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 3:
        	data = 0b00001000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 4:
        	data = 0b00010000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 5:
        	data = 0b001000000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 6:
        	data = 0b01000000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        default:break;
        }
        /* Routine loop */
        while(1<2) {
                /* Wait for the pulse handler to release the semaphore */
                if(0 == sem_wait(sync_sem)) {
                        /* Get the current time */
                        if(0 == clock_gettime(CLOCK_REALTIME, &tp)) {
                                elapsed_time = tp.tv_sec - starttime;

                                	//Calcul la distance parcourue
                               		if (All_Car[task_id].Get_Vitesse() == Fast)
                               		 	 All_Car[task_id].distance = All_Car[task_id].distance + FAST_SPEED_M_S*SPEED_UP_SIM*PERIOD_TASK_0/1000;
                               	 	 else if (All_Car[task_id].Get_Vitesse() == Slow)
                               		 	 All_Car[task_id].distance = All_Car[task_id].distance + SLOW_SPEED_M_S*SPEED_UP_SIM*PERIOD_TASK_0/1000;
                               	 	 else
                               	 		 All_Car[task_id].distance = All_Car[task_id].distance + 0;

                               		//Appel Take_photo apres 10m
                               	 	 if( All_Car[task_id].distance > 10){
                               			PosX = All_Car[task_id].Get_PosReal().x/PX_TO_M;
                               			PosY = All_Car[task_id].Get_PosReal().y/PX_TO_M;
                                       	pm.takePhoto({PosX,PosY});
                               			All_Car[task_id].distance = 0;
                               		}
                        }
                        else {
                                /* Print error */
                                printf("Task %d could not get time: %d\n",
                                                task_id, errno);
                        }
                }
                else {
                        printf("Task %d could not wait semaphore: %d\n",
                                        task_id, errno);
                        }
                }
}
/*
Fonction qui gère l'affichage

Input:
  Argument du thread

Commentaire:
  Gère l'affichage des valeurs pour chaque véhicules
*/
void* Affichage(void* args) {
        struct timespec tp;
        sem_t* sync_sem;
        uint32_t task_id;
        uint32_t starttime;
        uint32_t elapsed_time;
        /* Get the arguments */
        sync_sem = ((thread_args_t*)args)->semaphore;
        task_id = ((thread_args_t*)args)->id;
        starttime = ((thread_args_t*)args)->starttime;

        //Core affinity
        int data = 0b0;
        switch (task_id){
        case 0:
        	data = 0b00000001;
            ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
            break;
        case 1:
        	data = 0b00000010;
            ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
            break;
        case 2:
        	data = 0b00000100;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 3:
        	data = 0b00001000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 4:
        	data = 0b00010000;
        	ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
        	break;
        case 5:
            data = 0b001000000;
            ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
            break;
        case 6:
            data = 0b01000000;
            ThreadCtl(_NTO_TCTL_RUNMASK, (void *) data);
            break;
        default:break;
        }
        while(1<2) {
        	/* Wait for the pulse handler to release the semaphore */
            if(0 == sem_wait(sync_sem)) {
            	/* Get the current time */
                if(0 == clock_gettime(CLOCK_REALTIME, &tp)) {
                	elapsed_time = tp.tv_sec - starttime;
                    All_Car[task_id].Print_Info();
                }
                else {
                	/* Print error */
                    printf("Task %d could not get time: %d\n", task_id, errno);
                }
            }
            else {
            	printf("Task %d could not wait semaphore: %d\n", task_id, errno);
            }
        }
}
/*
Fonction qui gère créer un obstacles

Commentaire:
  Créer les obstacles et les dessines
*/
void Create_Rock() {
	int Dimension = rand() % 3;

	double W = rand() % (IMG_W_PX - (2*Rock_Dimension[Dimension]+1)) + Rock_Dimension[Dimension];
	double H = rand() % (IMG_W_PX - (2*Rock_Dimension[Dimension]+1)) + Rock_Dimension[Dimension];

	W = W - Rock_Dimension[Dimension] / 2;
	H = H - Rock_Dimension[Dimension] / 2;

	for (int i = 0; i < Rock_Dimension[Dimension]; i++)
		for(int j = 0; j <Rock_Dimension[Dimension]; j++){
			pair<double, double> p = make_pair(H + i, W + j);
			myv.push_back(p);
			pm.setPx(H + i, W + j , {255,102,204});
		}
}
/*
Fonction qui gère créer les obstacles

Commentaire:
  Créer les obstacles selon les paramètres
*/
void Obstacle_Creation(){

	E_Rock_Density Rock_Density;
	int r = rand() % 3;

	if (r == 0)
		Rock_Density = LOW;
	else if(r==1)
		Rock_Density = MEDIUM;
	else
		Rock_Density = HIGH;

	switch (Rock_Density){
	case LOW:
			for(int i = 0; i < Low_Density; i++)
				Create_Rock();
		break;
	case MEDIUM:
			for (int i = 0; i < Medium_Density; i++)
				Create_Rock();
		break;
	case HIGH:
			for (int i = 0; i < High_Density; i++)
				Create_Rock();
		break;
	default:
		break;
	}
}

int main(void) {

		int nb_car = 0;

		cout << "Entrez le nombre de voiture désiré (PLZ no letter cant handle): " << endl;
		cin >> nb_car;

		Obstacle_Creation();
		//Créer le nombre de véhicule désirer
		for(int i = 0; i< nb_car;i++){
			Car car;
			car.Initialisation_Car(i);
			All_Car.push_back(car);
			All_Car[i].obstacle = myv;
		}

		cout << "osbtacle créer"<< endl;
		// 	STRUCTURE POUR MULTI-CORE/ MULTI-THREAD
		struct timespec tp;

		pthread_t task;
		pthread_t task_Alarm;
		sem_t sempahore;
		pthread_t pulse;

		/* Timers event structures */
		timer_t task_timer;
		thread_args_t task_args;
		thread_args_t task_args_Alarm;

		struct sigevent task_event;
		struct itimerspec task_itime;
		for(int k = 0; k < nb_car;k++){
			sem_t *semaphore = new sem_t;
			Sem_Alarm_Batt[k] = semaphore;
			sem_t *sem = new sem_t;
			Protect_Car_ID[k] = sem;
		}
		int i = 0, j = 0;

		for (i = 0; i < nb_car; i++) {

			for (j = 0; j < NB_OF_THREAD; j++) {
				/* Semaphores for pulse synchronization */
				//v_task_sync.push_back(sempahore);
				sem_t* sempahore = new sem_t;
				/* PThread structures */
				v_task.push_back(task);
				v_pulse_handler.push_back(pulse);

				/* Timers event structures */
				v_task_event.push_back(task_event);
				v_task_itime.push_back(task_itime);
				v_task_timer.push_back(task_timer);

				/* Tasks arguments */
				v_task_args.push_back(task_args);
			}

			//one by car

			v_task_Alarm.push_back(task_Alarm);
			v_task_args_Alarm.push_back(task_args_Alarm);
			v_task_args_Alarm[i].id = i;

			/* Get the start time */
			if (0 != clock_gettime(CLOCK_REALTIME, &tp)) {
				/* Print error */
				printf("Could not get start time: %d\n", errno);
				return EXIT_FAILURE;
			}

			//Alarm thread
			if (0 != pthread_create(&v_task_Alarm[i], NULL, Alarm_Batt, &v_task_args_Alarm[i])) {
				/* Print error */
				printf("Could not create thread: %d\n", errno);
				return EXIT_FAILURE;
			}
			j = 0;
			for (j = 0; j < NB_OF_THREAD; j++) {
				/* Initialize the semaphore */
				if (0 != sem_init(&sempahore,0,0)){//&v_task_sync[i * NB_OF_THREAD + j], 0, 0)) {
					/* Print error */
					printf("Could not get init semaphore: %d\n", errno);
					return EXIT_FAILURE;
				}

				/* Initialize the tasks arguments */
				v_task_args[i * NB_OF_THREAD + j].id = i;
				v_task_args[i * NB_OF_THREAD + j].semaphore = &sempahore;// &v_task_sync[i * NB_OF_THREAD + j];
				v_task_args[i * NB_OF_THREAD + j].starttime = tp.tv_sec;
				v_task_args[i * NB_OF_THREAD + j].chid = ChannelCreate(0);

				if (-1 == v_task_args[i * NB_OF_THREAD + j].chid) {
					/* Print error */
					printf("Could not create channel: %d\n", errno);
					return EXIT_FAILURE;
				}
			}
			for (int j = 0; j < NB_OF_THREAD; j++) {
				/* Create the different tasks and their associated pulse handlers */
				if( j == 0)
					if (0 != pthread_create(&v_task[i * NB_OF_THREAD + j], NULL, Controle_Camera, &v_task_args[i * NB_OF_THREAD + j])) {
						/* Print error */
						printf("Could not create thread: %d\n", errno);
						return EXIT_FAILURE;
					}
				if(j == 1)
					if (0 != pthread_create(&v_task[i * NB_OF_THREAD + j], NULL, Controle_Navigation, &v_task_args[i * NB_OF_THREAD + j])) {
						/* Print error */
						printf("Could not create thread: %d\n", errno);
						return EXIT_FAILURE;
					}
				if(j == 2)
					if (0 != pthread_create(&v_task[i * NB_OF_THREAD + j], NULL, Affichage, &v_task_args[i * NB_OF_THREAD + j])) {
						/* Print error */
						printf("Could not create thread: %d\n", errno);
						return EXIT_FAILURE;
					}
				if(j == 3)
					if (0 != pthread_create(&v_task[i * NB_OF_THREAD + j], NULL, Control_Batt, &v_task_args[i * NB_OF_THREAD + j])) {
						/* Print error */
						printf("Could not create thread: %d\n", errno);
						return EXIT_FAILURE;
					}
				if(j == 4)
					if (0 != pthread_create(&v_task[i * NB_OF_THREAD + j], NULL, Control_Position, &v_task_args[i * NB_OF_THREAD + j])) {
						/* Print error */
						printf("Could not create thread: %d\n", errno);
						return EXIT_FAILURE;
					}

				if (0 != pthread_create(&v_pulse_handler[i * NB_OF_THREAD + j], NULL,
					task_pulse_handler, &v_task_args[i * NB_OF_THREAD + j])) {
					/* Print error */
					printf("Could not create thread: %d\n", errno);
					return EXIT_FAILURE;
				}

				/* Create timers */
				if (0 != init_timer(&v_task_event[i * NB_OF_THREAD + j], &v_task_itime[i * NB_OF_THREAD + j], &v_task_timer[i * NB_OF_THREAD + j],
					v_task_args[i * NB_OF_THREAD + j].chid, v_Task_Period[j])) {
					/* Print error */
					printf("Could not create timer: %d\n", errno);
					return EXIT_FAILURE;
				}

			}


		}

		//Creer le thread pour single core pour le dump image
		sem_t task_Image_sync;
		pthread_t task_Image;
		pthread_t task_Image_pulse_handler;
        struct sigevent task_Image_event;
        struct itimerspec task_Image_itime;
        timer_t task_Image_timer;
        thread_args_t task_Image_args;

        /* Initialize the semaphore */
        if(0 != sem_init(&task_Image_sync, 0, 0)) {
                /* Print error */
                printf("Could not get init semaphore: %d\n", errno);
                return EXIT_FAILURE;
        }

        /* Initialize the tasks arguments */
        task_Image_args.id = IMAGE_TASK_ID;
        task_Image_args.semaphore = &task_Image_sync;
        task_Image_args.starttime = tp.tv_sec;
        task_Image_args.chid = ChannelCreate(0);

        if(-1 == task_Image_args.chid) {
                /* Print error */
                printf("Could not create channel: %d\n", errno);
                return EXIT_FAILURE;
        }

        /* Create the different tasks and their associated pulse handlers */
         if(0 != pthread_create(&task_Image, NULL, Image_Dumping, &task_Image_args)) {
                 /* Print error */
                 printf("Could not create thread: %d\n", errno);
                 return EXIT_FAILURE;
         }
         if(0 != pthread_create(&task_Image_pulse_handler, NULL,
                         task_pulse_handler, &task_Image_args)) {
                 /* Print error */
                 printf("Could not create thread: %d\n", errno);
                 return EXIT_FAILURE;
         }
         /* Create timers */
         if(0 != init_timer(&task_Image_event, &task_Image_itime, &task_Image_timer,
        		 task_Image_args.chid, PERIOD_TASK_IMAGE)) {
                 /* Print error */
                 printf("Could not create timer: %d\n", errno);
                 return EXIT_FAILURE;
         }

cout << "fin thread"<< endl;
		/* Wait for the threads to finish */
		if (0 != pthread_join(v_task[0], NULL)) {
			/* Print error */
			printf("Could not wait for thread: %d\n", errno);
			return EXIT_FAILURE;
		}

		if (0 != pthread_join(v_pulse_handler[0], NULL)) {
			/* Print error */
			printf("Could not wait for thread: %d\n", errno);
			return EXIT_FAILURE;
		}

        return EXIT_SUCCESS;
}
