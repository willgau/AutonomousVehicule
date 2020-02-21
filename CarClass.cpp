/*
 * Astromobile2.cpp
 * Implémentation complète des fonctions de la classe Car
 * Créer par: William Gauvin
 * */

#include "astromobile.h"
// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;
//Get ID
int Car::Get_Car_ID() {
	return ID;
}

//Get Batterie lvl
double Car::Get_Batt_Lvl() {
	return Batterie_lvl;
}
//Set batterie lvl
void Car::Set_Batt_Lvl(double slope) {
	Batterie_lvl = Batterie_lvl + slope;
}

//Set Station recharge enable if true == enable if false == disable
void Car::Set_Station_recharge_Enable(bool status) {
	Station_Recharge_Enable = status;
}
//Get Station recharge status
bool Car::Get_Station_Recharge_Status() {
	return Station_Recharge_Enable;
}
//Get Position Reel
coord_t Car::Get_PosReal() {
	return Pos_Real;
}
//Set Position Reel
void Car::Set_PosReal(coord_t p) {
	Pos_Real = p;
}
//Get la vitesse
E_Vitesse Car::Get_Vitesse() {
	return Vitesse;
}
//Set la vitesse
void Car::Set_Vitesse(E_Vitesse v) {
	Vitesse = v;
}
//Get WayPoint
coord_t Car::Get_WP() {
	return WP;
};
//Set Waypoint
void Car::Set_WP(coord_t wp) {
	WP = wp;
}
//Set orientation
void Car::Set_Orientation(double o) {
	Orientation = o;
}
//Get Orientation
double Car::Get_Orientation() {
	return Orientation;
}
//Get Sleep timer
int Car::Get_Sleep_Timer() {
	return Sleep_Timer;
}
//Set Sleep Timer
void Car::Set_Sleep_Timer() {
	Sleep_Timer++;
}
//Get Sleep Status
bool Car::Get_Sleep_Status() {
	return Sleep_Status;
}
//Set Sleep Status
void Car::Set_Sleep_Status(bool s) {
	Sleep_Status = s;
}
//Set Final Dest Flag
void Car::Set_Dest_Final_Reach(bool f) {
	Dest_Final_Reach = f;
}
//Get Final Dest Flag
bool Car::Get_Dest_Final_Reach() {
	return Dest_Final_Reach;
}
//Get Destination Final
coord_t Car::Get_Dest_Final() {
	return Dest;
}
//Set Final Destination
void Car::Set_Dest_Final(coord_t d) {
	Dest = d;
}
//Get Initialisation status
bool Car::Get_Init() {
	return Init_0;
}
//Set Init status
void Car::Set_Init(bool i) {
	Init_0 = i;
}
//Reset Sleep Timer
void Car::Reset_Sleep_Timer() {
	Sleep_Timer = 0;
}
//Get WP Reach
bool Car::Get_WP_Reach() {
	return Dest_WP_Reach;
}
//Set WP Reach
void Car::Set_WP_Reach(bool b) {
	Dest_WP_Reach = b;
}

void Car::Print_Info() {
	/*
	cout << "***********************"<< endl;
	cout << "Car number: " << ID << endl;
	cout << "Niveau Batterie :"<<Batterie_lvl << endl;
	cout << "Reel x: "<< Pos_Real.x << " y:"<<Pos_Real.y<< endl;
	cout << "En recharge? : " << Station_Recharge_Enable <<endl;
	cout << "Vitesse 0=fast; 1=slow ; 2= stop : " << Vitesse <<endl;
	cout << "***********************"<< endl;
*/
}
/*
Fonction qui gère l'alarme de la batterie

Input:
  int i: ID du véhicule

Commentaire:
  Initialise le véhicule
*/
void Car::Initialisation_Car(int i) {
	Batterie_lvl = INIT_NIV_BAT;
	Pos_Real = { double(rand() % IMG_W_M), double(rand() % IMG_H_M) };
	Dest_Final_Reach = true;
	Vitesse = Stop;
	Orientation = 0;
	Sleep_Timer = 0;
	Sleep_Status = false;
	ID = i;
	Init_0 = true;
	distance = 0;
	Use_AStar = false;
	Ongoing_AStar = false;
}
/*
Fonction qui regarde s'il y a un obstacle sur la route du véhicule

Input:
  coord_t nPos: Next position du véhicule

Commentaire:
  calcul un crossproduct, un dotproduct et la distance entre les points pour
  vérifier s'il y a un objet entre sa position réel et le WP
*/
bool Car::isBlocked(coord_t nPos)
{
	for (auto i : obstacle) {
		int crossprod = int((i.second - int(Pos_Real.y)) * (nPos.x - int(Pos_Real.x)) - (i.first - int(Pos_Real.x)) * (nPos.y - int(Pos_Real.y)));
		int dotprod = int((i.first - Pos_Real.x) * (nPos.x - Pos_Real.x) + (i.second - Pos_Real.y) * (nPos.y - Pos_Real.y));
		int sqrlength = int((nPos.x - Pos_Real.x)*(nPos.x - Pos_Real.x) + (nPos.y - Pos_Real.y)*(nPos.y - Pos_Real.y));
		if (abs(crossprod) != 0)
			continue;
		else if (dotprod < 0)
			continue;
		else if (dotprod > sqrlength)
			continue;
		else
			return true;
	}
	return false;
}
/*
Fonction qui regarde si la position fournis par A* est la même que la destination

Input:
   int row; la ligne
   int col; la colonne
   pair<int, int> dest; pair de position
*/
bool isDestination(int row, int col, pair<int, int> dest)
{
	if (row == dest.first && col == dest.second)
		return true;
	else
		return false;
}
/*
Fonction qui calcul l'heuristic de H

Input:
   int row; la ligne
   int col; la colonne
   pair<int, int> dest; pair de position
*/
double calculateHValue(int row, int col, Pair dest)
{
	return ((double)sqrt((row - dest.first)*(row - dest.first)
		+ (col - dest.second)*(col - dest.second)));
}
/*
Fonction A*

Input:
   coord_t nPos; Position désiré
Commentaire:
L'algorithme général a été implémenté et inspirer du site: https://www.geeksforgeeks.org/a-search-algorithm/
L'algorithme a été modifié pour nos besoins mais l'algorithme général est déjà connue
*/
int Car::aStarSearch(coord_t nPos)
{
	//Regarde s'il y a un objet sur le chemin
	if (!isBlocked(nPos))
		return 1;

	//Calcul des dimensions du tableau
	int dim_x = abs(int(Pos_Real.x - nPos.x)) + Large_Rock + 1;
	int dim_y = abs(int(Pos_Real.y - nPos.y)) + Large_Rock + 1;

	//Position du WP sur la nouvelle carte
	int xx = abs(int(Pos_Real.x - nPos.x));
	int yy = abs(int(Pos_Real.y - nPos.y));

	vector<vector<bool>> closedList(dim_x+1, vector<bool>(dim_y+1, false));

	//Initialisation d'une cellule
	cell c;
	c.f = INT_MAX;
	c.g = INT_MAX;
	c.h = INT_MAX;
	c.parent_i = -1;
	c.parent_j = -1;

	//Vecteur de cellules
	vector<vector<cell>> cellDetails(dim_x+1, vector<cell>(dim_y+1,c));


	int i, j;

	//Initialisation de la 1er cellule (position courrante)
	i = 0, j = 0;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;

	//Créer la liste des points a chercher
	set<pPair> openList;

	//Met le point de début dans la liste
	openList.insert(make_pair(0.0, make_pair(i, j)));

	//Créer la pair de points pour la destination
	pair<int, int> dest = make_pair(xx, yy);

	//Protection d'accès au vecteur
	dim_y++;
	dim_x++;

	//Tant que la liste n'est pas vide
	while (!openList.empty())
	{
		pPair p = *openList.begin();

		// Enleve le point de la liste
		openList.erase(openList.begin());

		//ajoute le point à la liste de point final
		i = int(p.second.first);
		j = int(p.second.second);

		//Marque le point comme visité
		closedList[i][j] = 1;


		// variable pour les heuristics
		double gNew, hNew, fNew;

		//***************point (-1,0)**********************

		//Entre seulement si le point est dans la carte
		if (isValid(i - 1, j, dim_x, dim_y)){
			//Si la destination finale est atteinte
			if (isDestination(i - 1, j, dest)){
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
				tracePath(cellDetails, dest, ID);
				return 0;
			}
			//Ignore si le point est déjà dans la liste ou qu'il est bloqué
			else if (closedList[i - 1][j] == 0 &&
				!isBlocked({ double(i - 1),  double(j) })){
				//Calcul d'heuristics
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i - 1, j, dest);
				fNew = gNew + hNew;

				if (cellDetails[i - 1][j].f == INT_MAX ||
					cellDetails[i - 1][j].f > fNew){
					openList.insert(make_pair(fNew,make_pair(i - 1, j)));
					cellDetails[i - 1][j].f = fNew;
					cellDetails[i - 1][j].g = gNew;
					cellDetails[i - 1][j].h = hNew;
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
				}
			}
		}

		//***************point (+1,0)**********************

		//Entre seulement si le point est dans la carte
		if (isValid(i + 1, j, dim_x, dim_y)){
			//Si la destination finale est atteinte
			if (isDestination(i + 1, j, dest)){
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
				tracePath(cellDetails, dest, ID);
				return 0;
			}
			//Ignore si le point est déjà dans la liste ou qu'il est bloqué
			else if (closedList[i + 1][j] == 0 &&
				!isBlocked({ double(i + 1),  double(j) })){
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i + 1, j, dest);
				fNew = gNew + hNew;

				if (cellDetails[i + 1][j].f == INT_MAX ||
					cellDetails[i + 1][j].f > fNew){
					openList.insert(make_pair(fNew, make_pair(i + 1, j)));
					cellDetails[i + 1][j].f = fNew;
					cellDetails[i + 1][j].g = gNew;
					cellDetails[i + 1][j].h = hNew;
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
				}
			}
		}

		//***************point (0,+1)**********************

		//Entre seulement si le point est dans la carte
		if (isValid(i, j + 1, dim_x, dim_y)){
			//Si la destination finale est atteinte
			if (isDestination(i, j + 1, dest)){
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
				tracePath(cellDetails, dest, ID);
				return 0;
			}

			//Ignore si le point est déjà dans la liste ou qu'il est bloqué
			else if (closedList[i][j + 1] == 0 &&
				!isBlocked({ double(i),  double(j + 1) })){
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i][j + 1].f == INT_MAX ||
					cellDetails[i][j + 1].f > fNew){
					openList.insert(make_pair(fNew, make_pair(i, j + 1)));
					cellDetails[i][j + 1].f = fNew;
					cellDetails[i][j + 1].g = gNew;
					cellDetails[i][j + 1].h = hNew;
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
				}
			}
		}

		//***************point (0,-1)**********************

		//Entre seulement si le point est dans la carte
		if (isValid(i, j - 1, dim_x, dim_y)){
			//Si la destination finale est atteinte
			if (isDestination(i, j - 1, dest)){
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
				tracePath(cellDetails, dest, ID);
				return 0;
			}

			//Ignore si le point est déjà dans la liste ou qu'il est bloqué
			else if (closedList[i][j - 1] == 0 && !isBlocked({ double(i),  double(j - 1) })){
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j - 1, dest);
				fNew = gNew + hNew;
			if (cellDetails[i][j - 1].f == INT_MAX ||
				cellDetails[i][j - 1].f > fNew){
				openList.insert(make_pair(fNew,	make_pair(i, j - 1)));
				cellDetails[i][j - 1].f = fNew;
				cellDetails[i][j - 1].g = gNew;
				cellDetails[i][j - 1].h = hNew;
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
			}
			}
		}

		//***************point (-1,+1)**********************

		//Entre seulement si le point est dans la carte
		if (isValid(i - 1, j + 1, dim_x, dim_y)){
			//Si la destination finale est atteinte
			if (isDestination(i - 1, j + 1, dest)){
				cellDetails[i - 1][j + 1].parent_i = i;
				cellDetails[i - 1][j + 1].parent_j = j;
				tracePath(cellDetails, dest, ID);
				return 0;
			}
			//Ignore si le point est déjà dans la liste ou qu'il est bloqué
			else if (closedList[i - 1][j + 1] == 0 &&
				!isBlocked({ double(i - 1),  double(j + 1) })){
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j + 1, dest);
				fNew = gNew + hNew;
				if (cellDetails[i - 1][j + 1].f == INT_MAX ||
					cellDetails[i - 1][j + 1].f > fNew){
					openList.insert(make_pair(fNew, make_pair(i - 1, j + 1)));
					cellDetails[i - 1][j + 1].f = fNew;
					cellDetails[i - 1][j + 1].g = gNew;
					cellDetails[i - 1][j + 1].h = hNew;
					cellDetails[i - 1][j + 1].parent_i = i;
					cellDetails[i - 1][j + 1].parent_j = j;
				}
			}
		}

		//***************point (-1,-1)**********************

		//Entre seulement si le point est dans la carte
		if (isValid(i - 1, j - 1, dim_x, dim_y)){
			//Si la destination finale est atteinte
			if (isDestination(i - 1, j - 1, dest)){
				cellDetails[i - 1][j - 1].parent_i = i;
				cellDetails[i - 1][j - 1].parent_j = j;
				tracePath(cellDetails, dest, ID);
				return 0;
			}
			//Ignore si le point est déjà dans la liste ou qu'il est bloqué
			else if (closedList[i - 1][j - 1] == 0 &&
				!isBlocked({ double(i - 1),  double(j - 1) })){
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i - 1, j - 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i - 1][j - 1].f == INT_MAX ||
					cellDetails[i - 1][j - 1].f > fNew)	{
					openList.insert(make_pair(fNew, make_pair(i - 1, j - 1)));
					cellDetails[i - 1][j - 1].f = fNew;
					cellDetails[i - 1][j - 1].g = gNew;
					cellDetails[i - 1][j - 1].h = hNew;
					cellDetails[i - 1][j - 1].parent_i = i;
					cellDetails[i - 1][j - 1].parent_j = j;
				}
			}
		}

		//***************point (+1,+1)**********************

		//Entre seulement si le point est dans la carte
		if (isValid(i + 1, j + 1, dim_x, dim_y)){
			//Si la destination finale est atteinte
			if (isDestination(i + 1, j + 1, dest)){
				cellDetails[i + 1][j + 1].parent_i = i;
				cellDetails[i + 1][j + 1].parent_j = j;
				tracePath(cellDetails, dest, ID);
				return 0;
			}
			//Ignore si le point est déjà dans la liste ou qu'il est bloqué
			else if (closedList[i + 1][j + 1] == 0 &&
				!isBlocked({ double(i + 1),  double(j + 1) })){
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j + 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i + 1][j + 1].f == INT_MAX ||
					cellDetails[i + 1][j + 1].f > fNew){
					openList.insert(make_pair(fNew, make_pair(i + 1, j + 1)));
					cellDetails[i + 1][j + 1].f = fNew;
					cellDetails[i + 1][j + 1].g = gNew;
					cellDetails[i + 1][j + 1].h = hNew;
					cellDetails[i + 1][j + 1].parent_i = i;
					cellDetails[i + 1][j + 1].parent_j = j;
				}
			}
		}

		//***************point (+1,-1)**********************

		//Entre seulement si le point est dans la carte
		if (isValid(i + 1, j - 1, dim_x, dim_y)){
			//Si la destination finale est atteinte
			if (isDestination(i + 1, j - 1, dest)){
				cellDetails[i + 1][j - 1].parent_i = i;
				cellDetails[i + 1][j - 1].parent_j = j;
				tracePath(cellDetails, dest, ID);
				return 0;
			}
			//Ignore si le point est déjà dans la liste ou qu'il est bloqué
			else if (closedList[i + 1][j - 1] == 0 && !isBlocked({ double(i + 1),  double(j - 1) })){
				gNew = cellDetails[i][j].g + 1.414;
				hNew = calculateHValue(i + 1, j - 1, dest);
				fNew = gNew + hNew;

				if (cellDetails[i + 1][j - 1].f == INT_MAX ||
					cellDetails[i + 1][j - 1].f > fNew){
					openList.insert(make_pair(fNew,	make_pair(i + 1, j - 1)));
					cellDetails[i + 1][j - 1].f = fNew;
					cellDetails[i + 1][j - 1].g = gNew;
					cellDetails[i + 1][j - 1].h = hNew;
					cellDetails[i + 1][j - 1].parent_i = i;
					cellDetails[i + 1][j - 1].parent_j = j;
				}
			}
		}
	}
	return 1;
}
