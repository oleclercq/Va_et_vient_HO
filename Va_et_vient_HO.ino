/* Sweep
il faut 2 inter sur la gare, (une gare suffit)
*/


// CONFIGURATION METERIEL DE L ARDUINO
#define PWM_PIN (3) // D7
#define DIRECTION_PIN (12)
#define CPT_A (5) // D7
#define CPT_B (6) // D7
#define CPT_C (A2) // D7
#define CPT_D (A3) // D7

#define EACH_50MS (10)		// Valeur pour avoir un Timer toutes les 50ms.

#define PWM_RECHERCHE (200)		// Vitesse du train pour rechercher le 'Capteur A'
#define PWM_MAX (255)			// Vitesse MAX du train
#define PWM_STOP (0)			// Vitese ARRRET du train

#define NB_TRAJETS	(4) 
#define NB_GARE		(3)  
#define NB_CAPTEUR	(4)  

// Offset pour pointer sur la bonne gare
#define GARE_ST_DIZIER  0		
#define GARE_POSTE 		1
#define GARE_VILLENEUVE 2

// Offset pour pointer sur le bon ils
#define ILS_ST_DIZIER_A			0
#define ILS_POST_B 				1
#define ILS_VILLENEUVE_AVAL_C 	2
#define ILS_VILLENEUVE_AMONT_D 	3


typedef struct 	{	char*	 name_Gare; 		//
					int16_t  temps_gare;			// temps d'attente en gare. 
					int16_t  temps_gare_cpt;
					bool 	 firstAff;
				} ST_GARE;

typedef struct 	{	char* name;			// nom du capteur
					ST_GARE* gare;
					int   pin;
					bool  capt; 		// Lecture brut de l'état du capteur
					bool  captPassage;	// pour limiter l'affichage à une fois au passage. 
					bool  captAck;		// Prise en compte de la position pour effectuer un changement d'état, afin que l'état soit pris qu'une fois en compte et eviter la boucle sans fin si la loco s'arrete sur le capteur. (posssible si vitesse lente)
				} ST_CAPTEUR;

// Tous les temps sont exprimé en multiple de 50ms.
typedef struct 	{	char*	 name_Trajet; 		//
 					ST_GARE* gareDepart;
					ST_GARE* gareArrivee;
					int16_t  pas_acceleration;      // taux d'acceleration.
					int16_t  vitesse_max;			// vitesse MAx du train
					int16_t  temps_vitesse_max;		// temsp à la vitesse max
					int16_t  pas_desceleration;     // taux d'acceleration
					int16_t  vitesse_d_aproche;		// Vitesse d'aproche jusqu'au capteur. ca avance doucement
					int16_t  pas_pour_stoper;       // Etre le capteur et l'arret, taux de desceleration jusqu'a vitesse mini.
					int16_t  vitesse_minimun;      	// Vitesse mini pour ce trajet pour laquell la loco n'avance plus
					bool     sens;
				} ST_TRAJET; 
				
								
ST_GARE tabGare[NB_GARE] = {	{"St Dizier",  	5, 5, true },
								{"Poste",  		4, 4, true },
								{"Ville-Neuve", 3, 3, true },
								} ;
 				

				
//                                                                  									acceleration
//                                                                                                      |            temps_vitesse_max;
//                                                                                                      |            |  pas_desceleration          
//                                                                                                      |            |  |    vitesse_d_aproche          
//                                                                                                      |            |  |    |  pas_pour_stoper          
ST_TRAJET tabTrajet[NB_TRAJETS] = {	{"A->M ",  &tabGare[GARE_ST_DIZIER], 	&tabGare[GARE_VILLENEUVE], 	5, PWM_MAX, 10, 3,  115, 1,  70, false},
									{"M->B ",  &tabGare[GARE_VILLENEUVE], 	&tabGare[GARE_POSTE], 		3, PWM_MAX, 1, 3, 120, 2,  100, false},
									{"B->M ",  &tabGare[GARE_POSTE], 		&tabGare[GARE_VILLENEUVE], 	3, PWM_MAX, 1, 3, 105, 1,  70, true},
									{"M->A ",  &tabGare[GARE_VILLENEUVE], 	&tabGare[GARE_ST_DIZIER], 	5, PWM_MAX, 10, 3,   90, 1,  70, true}		// pro->offsetK ne doit pas d�passer ni egal � ENTREENUMERIQUE 19
								} ; 

ST_CAPTEUR tabCapt[NB_CAPTEUR] =  { {	"ILS_ST_DIZIER", 		&tabGare[GARE_ST_DIZIER], 	CPT_A, false, false, false },
									{	"ILS_POST", 			&tabGare[GARE_POSTE],		CPT_B, false, false, false },
									{	"ILS_VILLENEUVE_AVAL",	&tabGare[GARE_VILLENEUVE], 	CPT_C, false, false, false },
									{	"ILS_VILLENEUVE_AMONT",	&tabGare[GARE_VILLENEUVE], 	CPT_D, false, false, false },
									} ;
								
static int gGare = 0;

//ST_TRAJET* gTrajetEnCours = tabTrajet[3];

int pos = 0;    // variable to store the servo position

typedef enum { TRAIN_ACCELERER, TRAIN_RAPIDE, TRAIN_RALENTIR, TRAIN_LENT, TRAIN_DECELRATION, TRAIN_ARRET} E_ETAT_TRAIN;
typedef enum { DETECT_A,DETECT_B,DETECT_C,DETECT_D, GARE_A, GARE_M, GARE_B, ACCELERE,RAPIDE,DESCELERE,RALENTI,RECHERCHE} E_ETAT;
typedef enum { TRAJET_AM, TRAJET_MB, TRAJET_BM, TRAJET_MA, RECERCHE_A } E_TRAJET;

volatile E_ETAT_TRAIN gEtatTrain = TRAIN_ARRET;
volatile E_ETAT_TRAIN gEtatTrainOld = TRAIN_RALENTIR;
volatile E_TRAJET gTrajet = RECERCHE_A;

volatile int gnbPassage = 0;
volatile int gDuree = 0;
volatile int gVitesse = 0;



// ============================================================
// Prototypage.
// ============================================================
//void desceleration_fin(E_ETAT, int);
void init_sensor();
void init_sensor_aff();
void init_sensor_ack();
void initAllAffGare(void);

void action(void);
void train_acceleration(int taux);
void train_rapide();
void train_ralentir(int taux);
void train_lent();
void train_descelration_fin(int taux);
void train_arret(void);
void calcul_next_trajet();
void afficheCapteur(int i);
void detectForSTOP_immediat(int ils,int gare);
void detectForDescelerationStop(int ils,int gare);


/* ************************************************************************ */
/* ************************************************************************ */
void setup ()
{
	Serial.begin(9600);
  //Serial.print(__FILE__);
  //Serial.print(" ");
	Serial.print(__DATE__);
	Serial.print(" ");
	Serial.println(__TIME__);
  
  //myservo.attach(4);  // attaches the servo on pin 9 to the servo object
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(DIRECTION_PIN, OUTPUT);
	pinMode(PWM_PIN, OUTPUT);
  
	init_sensor();
	init_sensor_aff();
	init_sensor_ack();
	initAllAffGare();

  
  digitalWrite(DIRECTION_PIN, HIGH);
  analogWrite(PWM_PIN, PWM_RECHERCHE);
	for (int i=0 ; i<NB_CAPTEUR ; i++) {
		pinMode(tabCapt[i].pin, INPUT_PULLUP); 	
	}
  
cli();
// ================================================================================
// TIMER 1 16BIT
TCCR1A = 0;
TCCR1B = ( 1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10) ; // Prescaler / 64 ==> des PAS de 5ms
TCCR1C = 0; // not forcing output compare
TCNT1 = 0; // set timer counter initial value (16 bit value)
OCR1A = 1250; // Closer to one second than value above, 1250 pas de 4Âµs ca fait
TIMSK1 = 1 << OCIE1A; // enable timer compare match 1A interrupt
 
sei(); // enable interrupts
}

/* ************************************************************************ */
/* ************************************************************************ */
void loop()
{
	// Surveillance des capteurs en permanance.. 
	for (int i=0 ; i<NB_CAPTEUR ; i++) 
	{
		if (!digitalRead(tabCapt[i].pin)) 	
		{
			init_sensor();
			tabCapt[i].capt = true;
			afficheCapteur(i);
		}
	}
}

/* ************************************************************************ */
/* */
/* ************************************************************************ */
void init_sensor()
{
	for (int i=0 ; i<NB_CAPTEUR ; i++) {
		tabCapt[i].capt = false ;	
	}
}

/* ************************************************************************ */
/* */
/* ************************************************************************ */
void init_sensor_aff()
{	
	for (int i=0 ; i<NB_CAPTEUR ; i++) {
		tabCapt[i].captPassage = false ;	
	}
}

/* ************************************************************************ */
/* */
/* ************************************************************************ */
void init_sensor_ack()
{	
	for (int i=0 ; i<NB_CAPTEUR ; i++) {
		tabCapt[i].captAck = false ;	
	}
}

/* ************************************************************************ */
/* Fonction qui Affiche le capteurs qui vient d'etre traversé
/* ************************************************************************ */
void afficheCapteur(int i)
{
	if (!tabCapt[i].captPassage)	{
		Serial.println(tabCapt[i].name);
		init_sensor_aff();
		tabCapt[i].captPassage = true ;
	}
}
			
/* ************************************************************************ */
/* ON RENTRE DANS SE TIMER TOUTES LES 5ms
/* ************************************************************************ */
ISR(TIMER1_COMPA_vect) // 16 bit timer 1 compare 1A match
{
// Toutes les 5ms on rentre dans cette fonction !
  static int tempo = EACH_50MS;

  if (tempo-- == 0 ) {
    action();
    tempo = EACH_50MS ;
  }
}

/* ************************************************************************ */
// gere l'acceleration
/* ************************************************************************ */
void train_acceleration(int taux)
{
	if (taux ==0 ) { 
		taux = 1;	// scurité soft, sinon le vitesse n'est jamais diminuée
	}
	gVitesse += taux;
	analogWrite(PWM_PIN, gVitesse);
	if (gVitesse >= tabTrajet[gTrajet].vitesse_max ) {
		gEtatTrain = TRAIN_RAPIDE;
		gDuree = tabTrajet[gTrajet].temps_vitesse_max ;
		gVitesse = tabTrajet[gTrajet].vitesse_max;
		Serial.println("TRAIN_RAPIDE");
		analogWrite(PWM_PIN, gVitesse);
	}
}

/* ************************************************************************ */
// gere le temps de la rapidité en multiple de 50ms
/* ************************************************************************ */
void train_rapide()
{
	//Serial.print("RAPIDE:");
	//Serial.println(gDuree);
	if (gDuree-- <= 0 ) {
		gEtatTrain = TRAIN_RALENTIR;
		Serial.println("TRAIN_RALENTIR");
		gDuree = 0;
	}
}


/* ************************************************************************ */
// gere le ralentissement du train
/* ************************************************************************ */
void train_ralentir(int taux)
{
	gVitesse -= taux;
	
	if (gVitesse <= tabTrajet[gTrajet].vitesse_d_aproche ) {
		gVitesse = tabTrajet[gTrajet].vitesse_d_aproche;
		analogWrite(PWM_PIN, gVitesse);
		gEtatTrain = TRAIN_LENT;
		Serial.println("TRAIN_LENT");
	}
	else{
		analogWrite(PWM_PIN, gVitesse);
	}
}

/* ************************************************************************ */
// Le train avance lentement jusqu'a detection du detecteur de voie.
/* ************************************************************************ */
void train_lent()
{
	analogWrite(PWM_PIN, gVitesse);
}

/* ************************************************************************ */
// Le train descelere jusqu'a l'arret, prend 10cm sur la voie...
/* ************************************************************************ */
void train_descelration_fin(int taux)
{
	if (taux <= 0 ){
		taux = 5;
	}
	gVitesse-=taux;
	/*
	Serial.print("train_descelration_fin:");
	Serial.println(taux);
	Serial.print("gVitesse:");
	Serial.println(gVitesse);
	*/
	
	if ( gVitesse <= tabTrajet[gTrajet].vitesse_minimun)
	{
		gVitesse = PWM_STOP;
		analogWrite(PWM_PIN, gVitesse);
		gEtatTrain = TRAIN_ARRET;
		Serial.println("TRAIN_ARRET");
		//Serial.println(gVitesse);
	}else{
		analogWrite(PWM_PIN, gVitesse);	
		//Serial.println(gVitesse);		
	}
}

/* ************************************************************************ */
// Le train est a l'arret pendant un certain temps
/* ************************************************************************ */
void train_arret(ST_GARE* gare)
{
	
	static int sDureeSeconde = 20;
	//static int sCmptSeconde = 0;
	
	if (gare->firstAff) {
		Serial.print("GARE:");
		Serial.println(gare->name_Gare);
		initAllAffGare();
		gare->firstAff = false;
	}
	
	if (sDureeSeconde-- == 0){
		sDureeSeconde = 20 ;// 20 x 50ms = 1s
		
		Serial.print(" ");
		Serial.print(gare->temps_gare_cpt);
		
		if (gare->temps_gare_cpt-- == 0){
			Serial.println("");
			Serial.println("Depart");
			// Remise à l'init pour le prohain arret.
			gare->temps_gare_cpt = gare->temps_gare; // réinitialisation d la durée écoulée
			calcul_next_trajet();
			
			Serial.print("Trajet: ");
			Serial.print(tabTrajet[gTrajet].name_Trajet);
			Serial.print(": ");
			Serial.print(tabTrajet[gTrajet].gareDepart->name_Gare );
			Serial.print(" => ");
			Serial.println(tabTrajet[gTrajet].gareArrivee->name_Gare );

			Serial.print("Sens: ");
			if (tabTrajet[gTrajet].sens)	{
				Serial.println("Sens: AVANT");
			} else {
				Serial.println("Sens: ARRIERE");
			}
			
			digitalWrite(DIRECTION_PIN, tabTrajet[gTrajet].sens);	
			gEtatTrain = TRAIN_ACCELERER;
			Serial.println("TRAIN_ACCELERER");
					
		}
	}
}


/******************************************************/
// initialise le flag des Gare pour le prochain affichage
void initAllAffGare(void)
{
	for (int i=0 ; i<NB_GARE ; i++)	{
		tabGare[i].firstAff = true;
	}
}


/******************************************************/
/* FONCTION APPELLEE TOUTES LES 50ms                  */
void calcul_next_trajet()
{
		switch(gTrajet)
		{
			case TRAJET_AM: gTrajet = TRAJET_MB ; break;
			case TRAJET_MB: gTrajet = TRAJET_BM ; break;
			case TRAJET_BM: gTrajet = TRAJET_MA ; break;
			case TRAJET_MA: gTrajet = TRAJET_AM ; break;
			default: Serial.println("ERROR 1"); break;
		}
		
}

/******************************************************/
/* FONCTION APPELLEE TOUTES LES 50ms                  */
void action()
{
	static bool gCptB_acquit = false ;
	
	
	if ( gTrajet == RECERCHE_A)
	{
		if (tabCapt[ILS_ST_DIZIER_A].capt){
			Serial.println("EN GARE A INIT");
			gVitesse = PWM_STOP;
			analogWrite(PWM_PIN, gVitesse);
			gEtatTrain = TRAIN_ARRET ;
			gTrajet = TRAJET_MA ; // façon de faire pour ne plus jammais retomber dans la fonction recherche
		}
		return;
	}

	
	if (gTrajet == TRAJET_MA)	{ 
		detectForDescelerationStop(ILS_ST_DIZIER_A,GARE_ST_DIZIER);
		detectForSTOP_immediat(ILS_VILLENEUVE_AVAL_C,GARE_ST_DIZIER);
	}
	if (gTrajet == TRAJET_MB)	{ 
		detectForDescelerationStop(ILS_POST_B,GARE_POSTE);
		detectForSTOP_immediat(ILS_VILLENEUVE_AMONT_D,GARE_POSTE);
	}
	
	if (gTrajet == TRAJET_AM)	{ 
		detectForDescelerationStop(ILS_VILLENEUVE_AVAL_C,GARE_VILLENEUVE);
		detectForSTOP_immediat(ILS_ST_DIZIER_A, GARE_VILLENEUVE);
	}

	if (gTrajet == TRAJET_BM)	{ 
		detectForDescelerationStop(ILS_VILLENEUVE_AMONT_D,GARE_VILLENEUVE);
		detectForSTOP_immediat(ILS_POST_B,GARE_VILLENEUVE);
	}
	
	switch(gEtatTrain){
		case TRAIN_ACCELERER : 		
				train_acceleration(tabTrajet[gTrajet].pas_acceleration); 	
				break;
		case TRAIN_RAPIDE :       	
				train_rapide();												
				break;
		case TRAIN_RALENTIR :      	
				train_ralentir(tabTrajet[gTrajet].pas_desceleration);	 	
				break;
		case TRAIN_LENT :      		
				train_lent();												
				break;
		case TRAIN_DECELRATION :  	
				train_descelration_fin(tabTrajet[gTrajet].pas_pour_stoper); 
				break	;
		case TRAIN_ARRET :        	
				train_arret(&tabGare[gGare]); 
				break;
		default :  
				Serial.print("ERREUR:"); 
				analogWrite(PWM_PIN, PWM_STOP);
				break;
	}
}

/******************************************************/
/* detection ILS pour arreter le train                */
void detectForDescelerationStop(int ils,int gare)
{
	if (tabCapt[ils].capt )
	{
		if (tabCapt[ils].captAck != tabCapt[ils].capt)
		{
			gEtatTrain = TRAIN_DECELRATION ;	
			gGare = gare;
			Serial.println("B_TRAIN_DECELRATION");
			init_sensor_ack();
			tabCapt[ils].captAck = true;
		}
	}
}

/******************************************************/
/* Détection ARRET Cause dépassement
/* dans le cas ou l'arret dépasse le capteur..                                                   */
void detectForSTOP_immediat(int ils,int gare)
{
	
	for (int i=0 ; i>NB_CAPTEUR ; i++) {
		if (i==ils) {
			continue; 
		}
		if (tabCapt[ils].capt ){
			gEtatTrain = TRAIN_ARRET ;
			gGare = gare;
			Serial.println("TRAIN_STOP_IMMEDIAT");
			analogWrite(PWM_PIN, 0);
		}
	}
	
}


/******************************************************/
void enGare()
{
}
