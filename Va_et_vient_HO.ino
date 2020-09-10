/* Sweep
il faut 2 inter sur la gare, (une gare suffit)
*/



#define PWM_PIN (3) // D7
#define CPT_A (5) // D7
#define CPT_B (6) // D7
#define CPT_C (A2) // D7
#define CPT_D (A3) // D7

#define DIRECTION_PIN (12)
#define EACH_100MS (20)
#define EACH_50MS (10)

#define PWM_RECHERCHE (255)
#define PWM_MIN_STOP (50) // PWM à la quelle le train n'avence pas
#define PWM_MIN (120)	// Vitesse Minimum, jusqu'a attendre la butée (inter)
#define PWM_MAX (255)	// Vitesse MAX
#define PWM_STOP (0)
#define DUREE_EN_GARE	(100)	// 100 * 50ms = 5000ms = 5s;

#define TAUX_ACCELERATION	(2) // On acceler moyennement
#define TAUX_DESCELERATION	(3) // La deceleration est rapide
#define TAUX_STOP			(1) // La deceleration pour s'arreter est longue.


#define DUREE_VITESSE_MAX (20) // 50 passa ge cadencé ) 50ms  =50x50 = 2.5s

#define NB_PASS_DETECT	(100) // temps avant de détecter la gare d'arrivée

#define NB_TRAJETS	(4) 
#define NB_GARE		(3)  
#define NB_CAPTEUR	(4)  

#define GARE_ST_DIZIER  0
#define GARE_POSTE 		1
#define GARE_VILLENEUVE 2

#define ILS_ST_DIZIER_A			0
#define ILS_POST_B 				1
#define ILS_VILLENEUVE_AVAL_C 	2
#define ILS_VILLENEUVE_AMONT_D 	3


typedef struct 	{	char* name;			// nom du capteur
					int   pin;
					bool  capt; 		// Lecture brut de l'état du capteur
					bool  captPassage;	// pour limiter l'affichage à une fois au passage. 
					bool  captAck;		// Prise en compte de la position pour effectuer un changement d'état, afin que l'état soit pris qu'une fois en compte et eviter la boucle sans fin si la loco s'arrete sur le capteur. (posssible si vitesse lente)
				} ST_CAPTEUR;


typedef struct 	{	char*	 name_Gare; 		//
					int16_t  temps_gare;			// temps d'attente en gare. 
					int16_t  temps_gare_cpt;
					bool 	 firstAff;
				} ST_GARE;


// ous les temps sont exprimé en multiple de 50ms.
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
				
ST_CAPTEUR tabCapt[NB_CAPTEUR] =  { {	"ILS_ST_DIZIER", 		CPT_A, false, false, false },
									{	"ILS_POST", 			CPT_B, false, false, false },
									{	"ILS_VILLENEUVE_AVAL",	CPT_C, false, false, false },
									{	"ILS_VILLENEUVE_AMONT",	CPT_D, false, false, false },
									} ;
								
ST_GARE tabGare[NB_GARE] = {	{"St Dizier",  	5, 5, true },
								{"Poste",  		7, 7, true },
								{"Ville-Neuve", 9, 9, true },
								} ;
 				

				
ST_TRAJET tabTrajet[NB_TRAJETS] = {	
									{"A -> M",  &tabGare[GARE_ST_DIZIER], 	&tabGare[GARE_VILLENEUVE], 	TAUX_ACCELERATION, PWM_MAX, DUREE_VITESSE_MAX, TAUX_DESCELERATION, PWM_MIN, TAUX_STOP, PWM_MIN_STOP, false},
									{"M -> B",  &tabGare[GARE_VILLENEUVE], 	&tabGare[GARE_POSTE], 		TAUX_ACCELERATION, PWM_MAX, DUREE_VITESSE_MAX, TAUX_DESCELERATION, PWM_MIN, TAUX_STOP, PWM_MIN_STOP, false},
									{"B -> M",  &tabGare[GARE_POSTE], 		&tabGare[GARE_VILLENEUVE], 	TAUX_ACCELERATION, PWM_MAX, DUREE_VITESSE_MAX, TAUX_DESCELERATION, PWM_MIN, TAUX_STOP, PWM_MIN_STOP, true},
									{"M -> A",  &tabGare[GARE_VILLENEUVE], 	&tabGare[GARE_ST_DIZIER], 	TAUX_ACCELERATION, PWM_MAX, DUREE_VITESSE_MAX, TAUX_DESCELERATION, PWM_MIN, TAUX_STOP, PWM_MIN_STOP, true}		// pro->offsetK ne doit pas d�passer ni egal � ENTREENUMERIQUE 19
								} ; 

								
static int gGare = 0;

//Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

typedef enum { TRAIN_ACCELERER, TRAIN_RAPIDE, TRAIN_RALENTIR, TRAIN_LENT, TRAIN_DECELRATION, TRAIN_ARRET} E_ETAT_TRAIN;
typedef enum { DETECT_A,DETECT_B,DETECT_C,DETECT_D, GARE_A, GARE_M, GARE_B, ACCELERE,RAPIDE,DESCELERE,RALENTI,RECHERCHE} E_ETAT;
typedef enum { TRAJET_AM, TRAJET_MB, TRAJET_BM, TRAJET_MA, RECERCHE_A } E_TRAJET;

volatile E_ETAT_TRAIN gEtatTrain = TRAIN_ARRET;
volatile E_ETAT_TRAIN gEtatTrainOld = TRAIN_RALENTIR;
volatile E_TRAJET gTrajet = RECERCHE_A;

volatile int gnbPassage = 0;
volatile int gDuree = DUREE_EN_GARE;

volatile int gInterA_old = -1;
volatile int gInterB_old = -1;
volatile int gInterC_old = -1;
volatile int gInterD_old = -1;

volatile bool gCptA = false;
volatile bool gCptB = false;
volatile bool gCptC = false;
volatile bool gCptD = false;

volatile bool gCptA_old = false;
volatile bool gCptB_old = false;
volatile bool gCptC_old = false;
volatile bool gCptD_old = false;

int gPosA = 0;
int gPosB = 0;
int gPosC = 0;
int gPosD = 0;

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
  delay(1000);
  
  //myservo.attach(4);  // attaches the servo on pin 9 to the servo object
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(DIRECTION_PIN, HIGH);
  analogWrite(PWM_PIN, PWM_RECHERCHE);
	for (int i=0 ; i<NB_CAPTEUR ; i++) {
		pinMode(tabCapt[i].pin, INPUT_PULLUP); 	
	}
  
  pinMode(CPT_B, INPUT_PULLUP);
  pinMode(CPT_C, INPUT_PULLUP); 
  pinMode(CPT_D, INPUT_PULLUP); 
  


   

;
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
	
	// on surveilles capteurs en permanance..
	for (int i=0 ; i<NB_CAPTEUR ; i++) 
	{
		//Serial.println(tabCapt[i].name);
		if (!digitalRead(tabCapt[i].pin)) 	
		{
			init_sensor();
			tabCapt[i].capt = true;
			if (!tabCapt[i].captPassage)	{
				Serial.println(tabCapt[i].name);
				init_sensor_aff();
				tabCapt[i].captPassage = true ;
			}
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
	gVitesse += taux;
	analogWrite(PWM_PIN, gVitesse);
	//Serial.print("ACCELERATION:");
	//Serial.println(gVitesse);
	if (gVitesse >= PWM_MAX ) {
		gEtatTrain = TRAIN_RAPIDE;
		gDuree = DUREE_VITESSE_MAX;
		gVitesse = PWM_MAX;
		Serial.println("TRAIN_RAPIDE");
		// Serial.print("**");
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
	}
}


/* ************************************************************************ */
// gere le ralentissement du train
/* ************************************************************************ */
void train_ralentir(int taux)
{
	gVitesse -= taux;
	
	//Serial.print("RALENTIR:");
	//Serial.println(gVitesse);
	
	if (gVitesse <= PWM_MIN ) {
		gVitesse = PWM_MIN;
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
	gVitesse-=taux;
	//Serial.print("DECELERATION:");
	if ( gVitesse <= PWM_MIN_STOP)
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
void train_arret(int duree)
{
	
	static int sDureeSeconde = 20;
	static int sCmptSeconde = 0;
	
	if (tabGare[gGare].firstAff) {
		Serial.print("GARE:");
		Serial.println(tabGare[gGare].name_Gare);
		initAllAffGare();
		tabGare[gGare].firstAff = false;
	}
	
	if (sDureeSeconde-- == 0){
		sDureeSeconde = 20 ;// 20 x 50ms = 1s
		
		Serial.print(" ");
		Serial.print(++sCmptSeconde);
		
		if (tabGare[gGare].temps_gare_cpt-- == 0){
			Serial.println("");
			Serial.println("Depart");
			// Remise à l'init pour le prohain arret.
			tabGare[gGare].temps_gare_cpt = tabGare[gGare].temps_gare; // réinitialisation d la durée écoulée
			calcul_next_trajet();
			
			Serial.print("gTrajet=");
			Serial.print(gTrajet);
			Serial.print(" ");
			Serial.print(tabTrajet[gTrajet].name_Trajet);
			Serial.print(": ");
			Serial.print(tabTrajet[gTrajet].gareDepart->name_Gare );
			Serial.print(" => ");
			Serial.println(tabTrajet[gTrajet].gareArrivee->name_Gare );

			Serial.print("Sens: ");
			Serial.println(tabTrajet[gTrajet].sens);
			
			
			if (tabTrajet[gTrajet].sens)
			{
				Serial.println("Sens: AVANT");
			} else {
				Serial.println("Sens: ARRIERE");
			}
			digitalWrite(DIRECTION_PIN, tabTrajet[gTrajet].sens);	
			gEtatTrain = TRAIN_ACCELERER;
			Serial.println("TRAIN_ACCELERER");
			sCmptSeconde = 0;		
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
	
	if (gTrajet == TRAJET_MA)
	{
		if (tabCapt[ILS_ST_DIZIER_A].capt)
		{
			if (tabCapt[ILS_ST_DIZIER_A].captAck != tabCapt[ILS_ST_DIZIER_A].capt)
			{
				gEtatTrain = TRAIN_DECELRATION ;
				Serial.println("A_TRAIN_DECELRATION");
				gGare = GARE_ST_DIZIER;
				
				init_sensor_ack();
				tabCapt[ILS_ST_DIZIER_A].captAck = true;
			}
		}
	}
	if (gTrajet == TRAJET_MB)
	{ 
		if (tabCapt[ILS_POST_B].capt )
		{
			if (tabCapt[ILS_POST_B].captAck != tabCapt[ILS_POST_B].capt)
			{
				gEtatTrain = TRAIN_DECELRATION ;	
				Serial.println("B_TRAIN_DECELRATION");
				gGare = GARE_POSTE;
				
				init_sensor_ack();
				tabCapt[ILS_POST_B].captAck = true;
			}
		}
	}
	
	if (gTrajet == TRAJET_AM)
	{
		if (tabCapt[ILS_VILLENEUVE_AVAL_C].capt)
		{
			if (tabCapt[ILS_VILLENEUVE_AVAL_C].captAck != tabCapt[ILS_VILLENEUVE_AVAL_C].capt)
			{
				gEtatTrain = TRAIN_DECELRATION ;
				Serial.println("C_TRAIN_DECELRATION");
				gGare = GARE_VILLENEUVE;
			
				init_sensor_ack();
				tabCapt[ILS_VILLENEUVE_AVAL_C].captAck = true;
			}
		}
	}
	if (gTrajet == TRAJET_BM)
	{ 
		if (tabCapt[ILS_VILLENEUVE_AMONT_D].capt)
		{
			if (tabCapt[ILS_VILLENEUVE_AMONT_D].captAck != tabCapt[ILS_VILLENEUVE_AMONT_D].capt)
			{
				gEtatTrain = TRAIN_DECELRATION ;
				Serial.println("D_TRAIN_DECELRATION");
				gGare = GARE_VILLENEUVE;
			
				init_sensor_ack();
				tabCapt[ILS_VILLENEUVE_AMONT_D].captAck = true;
			}
		}
	}
	
	switch(gEtatTrain){
		case TRAIN_ACCELERER : 		train_acceleration(5);	break;
		case TRAIN_RAPIDE :       	train_rapide();			break;
		case TRAIN_RALENTIR :      	train_ralentir(5);		break;
		case TRAIN_LENT :      		train_lent();			break;
		case TRAIN_DECELRATION :  	train_descelration_fin(5);	break;
		case TRAIN_ARRET :        	train_arret(DUREE_EN_GARE); break;
		
		default :  
			Serial.print("RAPIDE:"); 
			gVitesse = PWM_STOP;		
			analogWrite(PWM_PIN, gVitesse);
			break;
	}
}


/******************************************************/
void enGare()
{
}
