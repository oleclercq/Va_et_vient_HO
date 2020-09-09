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

#define PWM_RECHERCHE (160)
#define PWM_MIN_STOP (50) // PWM à la quelle le train n'avence pas
#define PWM_MIN (100)	// Vitesse Minimum, jusqu'a attendre la butée (inter)
#define PWM_MAX (255)	// Vitesse MAX
#define PWM_STOP (0)
#define DUREE_EN_GARE	(100)	// 100 * 50ms = 5000ms = 5s;

#define TAUX_ACCELERATION	(2) // On acceler moyennement
#define TAUX_DESCELERATION	(3) // La deceleration est rapide
#define TAUX_STOP			(1) // La deceleration pour s'arreter est longue.


#define DUREE_VITESSE_MIN (100) // 50 passa ge cadencé ) 50ms  =50x50 = 2.5s
#define DUREE_VITESSE_MAX (50) // 50 passa ge cadencé ) 50ms  =50x50 = 2.5s

#define NB_PASS_DETECT	(100) // temps avant de détecter la gare d'arrivée

#define NB_TRAJETS	(4) 
#define NB_GARE		(4)  // 0 n'est pas une gare


#define GARE_ST_DIZIER  1
#define GARE_POSTE 		2
#define GARE_VILLENEUVE 3

// ous les temps sont exprimé en multiple de 50ms.
typedef struct 	{	char*	 name_Trajet; 		//
					int16_t  pas_acceleration;      // taux d'acceleration.
					int16_t  vitesse_max;			// vitesse MAx du train
					int16_t  temps_vitesse_max;		// temsp à la vitesse max
					int16_t  pas_desceleration;     // taux d'acceleration
					int16_t  vitesse_d_aproche;		// Vitesse d'aproche jusqu'au capteur. ca avance doucement
					int16_t  pas_pour_stoper;       // Etre le capteur et l'arret, taux de desceleration jusqu'a vitesse mini.
					int16_t  vitesse_minimun;      	// Vitesse mini pour ce trajet pour laquell la loco n'avance plus
				} ST_TRAJET; 
				
typedef struct 	{	char*	 name_Gare; 		//
					int16_t  temps_gare;			// temps d'attente en gare. 
					int16_t  temps_gare_cpt;
					bool 	 firstAff;
				} ST_GARE; 

				
ST_TRAJET tabTrajet[NB_TRAJETS] = {	{"A -> M",  TAUX_ACCELERATION, PWM_MAX, DUREE_VITESSE_MAX, TAUX_DESCELERATION, PWM_MIN, PWM_MIN_STOP},
									{"M -> B",  TAUX_ACCELERATION, PWM_MAX, DUREE_VITESSE_MAX, TAUX_DESCELERATION, PWM_MIN, PWM_MIN_STOP},
									{"B -> M",  TAUX_ACCELERATION, PWM_MAX, DUREE_VITESSE_MAX, TAUX_DESCELERATION, PWM_MIN, PWM_MIN_STOP},
									{"M -> A",  TAUX_ACCELERATION, PWM_MAX, DUREE_VITESSE_MAX, TAUX_DESCELERATION, PWM_MIN, PWM_MIN_STOP}		// pro->offsetK ne doit pas d�passer ni egal � ENTREENUMERIQUE 19
								} ; 
ST_GARE tabGare[NB_GARE] = {	{"INCONNUE",  	10, DUREE_EN_GARE, true },
								{"St Dizier",  	5, DUREE_EN_GARE, true },
								{"Poste",  		7, DUREE_EN_GARE, true },
								{"Ville Neuve", 9, DUREE_EN_GARE, true },
								} ; 
								

static bool gSensDeMarche_AB = false;
static int gDirection = 0;
static int gGare = 0;

//Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

typedef enum { TRAIN_ACCELERER, TRAIN_RAPIDE, TRAIN_RALENTIR, TRAIN_LENT, TRAIN_DECELRATION, TRAIN_ARRET} E_ETAT_TRAIN;
typedef enum { DETECT_A,DETECT_B,DETECT_C,DETECT_D, GARE_A, GARE_M, GARE_B, ACCELERE,RAPIDE,DESCELERE,RALENTI,RECHERCHE} E_ETAT;
typedef enum { RECERCHE_A,TRAJET_AM, TRAJET_MB, TRAJET_BM, TRAJET_MA } E_TRAJET;

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


int gInter_A = -1;
int gInter_B = -1;
int gInter_C = -1;
int gInter_D = -1;

int gPosA_old = -1;
int gPosB_old = -1;
int gPosC_old = -1;
int gPosD_old = -1;
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

  //myservo.attach(4);  // attaches the servo on pin 9 to the servo object
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(DIRECTION_PIN, HIGH);
  gSensDeMarche_AB = false;
  analogWrite(PWM_PIN, PWM_RECHERCHE);


  pinMode(CPT_A, INPUT_PULLUP); 
  pinMode(CPT_B, INPUT_PULLUP);
  pinMode(CPT_C, INPUT_PULLUP); 
  pinMode(CPT_D, INPUT_PULLUP); 
  Serial.begin(9600);
  Serial.print(__FILE__);
  Serial.print(" ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

   

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
  
  if (!digitalRead(CPT_A)) 	{
		init_sensor();
		gCptA = true;
		if (gCptA_old != gCptA)	{
			Serial.println("***** A *****");
			init_sensor_aff();
			gCptA_old = gCptA;
		}
  }
  
  if (!digitalRead(CPT_B)) 	{
		init_sensor();
		gCptB = true;
		if (gCptB_old != gCptB)	{
			Serial.println("***** B *****");
			init_sensor_aff();
			gCptB_old = gCptB;
		}
  }
  if (!digitalRead(CPT_C)) 	{
		init_sensor();
		gCptC = true;
		if (gCptC_old != gCptC)	{
			Serial.println("***** C *****");
			init_sensor_aff();
			gCptC_old = gCptC;
		}
  }
  
  if (!digitalRead(CPT_D)) 	{
		init_sensor();
		gCptD = true;
		if (gCptD_old != gCptD)	{
			Serial.println("***** D *****");
			init_sensor_aff();
			gCptD_old = gCptD;
		}
  }
}

/* ************************************************************************ */
/* */
/* ************************************************************************ */
void init_sensor()
{
	gCptA = false ;
	gCptB = false ;
	gCptC = false ;
	gCptD = false ;
}
void init_sensor_aff()
{	
	gCptA_old = false ;
	gCptB_old = false ;
	gCptC_old = false ;
	gCptD_old = false ;
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
			tabGare[gGare].temps_gare_cpt = tabGare[gGare].temps_gare; // réinitialisation d la durée écoulée
			calcul_next_trajet();
			digitalWrite(DIRECTION_PIN, !gSensDeMarche_AB);
			gEtatTrain = TRAIN_ACCELERER;
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
			case TRAJET_AM: gTrajet = TRAJET_MB ; Serial.println("TRAJET_MB");	gSensDeMarche_AB=true; break;
			case TRAJET_MB: gTrajet = TRAJET_BM ; Serial.println("TRAJET_BM");	gSensDeMarche_AB=false; break;
			case TRAJET_BM: gTrajet = TRAJET_MA ; Serial.println("TRAJET_MA");	gSensDeMarche_AB=false; break;
			case TRAJET_MA: gTrajet = TRAJET_AM ; Serial.println("TRAJET_AM");	gSensDeMarche_AB=true;  break;
			default: Serial.println("ERROR 1"); break;
		}
		
}

/******************************************************/
/* FONCTION APPELLEE TOUTES LES 50ms                  */
void action()
{
	
	
	
	if ( gTrajet == RECERCHE_A)
	{
		if (gCptA){
			//Serial.println("EN GARE A INIT");
			gVitesse = PWM_STOP;
			analogWrite(PWM_PIN, gVitesse);
			
			gTrajet = TRAJET_MA;
			gEtatTrain = TRAIN_ARRET ;
			
			gSensDeMarche_AB = true;
			
		}
		return;
	}
	
	if (gCptA){
		if (gTrajet == TRAJET_MA)
		{
			//Serial.print("TRAJET_MA:");
			gEtatTrain = TRAIN_DECELRATION ;
			Serial.println("A_TRAIN_DECELRATION");
			gCptA = false;
			gGare = GARE_ST_DIZIER;
		}
	}
	if (gCptB){
		if (gTrajet == TRAJET_MB)
		{ 
			//Serial.print("TRAJET_MB:");
			//calcul_next_trajet();
			gEtatTrain = TRAIN_DECELRATION ;	
			Serial.println("B_TRAIN_DECELRATION");
			gCptB = false;
			gGare = GARE_POSTE;
		}
	}
	if (gCptC){
		if (gTrajet == TRAJET_AM)
		{
			//Serial.print("TRAJET_AM:");
			//calcul_next_trajet();
			gEtatTrain = TRAIN_DECELRATION ;
			Serial.println("C_TRAIN_DECELRATION");
			gCptC = false;
			gGare = GARE_VILLENEUVE;
		}
	}
	if (gCptD){
		if (gTrajet == TRAJET_BM)
		{ 
			//Serial.print("TRAJET_BM:");
			//calcul_next_trajet();
			gEtatTrain = TRAIN_DECELRATION ;
			Serial.println("D_TRAIN_DECELRATION");
			gCptD = false;
			gGare = GARE_VILLENEUVE;
			
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
{/*
	static int gDureeSeconde=20;
		
      
      //    Premier traitement
      if (gEtatOld != gEtat ) 
      {
		  analogWrite(PWM_PIN, PWM_STOP);
          if (gEtat == GARE_A){
            Serial.println("** GARE_A **");
            analogWrite(PWM_PIN, PWM_STOP);
			digitalWrite(DIRECTION_PIN, LOW); 
			gSensDeMarche_AB = true;
			Serial.println("gSensDeMarche_AB = true");
			gDuree = 2;
          }

          else if (gEtat == GARE_B){
            Serial.println("** GARE_B **");
			digitalWrite(DIRECTION_PIN, HIGH); 
			gSensDeMarche_AB = false;
			Serial.println("gSensDeMarche_AB = false");
			gDuree = 3;
          }
		  
		  else if (gEtat == GARE_M){
			  Serial.println("** GARE_M **");
			  gDuree = 3;
		  }
         gEtatOld = gEtat;
      }
	  // On affiche le décompteur en Gare.
	  
	if (gEtatOld != gEtat ) 
    {
	}
		if (gDureeSeconde-- == 0){
			Serial.println("EN GARE");
			gDureeSeconde = 20 ;// 20 x 50ms = 1s
			
			if (gDuree-- <= 0 ) {
				gEtat = ACCELERE;
			}
		}

//    Dernier Traitement
      */
}
