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

#define PWM_RECHERCHE (150)
#define PWM_MIN_STOP (50) // PWM à la quelle le train n'avence pas
#define PWM_MIN (150)	// Vitesse Minimum, jusqu'a attendre la butée (inter)
#define PWM_MAX (255)	// Vitesse MAX
#define PWM_STOP (0)
#define DUREE_EN_GARE	(100)	// 100 * 50ms = 5000ms = 5s;

#define DUREE_VITESSE_MIN (100) // 50 passa ge cadencé ) 50ms  =50x50 = 2.5s
#define DUREE_VITESSE_MAX (120) // 50 passa ge cadencé ) 50ms  =50x50 = 2.5s

#define NB_PASS_DETECT	(100) // temps avant de détecter la gare d'arrivée



static bool gSensDeMarche_AB = false;
static int gDirection = 0;

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

  /*Serial.print("A=");
  Serial.println(gInter_A);
  
  Serial.print("B=");
  Serial.println(gInter_B);*/

/*
    if (gInter_A==LOW)
    {
		init_sensor();
		gCptA = TRUE;
		if (gPosA_old != gPosA)	{
			Serial.println("***** A *****");
			init_sensor();
			gPosA_old = gPosA;
		}
		
        if (gInterA_old != gInter_A) // pour l'anti rebond
        {
            //analogWrite(PWM_PIN, PWM_STOP);
            Serial.println("gInter_A=0*");
            gDuree = DUREE_EN_GARE; // 10 * 100ms = 1000ms;
            gInterA_old = gInter_A;
			gInterB_old = -10;
			if ( gEtat == RECHERCHE)
			{gEtat = GARE_A;} else {gEtat = DETECT_A;}
        }
    }
    
    if (gInter_B==LOW) 
    {
		if (gPosB_old != gPosB)	{
			Serial.println("***** B *****");
			init_sensor();
			gPosB_old = gPosB;
		}
        if (gInterB_old != gInter_B) // pour l'anti rebond
        {
          //analogWrite(PWM_PIN, PWM_STOP);
          Serial.println("gInter_B=0*");
            gDuree = DUREE_EN_GARE; 
            gInterB_old = gInter_B;
			gInterA_old = -10;
			if ( gEtat == RECHERCHE)
			{gEtat = GARE_B;} else {gEtat = DETECT_B;}
        }
    }
	
	
	if (gInter_C == LOW)
    {
		if (gPosC_old != gPosC)	{
			Serial.println("***** C *****");
			init_sensor();
			gPosC_old = gPosC;
			if ( gEtat != RECHERCHE){
				gDuree = DUREE_EN_GARE;
				gEtat = DETECT_C;
			}
			
		}
	}
		
	if (gInter_D == LOW)
    {
		if (gPosD_old != gPosD)	{
			Serial.println("***** D *****");
			init_sensor();
			gPosD_old = gPosD;
		}
	}*/
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
  // toutes les 5ms on rentre dans cette fonction !
  
  static int tempo = EACH_50MS;
  static uint16_t iLed = 0;

  if (tempo-- == 0 )
  {
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
	Serial.print("ACCELERATION:");
	Serial.println(gVitesse);
	if (gVitesse >= PWM_MAX ) {
		gEtatTrain = TRAIN_RAPIDE;
		gVitesse = PWM_MAX;
		Serial.print("**");
		analogWrite(PWM_PIN, gVitesse);
	}
}

/* ************************************************************************ */
// gere le temps de la rapidité en multiple de 50ms
/* ************************************************************************ */
void train_rapide()
{
	Serial.println("RAPIDE:");
	if (gDuree-- <= 0 ) {
		gEtatTrain = TRAIN_RALENTIR;
	}
}


/* ************************************************************************ */
// gere le ralentissement du train
/* ************************************************************************ */
void train_ralentir(int taux)
{
	gVitesse -= taux;
	
	Serial.print("RALENTIR:");
	Serial.println(gVitesse);
	
	if (gVitesse <= PWM_MIN ) {
		gVitesse = PWM_MIN;
		analogWrite(PWM_PIN, gVitesse);
		gEtatTrain = TRAIN_LENT;
	}
	else{
		analogWrite(PWM_PIN, gVitesse);
	}
}

/* ************************************************************************ */
// Le train avance lentement jusqu'a detection du detecteurde voie.
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
	Serial.print("DECELERATION:");
	Serial.println(gVitesse);
	if ( gVitesse <= PWM_MIN_STOP)
	{
		gVitesse = PWM_STOP;
		analogWrite(PWM_PIN, gVitesse);
		gEtatTrain = TRAIN_ARRET;
		Serial.println(gVitesse);
	}else{
		analogWrite(PWM_PIN, gVitesse);	
		Serial.println(gVitesse);		
	}
}

/* ************************************************************************ */
// Le train est a l'arret pendant un certain temps
/* ************************************************************************ */
void train_arret(int duree)
{
	static int gDureeSeconde = 20;
	if (gDureeSeconde-- == 0){
			Serial.println("EN GARE");
			gDureeSeconde = 20 ;// 20 x 50ms = 1s
	}
		
	if (gDuree-- == 0){
		Serial.println("Depart");
		gDureeSeconde = duree ;// 20 x 50ms = 1s
		calcul_next_trajet();
		digitalWrite(DIRECTION_PIN, !gSensDeMarche_AB);
		gEtatTrain = TRAIN_ACCELERER;	
	}
}

/******************************************************/
/* FONCTION APPELLEE TOUTES LES 50ms                  */

void calcul_next_trajet()
{
		switch(gTrajet)
		{
			case TRAJET_AM: gTrajet = TRAJET_MB ; Serial.println("_MB");	gSensDeMarche_AB=true; break;
			case TRAJET_MB: gTrajet = TRAJET_BM ; Serial.println("_BM");	gSensDeMarche_AB=false; break;
			case TRAJET_BM: gTrajet = TRAJET_MA ; Serial.println("_MA");	gSensDeMarche_AB=false; break;
			case TRAJET_MA: gTrajet = TRAJET_AM ; Serial.println("_AM");	gSensDeMarche_AB=true;  break;
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
			Serial.println("EN GARE A INIT");
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
			Serial.print("TRAJET_MA:");
			gEtatTrain = TRAIN_DECELRATION ;
			gCptA = false;
			gDuree = DUREE_EN_GARE;
		}
	}
	if (gCptB){
		if (gTrajet == TRAJET_MB)
		{ 
			Serial.print("TRAJET_MB:");
			gEtatTrain = TRAIN_DECELRATION ;	
			gCptB = false;
			gDuree = DUREE_EN_GARE;
		}
	}
	if (gCptC){
		if (gTrajet == TRAJET_AM)
		{
			Serial.print("TRAJET_AM:");
			gEtatTrain = TRAIN_DECELRATION ;
			gCptC = false;
			gDuree = DUREE_EN_GARE;
		}
	}
	if (gCptD){
		if (gTrajet == TRAJET_BM)
		{ 
			Serial.print("TRAJET_BM:");
			gEtatTrain = TRAIN_DECELRATION ;	
			gCptD = false;
			gDuree = DUREE_EN_GARE;
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
	
	/*
  switch(gEtat)
  {
	case DETECT_A : desceleration_fin(GARE_A,2); break;
	case DETECT_B : desceleration_fin(GARE_B,2); break;
	case DETECT_C : 
		if ( gSensDeMarche_AB)	{
			desceleration_fin(GARE_M,10); 
		}
		break;
	case DETECT_D : 
		if ( !gSensDeMarche_AB)	{
			desceleration_fin(GARE_M,10); 
		}
		break;
	  
    case GARE_A: 
    case GARE_B: 	enGare();	break;
      
    case ACCELERE: 

      if (gEtatOld != gEtat ) {
			Serial.println("ACCELERE");
			gEtatOld = gEtat;
			gVitesse = PWM_MIN;
			gnbPassage = 0;
      }

      if (gnbPassage++ > NB_PASS_DETECT)
      {
          Serial.println("ACCELERE Auth inter");
          gInterA_old = -10; 
          gInterB_old = -10; 
          gnbPassage = 0;
      }
	  
      gVitesse += 3;
	  analogWrite(PWM_PIN, gVitesse);
	  // Serial.print("gVitesse:"); Serial.println(gVitesse);
      if (gVitesse >= PWM_MAX ) {
          gEtat = RAPIDE;
          gDuree = DUREE_VITESSE_MAX;
		  gVitesse = PWM_MAX;
		  Serial.print("gVitesse:"); Serial.println(gVitesse);
		  analogWrite(PWM_PIN, PWM_MAX);
      }
      break;
      
    case RAPIDE: 
		if (gEtatOld != gEtat ) {
			Serial.println("RAPIDE");
			gEtatOld = gEtat;
			
					
		}
		//Serial.print("RAPIDE:");
		//Serial.println(gDuree);
		if (gDuree-- <= 0 ) {
			gEtat = DESCELERE;
		}
		break;
      
    case DESCELERE: 
		if (gEtatOld != gEtat ) 	{
			Serial.println("DESCELERE");
			gEtatOld = gEtat;
			gVitesse = PWM_MAX;
			
		}
		
		gVitesse -= 3;
		analogWrite(PWM_PIN, gVitesse);
		Serial.print("gVitesse:"); Serial.println(gVitesse);
		if (gVitesse <= PWM_MIN ) {
			gEtat = RALENTI;
			gDuree = DUREE_VITESSE_MIN;
			gVitesse = PWM_MAX;
		}
    break;
    case RALENTI: 
      if (gEtatOld != gEtat ) 
      {
          Serial.println("RALENTI");
          gEtatOld = gEtat;
      }
	  gVitesse = PWM_MIN;
      analogWrite(PWM_PIN, PWM_MIN);
      break;
      
	  
	case RECHERCHE: 
		if (gEtatOld != gEtat ) 
		{
			Serial.println("RECHERCHE");
			gEtatOld = gEtat;
			gVitesse = PWM_RECHERCHE;
			Serial.print("gVitesse:"); Serial.println(gVitesse);
			analogWrite(PWM_PIN, gVitesse);
		}
		
		Serial.print("RECHERCHE:");
		Serial.println(gVitesse);
      break;
      
    default: 
      gEtat = RALENTI ;
      break;
    }*/
}


/******************************************************/
/*
void desceleration_fin(E_ETAT next, int taux)
{
	// il faut faire une desceleration rapide
		 //Serial.println("desceleration_fin");
		 if (gEtatOld != gEtat ) {
			 Serial.println("Arrive en garre.");
			 gEtatOld = gEtat;
		 }
		 gVitesse-=taux;
		 analogWrite(PWM_PIN, gVitesse);
		 //Serial.print("gVitesse:"); Serial.println(gVitesse);
		 if ( gVitesse <= PWM_MIN_STOP)
		 {
			 analogWrite(PWM_PIN, PWM_STOP);
			 gEtat = next;
		 }
}
*/

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
