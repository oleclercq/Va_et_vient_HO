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
#define PWM_MIN (120)	// Vitesse Minimum, jusqu'a attendre la butée (inter)
#define PWM_MAX (255)	// Vitesse MAX
#define PWM_STOP (0)
#define DUREE_EN_GARE	(100)	// 100 * 50ms = 5000ms = 5s;

#define DUREE_VITESSE_MIN (100) // 50 passa ge cadencé ) 50ms  =50x50 = 2.5s
#define DUREE_VITESSE_MAX (120) // 50 passa ge cadencé ) 50ms  =50x50 = 2.5s

#define NB_PASS_DETECT	(100) // temps avant de détecter la gare d'arrivée




static int gDirection = 0;

//Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

typedef enum { DETECT_A,DETECT_B,DETEC_C,DETEC_D, GARE_A,GARE_B,ACCELERE,RAPIDE,DESCELERE,RALENTI,RECHERCHE} E_ETAT;
volatile E_ETAT gEtat = RECHERCHE;
volatile E_ETAT gEtatOld = RAPIDE;
volatile int gSens = 0;

volatile int gnbPassage = 0;
volatile int gDuree = DUREE_EN_GARE;

volatile int gInterA_old = -1;
volatile int gInterB_old = -1;
volatile int gInterC_old = -1;
volatile int gInterD_old = -1;

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
void init_sensor();

// Prototypage.
void desceleration_fin(E_ETAT);


/* ************************************************************************ */
/* ************************************************************************ */
void setup ()
{

  //myservo.attach(4);  // attaches the servo on pin 9 to the servo object
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(DIRECTION_PIN, HIGH); 
  analogWrite(PWM_PIN, 100);
  Serial.print("gVitesse:"); Serial.println(gVitesse);



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

void loop()
{
  gInter_A = digitalRead(CPT_A);
  gInter_B = digitalRead(CPT_B);
  gInter_C = digitalRead(CPT_C);
  gInter_D = digitalRead(CPT_D);

  /*Serial.print("A=");
  Serial.println(gInter_A);
  
  Serial.print("B=");
  Serial.println(gInter_B);*/


    if (gInter_A==LOW)
    {
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
		}
	}
		
	if (gInter_D == LOW)
    {
		if (gPosD_old != gPosD)	{
			Serial.println("***** D *****");
			init_sensor();
			gPosD_old = gPosD;
		}
	}
}

/* ************************************************************************ */
/* */
/* ************************************************************************ */
void init_sensor()
{
	gPosA_old = -1 ;
	gPosB_old = -1 ;
	gPosC_old = -1 ;
	gPosD_old = -1 ;
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

/******************************************************/
/* FONCTION APPELLEE TOUTES LES 50ms                  */
void action()
{
  switch(gEtat)
  {
	case DETECT_A : desceleration_fin(GARE_A); break;
	case DETECT_B : desceleration_fin(GARE_B); break;
	  
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
	  
      analogWrite(PWM_PIN, gVitesse);
	  gVitesse += 3;
	  Serial.print("gVitesse:"); Serial.println(gVitesse);
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
		//Serial.print("DESCELERE:");
		//Serial.println(gVitesse);
		Serial.print("gVitesse:"); Serial.println(gVitesse);
		analogWrite(PWM_PIN, gVitesse);
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
    }
}


/******************************************************/

void desceleration_fin(E_ETAT next)
{
	// il faut faire une desceleration rapide
		 if (gEtatOld != gEtat ) {
			 Serial.println("Arrive en garre A");
			 gEtatOld = gEtat;
		 }
		 gVitesse-=2;
		 analogWrite(PWM_PIN, gVitesse);
		 Serial.print("gVitesse:"); Serial.println(gVitesse);
		 if ( gVitesse <= PWM_MIN_STOP)
		 {
			 analogWrite(PWM_PIN, PWM_STOP);
			 gEtat = next;
		 }
}


/******************************************************/
void enGare()
{
	static int gDureeSeconde=20;
		
      
      //    Premier traitement
      if (gEtatOld != gEtat ) 
      {
		  analogWrite(PWM_PIN, PWM_STOP);
          if (gEtat == GARE_A){
            Serial.println("GARE_A*****");
            analogWrite(PWM_PIN, PWM_STOP);
			digitalWrite(DIRECTION_PIN, LOW); 
			gDuree = 2;
          }

          if (gEtat == GARE_B){
            Serial.println("GARE_B****");
			digitalWrite(DIRECTION_PIN, HIGH); 
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
      
}
