// Librerie utili e variabili di ambiente
#include <Stepper.h>
#define pECHO 2 
#define pTRIG 3
#define MAPPATURA_GRADI 2048
#define STEP_PER_RIVOLUZIONE 2048

// Dichiaro l'oggetto stepper
Stepper myStepper(STEP_PER_RIVOLUZIONE, 11, 9, 10, 8);

// Variabili globali utili
int flag = 0;      // indica se la rotazione è oraria o anti-oraria
int count = 0;    // contiene il numero delle step effettuate nella rotazione corrente
long duration;    // contiene la durata dall'invio del segnale al suo ritorno
int distance;    // contiene la distanza dal primo ostacolo incontrato

// Funzione di setup dell'arduino
void setup() {
  pinMode(pTRIG, OUTPUT);
  pinMode(pECHO, INPUT);
  Serial.begin(9600);
  myStepper.setSpeed(5);
  // inizializzo il messaggio di tipo sonar
}

// La funzione calcola la distanza dal primo ostacolo incontrato
float calculate_distance(int repetitions){
  // repetitions è il numero di volte per il quale va calcolata la distanza e poi fatta la media
  int average_distance = 0;
  for(int i = 0; i < repetitions; i++){
    duration = pulseIn(pECHO, HIGH);
    distance = duration * 0.034 / 2;
    float tuned_distance = distance* 1.10; // dai test è emerso che il sensore sottostima la distanza di circa un 10%
    average_distance = average_distance + tuned_distance;
  }
  average_distance = average_distance / repetitions;
  return average_distance;
}

// La funzione azzera i triggers liberando i pin dai dati ricevuti in precedenza
void azzera_triggers(){
  // Variabili per il delay
  int first_delay = 1;
  int second_delay = 5;
  // Setta il pin di trigger a LOW in modo tale da pulire il dato ricevuto in input
  digitalWrite(pTRIG, LOW);
  delayMicroseconds(first_delay);
  // Setta il pin di trigger ad HIGH in modo tale da far attendere il dato da ricevere input
  digitalWrite(pTRIG, HIGH);
  delayMicroseconds(second_delay);   
  // risettto il ping del trigger a low
  digitalWrite(pTRIG, LOW);
}
// La funzione fa ruotare lo stepper e calcola la distanza dal primo ostacolo incontrato
void move_and_calculate_distance(int gradi){
  int x_value= -1;
  int y_value= -1;
  
  // muove lo stepper di un grado
  myStepper.step(gradi);
  float  angle = (float) (count*6.2831)/2048.0; //calcolo angolo del sonar in radianti
  azzera_triggers();
  distance = calculate_distance(1);   
  // Stampa la distanza sulla console
  if (gradi > 0){
    x_value = distance*cos(angle);        //decompongo la misura su x and y 
    y_value = distance* sin(angle);       //decompongo la misura su x and y 
  }
  else {
     x_value = distance*cos(6.2831 -angle);        //decompongo la misura su x and y 
     y_value = distance* sin(6.2831 -angle);       //decompongo la misura su x and y 
  }
  
Serial.print(x_value);
Serial.print(",");
Serial.print(y_value);
Serial.print("\n");
  count = count + 1;
  // se il numero di step ha raggiunto il numero necessario per terminare la rivoluzione 
  // azzera il conteggio e imposta la flag ad 1 per per invertire la mozione allo stepper
  if(count > MAPPATURA_GRADI){
    flag = ( flag == 0 ) ? 1 : 0;
    count = 0;
  }
}

void loop() {
  // muovo di una sola step per volta
  int gradi = 1;
  // Se il flag è a zero allora ruoto in senso orario altrimenti al contrario
  // la funzione fa ruotare lo stepper e calcola i gradi
  flag == 0 ? move_and_calculate_distance(gradi) : move_and_calculate_distance(-gradi);
  // Applico un delay di 50ms una volta che viene completata una rivoluzione
  delay(50);
}
