// Librerie utili e variabili di ambiente
#include <Stepper.h>
#define pECHO 2 
#define pTRIG 3
#define MAPPATURA_GRADI 360
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
  myStepper.setSpeed(10);
}

// La funzione calcola la distanza dal primo ostacolo incontrato
int calculate_distance(){
  // num è il numero di volte per il quale va calcolata la distanza e poi fatta la media
  int num = 1;
  int average_distance = 0;
  for(int i = 0; i < num; i++){
    duration = pulseIn(pECHO, HIGH);
    distance = duration * 0.034 / 2;
    average_distance = average_distance + distance;
  }
  average_distance = average_distance / num;
  
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
  
  // muove lo stepper di un grado
  myStepper.step(gradi);
  
  // azzera i triggers
  azzera_triggers();
    
  // Calcolo la distanza
  distance = calculate_distance(); 
    
  // Stampa la distanza sulla console
  Serial.println("Distance: " + (String)distance + " cm");
    
  // conto una step effettuata
  count = count + 1;
    
  // se il numero di step ha raggiunto il numero necessario per terminare la rivoluzione 
  // azzera il conteggio e imposta la flag ad 1 per per invertire la mozione allo stepper
  if(count == MAPPATURA_GRADI){
    flag = ( flag == 0 ) ? 1 : 0;
    count = 0;
  }
}

void loop() {
  // muovo di una sola step per volta
  float gradi = 1;
  
  // mappo i gradi facendo in modo che la rotazione sia composta da solo 360 steps
  gradi = map(gradi, 0, MAPPATURA_GRADI, 0, STEP_PER_RIVOLUZIONE);
  
  // Se il flag è a zero allora ruoto in senso orario altrimenti al contrario
  // la funzione fa ruotare lo stepper e calcola i gradi
  flag == 0 ? move_and_calculate_distance(gradi) : move_and_calculate_distance(-gradi);
  
  // Applico un delay di 50ms una volta che viene completata una rivoluzione
  delay(50);
  
}
