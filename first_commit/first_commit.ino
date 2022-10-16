// Librerie utili e variabili di ambiente
#include <Stepper.h>
#define pECHO 2 
#define pTRIG 3
#define MAPPATURA_GRADI 2048
#define STEP_PER_RIVOLUZIONE 2048

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud.h>
 
ros::NodeHandle  nh;                            // NodeHandle di radarino
sensor_msgs::PointCloud sonar_msg;              // le misure sono inviate a Rviz tramite messaggio nuvola di punti
ros::Publisher pub_sonar( "sonar", &sonar_msg); // message publisher
geometry_msgs::Point32 points_storage[1];       // per risparmiare memoria ogni messaggio contiene un singolo punto

char frameid[] ="map";                          // nome del frame id di Rviz
unsigned long publisher_timer;

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
  myStepper.setSpeed(5);
  // inizializzo il messaggio di tipo sonar
  sonar_msg.header.seq = 0;
  sonar_msg.header.frame_id =  frameid; 
  // inizializzo il primo messaggio sonar a zero e setto la lunghezza a 1 
  sonar_msg.points = points_storage;
  sonar_msg.points[0].x = 0;
  sonar_msg.points[0].y = 0;
  sonar_msg.points_length = 1;
  // inizializzo ros
  nh.initNode();
  nh.advertise(pub_sonar);  
}

// La funzione calcola la distanza dal primo ostacolo incontrato
float calculate_distance(){
  // num è il numero di volte per il quale va calcolata la distanza e poi fatta la media
  int num = 1;
  int average_distance = 0;
  for(int i = 0; i < num; i++){
    duration = pulseIn(pECHO, HIGH);
    distance = duration * 0.034 / 2;
    float tuned_distance = distance* 1.10; // dai test è emerso che il sensore sottostima la distanza di circa un 10%
    average_distance = average_distance + tuned_distance;
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
  float  angle = (float) (count*6.2831)/2048.0; //calcolo angolo del sonar in radianti
  //Serial.println(angle,5);
  // azzera i triggers
  azzera_triggers();
  // Calcolo la distanza
  distance = calculate_distance();   
  // Stampa la distanza sulla console
  if (gradi > 0){
    points_storage[0].x = distance*cos(angle);        //decompongo la misura su x and y 
    points_storage[0].y = distance* sin(angle);       //decompongo la misura su x and y 
  }
  else {
     points_storage[0].x = distance*cos(6.2831 -angle);        //decompongo la misura su x and y 
    points_storage[0].y = distance* sin(6.2831 -angle);       //decompongo la misura su x and y 
  }

  //salvo la misura e la publico
  sonar_msg.points[0].x = points_storage[0].x;    
  sonar_msg.points[0].y = points_storage[0].y;    
  sonar_msg.header.stamp = nh.now();
  pub_sonar.publish(&sonar_msg);  
  publisher_timer = millis();

  // conto una step effettuata
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
  nh.spinOnce(); // chiama il callback
  delay(5);
}
