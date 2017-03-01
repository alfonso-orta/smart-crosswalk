#include <PubSubClient.h>
#include <ESP8266WiFi.h>

//ESTADOS DEL SISTEMA
#define NADA 0
#define PEATON 1
#define COCHE 2

//DEFINICION PINES DE ENTRADA/SALIDA
#define ECHOPIN D7
#define TRIGPIN D6
#define LDR A0
#define OUT_LED D4


//CONFIGURACION ACCESO A LA RED WIFI
const char* ssid = "Vodafone6717";
const char* password = "MMCDQCPIYODWPL";

//CONFIGURACION SERVIDOR MQTT Y TOPIC
#define MQTT_SERVER "192.168.0.5"
char* lightTopic = "PASOPEATONES";

//VARIABLES ESTADOS
 int ESTADO_OTHER=NADA;
 double instante_ESTADO_OTHER;
 int last_state=NADA;

 ///////////////////////////////////////////////////////////////////////////
 //////////////////////VARIABLES SISTEMA DIFUSO/////////////////////////////
 //////////////////////////////////////////////////////////////////////////
 double x0K1[2];
 double x1K1[2];
 double x2K1[2];
 double x3K1[2];
 double x0K2[2];
 double x1K2[2];
 double x2K2[2];
 double x3K2[2];
 double x0K3[2];
 double x1K3[2];
 double x2K3[2];
 double x3K3[2];
 int K1[8];
 int K2[8];
 int K3[8];
 int C1[8];
 double H[8];
 ///////////////////////////////////////////////////////////////////////////
 //////////////////////VARIABLES OBTENCION LUZ/////////////////////////////
 //////////////////////////////////////////////////////////////////////////

 int medidas_luz[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  //valores de ejemplo
 int n_medidas_luz=10;
 int real_luz[4] = {0, 323, 626, 934};                  //valores de ejemplo
 int medida_luz[4] = {8, 852, 867, 913};                //valores de ejemplo
 int n_lookup_luz = 4;
 int min_medida_luz = 8;                                //valores de ejemplo
 int max_medida_luz = 913;                              //valores de ejemplo
 int min_real_luz = 0;                                //valores de ejemplo
 int max_real_luz = 934;                              //valores de ejemplo
 int old_index_luz = 0;
 int tolerancia_luz = 2;
 int out_luz=0;
 int UMBRAL_LUZ=600;
 double instante_no_luz;
 double MAX_NO_LUZ = 5000;

 ///////////////////////////////////////////////////////////////////////////
 ////////////////////VARIABLES OBTENCION DISTANCIA//////////////////////////
 //////////////////////////////////////////////////////////////////////////
 int medidas_distancia[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  //valores de ejemplo
 int n_medidas_distancia=10;
 int min_medida_distancia = 3;                                //valores de ejemplo
 int max_medida_distancia = 13;                              //valores de ejemplo
 int min_real_distancia = 3;                                //valores de ejemplo
 int max_real_distancia = 13;                              //valores de ejemplo
 int old_index_distancia = 0;
 int tolerancia_distancia = 2;
 int out_distancia=0;

 ///////////////////////////////////////////////////////////////////////////
 ///////////////////////VARIABLES TIEMPO PEATON////////////////////////////
 //////////////////////////////////////////////////////////////////////////
 double instante_peaton;
 double MAX_PEATON = 4000;
 double instante_evento;
 double MAX_ENVIO_EVENTO=2000;
 double MAX_LED_ENCENDIDO=4000;


 ///////////////////////////////////////////////////////////////////////////
 ///////////////////////FUNCIONES Y OBJETOS ESP8266/////////////////////////
 //////////////////////////////////////////////////////////////////////////

//Definicion de la funcion callback del cliente MQTT
void callback(char* topic, byte* payload, unsigned int length);

//Funcion que convierte una direccion mac a string
String macToStr(const uint8_t* mac) {
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5) {
      result += ':';
    }
  }
  return result;
}
//VARAIABLES CONEXIÓN A LA RED Y AL SERVIDOR MQTT
WiFiClient wifiClient;
PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);

void callback(char* topic, byte* payload, unsigned int length) {


  if (payload[0] == 'P') {//Si el evento es peaton, otro nodo terminal ha detectado a un peaton
    ESTADO_OTHER=PEATON;
    instante_ESTADO_OTHER=millis();
    Serial.println("Evento peaton recibido");
  }
}

//FUNCION DE CONEXION A LA RED
void reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
  }
  if (WiFi.status() == WL_CONNECTED) {
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      String clientName;
      clientName += "esp8266-";
      uint8_t mac[6];
      WiFi.macAddress(mac);
      clientName += macToStr(mac);
      if (client.connect((char*) clientName.c_str())) {
        Serial.print("\tMTQQ Connected");
        client.subscribe(lightTopic);
        }
        else {
          Serial.println("\tFailed.");
          abort();
        }
      }
    }
  }


//FUNCION QUE MAPEA LA VARIABLE DE ENTRADA CON EL RANGO DE ENTRADA EN EL RANGO DE SALIDA
double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//FUNCION QUE EJECUTA EL SISTEMA DIFUSO
int fuzzysystem(double E1, double E2, double E3) {

    //Obtencion del grado de emparejamiento
    double corte;
    for (int i = 0; i<8; i++) {
      H[i] = 2147483648; //{IniciKlizo K un mÂ¡nimo muy grKnde}
      corte = 0;
      if ((E1 >= x0K1[K1[i]]) && (E1<x1K1[K1[i]])) { corte = (E1 - x0K1[K1[i]]) / (x1K1[K1[i]] - x0K1[K1[i]]); }
      if ((E1 >= x1K1[K1[i]]) && (E1<x2K1[K1[i]])) { corte = 1; }
      if ((E1 >= x2K1[K1[i]]) && (E1<=x3K1[K1[i]])) { corte = (E1 - x3K1[K1[i]]) / (x2K1[K1[i]] - x3K1[K1[i]]); }
      if (corte<H[i]) { H[i] = corte; }// {MÂ¡nimo de los Kntecedentes hKstK KhorK}
      corte = 0;
      if ((E2 >= x0K2[K2[i]]) && (E2<x1K2[K2[i]])) { corte = (E2 - x0K2[K2[i]]) / (x1K2[K2[i]] - x0K2[K2[i]]); }
      if ((E2 >= x1K2[K2[i]]) && (E2<x2K2[K2[i]])) { corte = 1; }
      if ((E2 >= x2K2[K2[i]]) && (E2<=x3K2[K2[i]])) { corte = (E2 - x3K2[K2[i]]) / (x2K2[K2[i]] - x3K2[K2[i]]); }
      if (corte<H[i]) { H[i] = corte; } //{MÂ¡nimo de los Kntecedentes hKstK KhorK}
      corte = 0;
      if ((E3 >= x0K3[K3[i]]) && (E3<x1K3[K3[i]])) { corte = (E3 - x0K3[K3[i]]) / (x1K3[K3[i]] - x0K3[K3[i]]); }
      if ((E3 >= x1K3[K3[i]]) && (E3<x2K3[K3[i]])) { corte = 1; }
      if ((E3 >= x2K3[K3[i]]) && (E3<=x3K3[K3[i]])) { corte = (E3 - x3K3[K3[i]]) / (x2K3[K3[i]] - x3K3[K3[i]]); }
      if (corte<H[i]) { H[i] = corte; }// {MÂ¡nimo de los Kntecedentes hKstK KhorK}
    }

    //Obtencion del grado de emparejamiento acumulado por etiquetas de salida
    double sumHM[3];
    for (int i = 0; i<3; i++) {
      sumHM[i] = 0;
    }

    for (int i = 0; i<8; i++) {
      sumHM[C1[i]] = sumHM[C1[i]] + H[i];
    }

    //Obtencion de la etiqueta de salida con mayor emparejamiento
    int resultado = NADA;
    double value = 0;
      for (int i = 0; i < 3; i++) {
        if (sumHM[i] > value) {
          resultado = i;
          value = sumHM[i];
        }
      }

    return resultado;
}

//FUNCION QUE RESTRINGE EL RANGO DE UNA VARIABLE
double constraintdouble(double x, double in_min, double in_max){
  if(x<in_min){
    return in_min;
  }else if(x>in_max){
    return in_max;
  }else{
    return x;
  }
}

//FUNCION QUE OBTIENE LA MEDIDA DE LA LUZ A PARTIR DE LA TABLA LOOK UP
int look_up_luz(){
    // Obtencion de la medida
    int luz_in = analogRead(LDR);
    int luz_out=0;
    if (luz_in < min_medida_luz) {
      //Si la medida es inferior al limite inferior
      luz_out = real_luz[0];
    } else if (luz_in > max_medida_luz) {
      //Si la medida es superior al limite superior
      luz_out = real_luz[n_lookup_luz - 1];
    } else {
      //Obtencion del valor
      int i = 1;
      while (i < (n_lookup_luz - 1) && medida_luz[i] < luz_in) {
        i++;
      }
      luz_out = map(luz_in, medida_luz[i - 1], medida_luz[i], real_luz[i - 1], real_luz[i]);
    }
    return luz_out;
}

//FUNCION QUE OBTIENE LA MEDIA DE LAS MEDIDAS ALMACENADAS EN EL CASO DE LA LUZ
int mean_medidas_luz() {
  float sum = 0;

  for (int i = 0; i < n_medidas_luz; i++) {
    sum = sum + medidas_luz[i];
  }
  return round(sum / n_medidas_luz);
}

//FUNCION QUE OBTIENE LA MEDIA DE LAS MEDIDAS ALMACENADAS EN EL CASO DE LA DISTANCIA
int mean_medidas_distancia() {
  float sum = 0;

  for (int i = 0; i < n_medidas_distancia; i++) {
    sum = sum + medidas_distancia[i];
  }
  return round(sum / n_medidas_distancia);
}

//FUNCION QUE OBTIENE LA MEDIDA DE LA DISTANCIA
int distancia() {
  long tiempo;
  digitalWrite(TRIGPIN, LOW); /* Por cuestiÓn de estabilizaciÓn del sensor*/
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH); /* envÃ­o del pulso ultrasÓnico*/
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  tiempo = pulseIn(ECHOPIN, HIGH); /* FunciÓn para medir la longitud del pulso entrante. Mide el tiempo que transcurrido entre el envÃ­o
  del pulso ultrasÃ³nico y cuando el sensor recibe el rebote, es decir: desde que el pin 12 empieza a recibir el rebote, HIGH, hasta que
  deja de hacerlo, LOW, la longitud del pulso entrante*/
  int distancia = int(tiempo/58.2); /*fÓrmula para calcular la distancia obteniendo un valor entero*/
  /*MonitorizaciÃ³n en centÃ­metros por el monitor serial*/
  if (distancia < min_medida_distancia) {
    distancia = min_medida_distancia;
  } else if (distancia > max_medida_distancia) {
    distancia = max_medida_distancia;
  }

  return distancia;
}

//FUNCION DE INICIALIZACION PRINCIPAL
void setup() {
  //Inicializacion puerto serial
  Serial.begin(115200); delay(100);
  //Conexion red wifi
  WiFi.begin(ssid, password);
  reconnect();
  delay(2000);
  ///////////////////////////////////////////////////////////////////////////
  ///////////////INICIALIZACION BáSICA SISTEMA DIFUSO////////////////////////
  //////////////////////////////////////////////////////////////////////////

  x0K1[0] = 0;       x1K1[0] = 0;       x2K1[0] = 0.3333;  x3K1[0] = 0.6667;
  x0K1[1] = 0.3333;  x1K1[1] = 0.6667;  x2K1[1] = 1;       x3K1[1] = 1;
  x0K2[0] = 0;       x1K2[0] = 0;       x2K2[0] = 0.3333;  x3K2[0] = 0.6667;
  x0K2[1] = 0.3333;  x1K2[1] = 0.6667;  x2K2[1] = 1;       x3K2[1] = 1;
  x0K3[0] = 0;       x1K3[0] = 0;       x2K3[0] = 0.3333;  x3K3[0] = 0.6667;
  x0K3[1] = 0.3333;  x1K3[1] = 0.6667;  x2K3[1] = 1;       x3K3[1] = 1;
  K1[0] = 0; K1[1] = 1; K1[2] = 1; K1[3] = 0; K1[4] = 0; K1[5] = 0; K1[6] = 1; K1[7] = 1;
  K2[0] = 0; K2[1] = 0; K2[2] = 1; K2[3] = 1; K2[4] = 1; K2[5] = 0; K2[6] = 0; K2[7] = 1;
  K3[0] = 0; K3[1] = 0; K3[2] = 0; K3[3] = 0; K3[4] = 1; K3[5] = 1; K3[6] = 1; K3[7] = 1;
  C1[0] = 2; C1[1] = 2; C1[2] = 2; C1[3] = 2; C1[4] = 2; C1[5] = 2; C1[6] = 1; C1[7] = 0;

  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////CALIBRACION LUZ (LDR)///////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  pinMode(LDR, INPUT);
  for (int i = 0; i < n_medidas_luz; i++) {
    medidas_luz[i] = 0;
  }

  //Obtencion tabla lookup de la variable luz
  Serial.println("CALIBRACION DEL SENSOR");
  char continuar = 'n';
  while (continuar == 'n') {
    Serial.println("A continuacion se deberan introducir 4 valores de menor a mayor del sensor de iluminacion");
    int i = 0;
    while (i < n_lookup_luz) {
      Serial.print("Introduzca el valor real actual ");
      Serial.println((i + 1));
      while (!Serial.available()) {}
      real_luz[i] = Serial.parseInt();
      medida_luz[i] = analogRead(LDR);
      i++;
    }

    Serial.println("Tabla Lockup");
    Serial.println("Medida, Real");
    for (int i = 0; i < n_lookup_luz; i++) {
      Serial.print(medida_luz[i]);
      Serial.print(",");
      Serial.println(real_luz[i]);
    }

    do {
      Serial.println("Â¿Los valores son correctos? (y/n)");
      while (!Serial.available()) {
      }
      continuar = (char)Serial.read();
    } while (continuar != 'y' && continuar != 'n');
  }

  //Ajuste de los rangos de la medida luz
  min_medida_luz = medida_luz[0];
  max_medida_luz = medida_luz[n_lookup_luz - 1];
  min_real_luz = real_luz[0];
  max_real_luz = real_luz[n_lookup_luz - 1];
  tolerancia_luz=2; //tolerancia_luz -> obtenido realizando estudio previo
  UMBRAL_LUZ = (real_luz[1]+real_luz[2])/2;
  instante_no_luz=millis();

  //Configuracion y adapta de las etiquetas Fuzzy segun la calibración
  x2K3[0] = mapdouble(real_luz[1],min_real_luz,max_real_luz,0,1);
  x3K3[0] = mapdouble(real_luz[2],min_real_luz,max_real_luz,0,1);
  x0K3[1] =  mapdouble(real_luz[1],min_real_luz,max_real_luz,0,1);
  x1K3[1] = mapdouble(real_luz[2],min_real_luz,max_real_luz,0,1);

  //Rellena Medidas Lookup_luz
 Serial.println("Tabla Medidas Luz Rellena");
  for (int i = 0; i < n_medidas_luz; i++) {
    medidas_luz[i] = look_up_luz();
    while (medidas_luz[i] < min_real_luz || medidas_luz[i] > max_real_luz) {
      medidas_luz[i] = look_up_luz();
    }
  }
  ///////////////////////////////////////////////////////////////////////////
  //////////////////CALIBRACION DISTANCIA (HC-SR04)/////////////////////////
  //////////////////////////////////////////////////////////////////////////
  //No es necesario calibracion, solo inicializacion
  pinMode(TRIGPIN, OUTPUT); /*Activación del pin sw salida: para el pulso ultrasónico*/
  pinMode(ECHOPIN, INPUT);

  for (int i = 0; i < n_medidas_distancia; i++) {
    medidas_distancia[i] = 0;
  }

  //Relleno Medidas Distancia
    Serial.println("Tabla Medidas Distancia Rellena");
  for (int i = 0; i < n_medidas_distancia; i++) {
    medidas_distancia[i] = distancia();//obtener distancia del sensor
    while (medidas_distancia[i] < min_real_distancia || medidas_distancia[i] > max_real_distancia) {
      medidas_distancia[i] = distancia();//obtener distancia del sensor);
    }
  }

  ///////////////////////////////////////////////////////////////////////////
  ////////////////////INICIALIZACION EVENTO PEATON///////////////////////////
  //////////////////////////////////////////////////////////////////////////
  instante_peaton=millis();
  pinMode(OUT_LED, OUTPUT);
  digitalWrite(OUT_LED,LOW);
  instante_evento=millis();
}

//FUNCION BUBLE PRINCIPAL
void loop() {
  //Comprobacion y conexion de la red
  if (!client.connected() && WiFi.status() == 3) {
      reconnect();
    }
    //Ejecucion bucle principal cliente MQTT
   client.loop();

  //////////////OBTENCION LUZ (0,1)//////////////

  int in_luz = look_up_luz();
  while (in_luz < min_real_luz || in_luz > max_real_luz) {
    in_luz = look_up_luz();
  }
  medidas_luz[old_index_luz] = in_luz;
  old_index_luz = (old_index_luz + 1) % n_medidas_luz;

  int result_mean_luz = mean_medidas_luz();

  if (in_luz > (result_mean_luz - tolerancia_luz) && in_luz < (result_mean_luz+ tolerancia_luz)) {
    out_luz = result_mean_luz;
  }

  ///////////OBTENCION DISTANCIA (0,1)///////////

   int in_distancia = distancia();//obtener distancia del sensor
  while (in_distancia < min_real_distancia || in_distancia > max_real_distancia) {
    in_distancia = distancia();//obtener distancia del sensor);
  }
  medidas_distancia[old_index_distancia] = in_distancia;
  old_index_distancia = (old_index_distancia + 1) % n_medidas_distancia;

  int result_mean_distancia = mean_medidas_distancia();

  if (in_distancia > (result_mean_distancia - tolerancia_distancia) && in_distancia < (result_mean_distancia + tolerancia_distancia)) {
    out_distancia = result_mean_distancia;
  }


  ////////////OBTENCION TIEMPO (0,1)////////////
  if(out_luz<UMBRAL_LUZ){
    instante_no_luz=millis();
  }
  double tiempo_no_luz=millis()-instante_no_luz;
  if(tiempo_no_luz>MAX_NO_LUZ){
    tiempo_no_luz=MAX_NO_LUZ;
  }
  //////////////PRE-CALCULOS FUZZY//////////////

  double E1=mapdouble(tiempo_no_luz,0,MAX_NO_LUZ,0,1);
  double E2=mapdouble(out_distancia,min_real_distancia,max_real_distancia,0,1);
  double E3=mapdouble(out_luz,min_real_luz,max_real_luz,0,1);

  ///////////////SISTEMA FUZZY///////////////

 // Serial.println(out_distancia);
  int ESTADO=fuzzysystem(E1,E2,E3);

  ////////////ACCIONES POST FUZZY////////////
  if(ESTADO!=last_state){
    last_state=ESTADO;
    Serial.println("Cambio de estado");
    if(ESTADO==NADA){
       Serial.println("NADA");

    }else if(ESTADO==PEATON){
       Serial.println("PEATON");

    }else if(ESTADO == COCHE){
       Serial.println("COCHE");
    }
  }

//////DETECCION DE EVENTO PEATON Y ENCENDIDO DEL LED//////
  double tiempo_evento=millis()-instante_evento;
  if(ESTADO==PEATON && tiempo_evento > MAX_ENVIO_EVENTO){
    //Emitir evento
    instante_evento=millis();
    Serial.println("ENVIO EVENTO");
    client.publish(lightTopic, "P");
  }

  if((ESTADO_OTHER==PEATON && ((millis()-instante_ESTADO_OTHER)<MAX_LED_ENCENDIDO))|| ESTADO==PEATON){
    //Encender led
    digitalWrite(OUT_LED, HIGH);
    instante_peaton=millis();
  }

  double tiempo_peaton=millis()-instante_peaton;
  if(tiempo_peaton>MAX_PEATON){
    //Apagar led
    digitalWrite(OUT_LED, LOW);
  }


}
