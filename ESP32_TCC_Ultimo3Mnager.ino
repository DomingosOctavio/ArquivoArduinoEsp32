#include <NTPClient.h>
#include <ArduinoUniqueID.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <WiFiManager.h>
#include <DNSServer.h>
#include <Wifi.h>
#include <HTTPClient.h>
#include <Ethernet.h>
#include <WiFiUDP.h>//Biblioteca do UDP.
#include <WiFiUdp.h>

WiFiManager wifiManager;//Objeto de manipulação do wi-fi

//Serial.begin(115200);
//WiFiManager wifiManager;
//wifiManager.autoConnect("AutoConnectAP");
//Serial.println("Conectado");
  
//const char* ssid = "Claro local";
//const char* password = "RJ150884";
const char* host = "www.projetopainelsolar.tech";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

EthernetClient client;


float tensao;
float corrente;
float leituraLDR;
String horario = "";
String ip_atual;
String Unique_Id = "";
String formattedDate;
String data_atual;
int valorpot = 0; //Armazena valor lido do LDR, entre 0 e 1023 

 //tensao
float tensaoEntrada = 0.0; //VARIÁVEL PARA ARMAZENAR O VALOR DE TENSÃO DE ENTRADA DO SENSOR
float tensaoMedida = 0.0; //VARIÁVEL PARA ARMAZENAR O VALOR DA TENSÃO MEDIDA PELO SENSOR
float valorR1 = 30000.0; //VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO
float valorR2 = 7500.0; // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO
int leituraSensor = 0; //VARIÁVEL PARA ARMAZENAR A LEITURA DO PINO ANALÓGICO

WiFiServer server(80);

const int pinoSensorLDR = 35; //PINO DIGITAL UTILIZADO PELA SAÍDA DO SENSOR LDR
const int pinoSensorTensao = 34; //PINO DIGITAL UTILIZADO PELA SAÍDA DO SENSOR TENSAO
const int pinoSensorCorrente = 33;

void setup() //------------------------------------SETUP-------------------------------------
{
   WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    // it is a good practice to make sure your code sets wifi mode how you want it.
  Serial.begin(115200);


bool res;
  WiFiManager wifiManager;
   res = wifiManager.autoConnect("Claro local2","claro1234");
  Serial.println("Conectado");

   if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }
  //pino sensor luminosidade
  pinMode (pinoSensorLDR, INPUT); //DEFINE O PINO COMO ENTRADA
  //pino sensor  TENSAO
  pinMode(pinoSensorTensao, INPUT); //DEFINE O PINO COMO ENTRADA

  Serial.println();
  Serial.print("Conectando-se a ");
  //Serial.println(ssid);
  //WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectada.");
  Serial.println("Endereço de IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  // Initialize a NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(-10800);

  GravarLogin(UniqueId(), WiFi.localIP().toString().c_str());
}

void loop() //------------------------------------LOOP-------------------------------------
{  
 // Serial.println(RetornarHorario());
  Serial.println(LerTENSAOSensor());
  Serial.println(LerCORRENTESensor());
  Serial.println(LerLDRSensor());
  
  EnviarJson(LerTENSAOSensor(), LerCORRENTESensor(), LerLDRSensor()); // Enviar Json Serviço
  GravarHorario(LerTENSAOSensor(), LerCORRENTESensor(), LerLDRSensor(), RetornarData(),UniqueId()); // Gravar Hora Tabela

    Serial.println(WiFi.localIP());
 }
  
//----------------------------------------------------------------------------------------------------------------------EnviarJson

void EnviarJson(float tensao, float corrente, float luminosidade)
{
  delay(500);
  //Serial.print(F("Sending: "));
  StaticJsonDocument<1000> doc2;
   
   WiFiClient client = server.available();
   if (client) {
    
   // Write response headers

   doc2["tensao"] = tensao;
   doc2["corrente"] = corrente;
   doc2["luminosidade"] = luminosidade;
   doc2["horario"] = horario;
   doc2["data"] = data_atual;
  
   //Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println(F("Content-Type: application/json"));
            client.println(measureJsonPretty(doc2));
            client.println();
            serializeJsonPretty(doc2, client);
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
//----------------------------------------------------------------------------------------------------------------------RetornarHorario
String RetornarHorario()
{
   while(!timeClient.update()) 
   {
   timeClient.forceUpdate();
   }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
   formattedDate = timeClient.getFormattedDate();
  // Extract date
  int splitT = formattedDate.indexOf("T");
  // Extract time
  horario = formattedDate.substring(splitT+1, formattedDate.length()-1);

  return horario;
}

//----------------------------------------------------------------------------------------------------------------------RetornarData
String RetornarData()
{
  while(!timeClient.update()) 
   {
   timeClient.forceUpdate();
   }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
   formattedDate = timeClient.getFormattedDate();

  // Extract date
  int splitT = formattedDate.indexOf("T");
  data_atual = formattedDate.substring(0, splitT);

  return data_atual;
}
//----------------------------------------------------------------------------------------------------------------------UniqueId
String UniqueId()
{

  String UniqueId = "";
  for(size_t i = 0; i < 5; i++)
  {
      UniqueId = UniqueId + String(UniqueID8[i], HEX);
  }
  return UniqueId;
}
//----------------------------------------------------------------------------------------------------------------------GravarLogin
void GravarLogin(String uniqueId, String ip)
{
  const int httpPort = 80;
  WiFiClient client3;

  if (!client3.connect(host, httpPort)) {
    Serial.println("connection failed");
    delay(1000);
    return;
  }

  String url = "/login.php?";

  url += "uniqueId=";
  url +=  uniqueId;
  url += "&ip=";
  url +=  ip;

  Serial.println("receiving from remote server");
  Serial.println(url);
  
  client3.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");


  unsigned long timeout = millis();
  while (client3.available() == 0)
  {
    if (millis() - timeout > 5000 )
    {
      Serial.println (">>> Client Timeout |");
      client3.stop();
      return;
    }
  }

  while (client3.available())
  {
    String line = client3.readStringUntil('\r');
    Serial.print(line);

    if (line.indexOf("salvo_com_sucesso") != -1)
    {
      Serial.println();
      Serial.println("Atenção, dados salvos na tabela login!");
    }
  }
  // Close the connection
  Serial.println();
  Serial.println("Conexao Fechada");
  client3.stop();
  //fim_mysql
}

//----------------------------------------------------------------------------------------------------------------------GRAVAR HORARIO

void GravarHorario(float tensao, float corrente, float luminosidade, String data_atual, String UniqueId)
{

  if (horario.compareTo("06:00:00") == 0)
  {
    String coluna = "6h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("07:00:00") == 0)
  {
    String coluna = "7h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("08:00:00") == 0)
  {
    String coluna = "8h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("09:00:00") == 0)
  {
    String coluna = "9h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("10:00:00") == 0)
  {
    String coluna = "10h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("11:00:00") == 0)
  {
    String coluna = "11h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna); 
  }
  else if (horario.compareTo("12:00:00") == 0)
  {
    String coluna = "12h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("13:00:00") == 0)
  {
    String coluna = "13h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual,UniqueId, coluna);
  }
  else if (horario.compareTo("14:00:00") == 0)
  {
    String coluna = "14h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("15:00:00") == 0)
  {
    String coluna = "15h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("16:00:00") == 0)
  {
    String coluna = "16h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna);
  }
  else if (horario.compareTo("17:00:00") == 0)
  {
    String coluna = "17h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual,UniqueId, coluna);
  }
  else if (horario.compareTo("18:00:00") == 0)
  {
    String coluna = "18h";
    GravarDadosMySql(tensao, corrente, luminosidade, data_atual, UniqueId, coluna); 
  }
 
}

//----------------------------------------------------------------------------------------------------------------------GravarDadosMySql

void GravarDadosMySql(float tensao, float corrente, float luminosidade, String data_atual, String token, String coluna)
{ 
  delay(1000);
  const int httpPort = 80;
  WiFiClient client2;

  if (!client2.connect(host, httpPort)) {
    Serial.println("connection failed");
    delay(1000);
    return;
  }

  String url = "/salvar.php?";

  url += "tensao=";
  url +=  tensao;

  url += "&corrente=";
  url +=  corrente;

  url += "&luminosidade=";
  url +=  luminosidade;

  url += "&horario=";
  url +=  horario;

  url += "&data_atual=";
  url +=  data_atual;

  url += "&token=";
  url +=  token;

  url += "&varHoraColuna=";
  url +=  coluna;

  
  Serial.println("receiving from remote server");
  Serial.println(url);


  client2.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");


  unsigned long timeout = millis();
  while (client2.available() == 0)
  {
    if (millis() - timeout > 5000 )
    {
      Serial.println (">>> Client Timeout |");
      client2.stop();
      return;
    }
  }
  while (client2.available())
  {
    String line = client2.readStringUntil('\r');
    Serial.print(line);

    if (line.indexOf("salvo_com_sucesso") != -1 || line.indexOf(" "))
    {
      Serial.println();
      Serial.println("Atenção, dados salvos com sucesso!");
    }
  }
  // Close the connection
  Serial.println();
  Serial.println("Conexao Fechada");
  client2.stop();
  
  delay(1000);
 }

 //SENSORES
 //-------------------------------------------------------------------------------------------SENSOR LDR
float LerLDRSensor()
{
  valorpot = analogRead(pinoSensorLDR);  
  leituraLDR = map(valorpot, 0, 4095, 100, 0); 
  return leituraLDR;
}
 //-------------------------------------------------------------------------------------------SENSOR TENSAO
 
 float LerTENSAOSensor()
{
  // SENSOR TENSAO
   leituraSensor = analogRead(pinoSensorTensao); //FAZ A LEITURA DO PINO ANALÓGICO E ARMAZENA NA VARIÁVEL O VALOR LIDO
   tensaoEntrada = (leituraSensor * 5.0) / 4096.0; //VARIÁVEL RECEBE O RESULTADO DO CÁLCULO
   tensaoMedida = tensaoEntrada / (valorR2/(valorR1+valorR2)); //VARIÁVEL RECEBE O VALOR DE TENSÃO DC MEDIDA PELO SENSOR

   return tensaoMedida;
}

 //-------------------------------------------------------------------------------------------SENSOR CORRENTE
 float LerCORRENTESensor()
{
    unsigned int x=0;
    float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;
    
      for (int x = 0; x < 150; x++){ //Get 150 samples
      AcsValue = analogRead(pinoSensorCorrente);     //Read current sensor values   
      Samples = Samples + AcsValue;  //Add samples together
      delay (3); // let ADC settle before next sample 3ms
    }
    AvgAcs=Samples/150.0;//Taking Average of Samples
    AcsValueF = (2.5 - (AvgAcs * (5.0 / 4096.0)) )/0.185;
    return AcsValueF - 1.5;
}

 //-------------------------------------------------------------------------------------------MANAGER
