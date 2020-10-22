//le programme va envoyer au serveur toutes les valeurs lues sur les broches arduino 2,3,4 et A2,A3,A4,A5,A6,A7
//A0,A1 reserve
//%MW102 : 2 %MW103 : 3 %MW104 : 4    -> digitalRead
//%MW112 : A2 %MW113 : A3 %MW114 : A4 %MW115 : A5 %MW116 : A6 %MW117 : A7 -> analogRead
// et lire sur le serveur les adresses dédiées pour les renvoyées sur les broches de sorties 5,6,7,8,9
//%MW125 : 5, %MW126 : 6   -> analogWrite
//%MW137 : 7, %MW138 : 8, %MW139 : 9, -> digitalWrite

/************************trame modbus/TCP*************************/
/*
[                         HEADER                              ][   PAYLOAD                              ]
|MSB counter trame|LSB counteur trame| 0 | 0 | 0 | lenght | 1 |****write or read multiples registers****|

[ PAYLOAD ]  -------> FUNCTION 16 : Write multiple WORDs to server
[                 request :   HEADER PAYLOAD  (size 6 bytes) SIZE_PAYLOAD_WRITE_FROM_SERVER                                    ][    PAYLOAD    ] 
|0x10|MSB address first WORD|LSB address first WORD|MSB numbers of WORDS|LSB numbers of WORDS|numbers of BYTES|VALUES TO WRITES|
[               response :   HEADER PAYLOAD and PAYLOAD (size 5 bytes)  SIZE_PAYLOAD_WRITE_FROM_SERVER ] 
|0x10|MSB address first WORD|LSB address first WORD|MSB numbers of WORDS|LSB numbers of WORDS|
[ PAYLOAD ]  -------> FUNCTION 3 : read multiple WORDs from server
[                                             request :   HEADER PAYLOAD  (size 5 bytes)    ]
|0x3|MSB address first WORD|LSB address first WORD|MSB numbers of WORDS|LSB numbers of WORDS|
[response :   HEADER PAYLOAD  (size 2 bytes)][    PAYLOAD    ] 
|0x3  |   numbers of BYTES                  | VALUES TO READ ]
*/

//MODBUS TCP client sur port 502 JLS
//
// License: GPLv2
#include <Arduino.h>
#include <UIPEthernet.h>
#include <TimerOne.h>
#include "utility/logging.h"

// address of WORDS to write to server MODBUS/TCP
const uint8_t DIGITAL_IN_FIRST_WORD = 102; //%MW102 = D2, %MW103=D3, use of digital_in
const uint8_t ANALOG_IN_FIRST_WORD = 112;  //%MW112 = A2, %MW113=A3, use of analog_in
// address of WORDS to read from server MODBUS/TCP
const uint8_t ANALOG_OUT_FIRST_WORD = 125;  //%MW125 = D5, %MW126=D6, use of analog_out
const uint8_t DIGITAL_OUT_FIRST_WORD = 137; //%MW137 = D7, %MW138=D8, use of digital_out
// count of WORDS to write to server MODBUS/TCP
const uint8_t DIGITAL_IN_LENGTH = 3;      //D2,D3,D4
const uint8_t ANALOG_IN_FIRST_LENGTH = 6; //A2,A3,A4,A5,A6,A7
// count of WORDS to read from server MODBUS/TCP
const uint8_t ANALOG_OUT_FIRST_LENGTH = 2;  //D5,D6
const uint8_t DIGITAL_OUT_FIRST_LENGTH = 3; //D7,D8,D9

#define GREEN A0  //LED de debug
#define RED A1  //LED de debug

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.

#define MACADDRESS 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
#define MYIPADDR 192, 168, 1, 6
#define MYIPMASK 255, 255, 255, 0
// TCP MODBUS server is on UNITY PRO student simulator
// change your static IP address on WINDOWS
#define IP_MODBUS_SERVER 192, 168, 1, 101
#define TCP_MODBUS_PORT 502
// DO NOT CHANGE
uint8_t mac[6] = {MACADDRESS};
uint8_t myIP[4] = {MYIPADDR};
uint8_t myMASK[4] = {MYIPMASK};
EthernetClient client;

#define UARTBAUD 115200

#define WAITING_TO_NEXT_CONNECT 3000 //time in second between two try to connect
#define TIME_OUT 5000                // time between send to server and response from server
#define TIME_TO_UPDATE 2            // time between two data update every 500ms

// DO NOT CHANGE : used for function 16 and 3 of MODBUS/TCP trame
const uint8_t SIZE_OF_HEADER = 7;                 //|MSB counter trame|LSB counteur trame| 0 | 0 | 0 | lenght | 1 |
const uint8_t SIZE_PAYLOAD_WRITE_TO_SERVER = 6;   //|0x10|MSB add 1rst WORD|LSB add 1rst WORD|MSB count WORDS|LSB count WORDS|count BYTES|
const uint8_t SIZE_PAYLOAD_WRITE_FROM_SERVER = 5; //|0x10|MSB add 1rst WORD|LSB add 1rst WORD|MSB count WORDS|LSB count WORDS|
const uint8_t SIZE_PAYLOAD_READ_TO_SERVER = 2;    //|0x3|numbers of BYTES |
const uint8_t SIZE_PAYLOAD_READ_FROM_SERVER = 5;  //|0x10|MSB add 1rst WORD|LSB add 1rst WORD|MSB count WORDS|LSB count WORDS|
const uint8_t MODBUS_FUNCTION_READ = 3;           //read multiples words
const uint8_t MODBUS_FUNCTION_WRITE = 16;         //write multiples words

//DO NOT CHANGE : header
uint8_t header_modbus[SIZE_OF_HEADER] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

long wait = 0;
uint16_t counter_trame_send = 0; //counter of trame send to server
uint16_t counter_trame_receive = 0;//counter of trame receive from server

/******************************** STATE MACHINE ****************************************/
volatile uint8_t count_500ms = 0; //used for state machine
volatile bool flag_send=false;//used for state machine
enum class state
{
  ERROR,
  CONNECTING,
  CONNECTED,
  OK
};
volatile state connexion_state = state::ERROR;



struct in_out_arduino
{
  uint8_t *tab;
  uint8_t first_address_on_server;
  uint8_t first_offset_on_server;
  uint8_t size;
};
struct trame
{
  uint8_t *payload;
  uint8_t size_payload;
};

struct trame trame_digital_in_send;
struct trame trame_analog_in_send;
struct trame trame_digital_out_get;
struct trame trame_analog_out_get;

struct in_out_arduino digital_in, digital_out, analog_in, analog_out;

void state_machine()
{
  switch (connexion_state)
  {
  case state::ERROR:
    digitalWrite(GREEN, !digitalRead(GREEN));
    digitalWrite(RED, !digitalRead(RED));
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_strln(F(" STATE:ERROR"));
#endif
    break;
  case state::CONNECTING:
    digitalWrite(GREEN, 0);
    digitalWrite(RED, !digitalRead(RED));
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_strln(F(" STATE:CONNECTING"));
#endif
    break;
  case state::CONNECTED:
    digitalWrite(GREEN, !digitalRead(GREEN));
    digitalWrite(RED, 0);
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_strln(F(" STATE:CONNECTED"));
#endif
    break;
  case state::OK:
    digitalWrite(GREEN, 1);
    digitalWrite(RED, 0);
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_strln(F(" STATE:OK"));
#endif
    break;
  }
  count_500ms++;
  if (count_500ms == TIME_TO_UPDATE ){
    flag_send=true;
    count_500ms=0;
  }
}

void log_payload(struct trame *st)
{
#if ACTLOGLEVEL >= LOG_DEBUG
  LogObject.uart_send_str(F(" payload:"));
  LogObject.print(st->size_payload);
  LogObject.uart_send_str(F("  "));
  for (uint8_t i = 0; i < (st->size_payload); i++)
  {
    LogObject.print(st->payload[i], HEX);
    LogObject.uart_send_str(F(" "));
  }
#endif
}
void log_array(uint8_t *msg, uint8_t size)
{
#if ACTLOGLEVEL >= LOG_INFO
  uint8_t i;
  for (i = 0; i < size; i++)
  {
    LogObject.print(msg[i], HEX);
    LogObject.uart_send_str(F(" "));
  }
#endif
}
void log_data(struct in_out_arduino *data)
{
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.uart_send_str(F(" read:"));
  for (uint8_t i = 0; i < (data->size) * 2; i++)
  {
    LogObject.print(data->tab[i], HEX);
    if (i % 2 == 1)
      LogObject.uart_send_str(F(" "));
  }
#endif
}
void log_size(uint8_t size)
{
#if ACTLOGLEVEL >= LOG_DEBUG
  LogObject.uart_send_str(F(" malloc:"));
  LogObject.print(size);
  LogObject.uart_send_str(F("  "));
#endif
}
void log_size_real(uint8_t size_real)
{
#if ACTLOGLEVEL >= LOG_DEBUG
  LogObject.uart_send_str(F(" received:"));
  LogObject.print(size_real);
  LogObject.uart_send_str(F("  "));
#endif
}
void newLine()
{
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.uart_send_strln(F(""));
#endif
}

void init_struct_in_out()
{
  uint8_t add_dig_in = (DIGITAL_IN_FIRST_WORD / 10) * 10;
  uint8_t offset_dig_in = DIGITAL_IN_FIRST_WORD % add_dig_in;
  uint8_t add_ana_in = (ANALOG_IN_FIRST_WORD / 10) * 10;
  uint8_t offset_ana_in = ANALOG_IN_FIRST_WORD % add_ana_in;
  uint8_t add_dig_out = (DIGITAL_OUT_FIRST_WORD / 10) * 10;
  uint8_t offset_dig_out = DIGITAL_OUT_FIRST_WORD % add_dig_out;
  uint8_t add_ana_out = (ANALOG_OUT_FIRST_WORD / 10) * 10;
  uint8_t offset_ana_out = ANALOG_OUT_FIRST_WORD % add_ana_out;

  digital_in = {(uint8_t *)malloc(DIGITAL_IN_LENGTH * 2), add_dig_in, offset_dig_in, DIGITAL_IN_LENGTH};
  analog_in = {(uint8_t *)malloc(ANALOG_IN_FIRST_LENGTH * 2), add_ana_in, offset_ana_in, ANALOG_IN_FIRST_LENGTH};
  analog_out = {(uint8_t *)malloc(ANALOG_OUT_FIRST_LENGTH * 2), add_ana_out, offset_ana_out, ANALOG_OUT_FIRST_LENGTH};
  digital_out = {(uint8_t *)malloc(DIGITAL_OUT_FIRST_LENGTH * 2), add_dig_out, offset_dig_out, DIGITAL_OUT_FIRST_LENGTH};
}

int get_begin(struct in_out_arduino *st)
{
  return (st->first_address_on_server + st->first_offset_on_server);
}
int get_end(struct in_out_arduino *st)
{
  return (get_begin(st) + st->size);
}
int get_size(struct in_out_arduino *st)
{
  return st->size;
}
void init_tab(uint8_t *t, int size)
{
  for (uint8_t i = 0; i < size; i++)
    t[i] = 0;
}
void init_struct_trame_send(struct trame *st, struct in_out_arduino *st1)
{
  uint8_t size;
  size = get_size((struct in_out_arduino *)st1);
  st->size_payload = size * 2 + SIZE_PAYLOAD_WRITE_TO_SERVER;
  st->payload = (uint8_t *)malloc(st->size_payload);
  init_tab(st->payload, st->size_payload);
  st->payload[0] = MODBUS_FUNCTION_WRITE;
  st->payload[2] = get_begin(st1);
  st->payload[4] = size;
  st->payload[5] = size * 2;

  log_payload(st);
}

void init_struct_trame_get(struct trame *st, struct in_out_arduino *st1)
{
  uint8_t size;
  st->size_payload = SIZE_PAYLOAD_READ_FROM_SERVER;
  size = get_size((struct in_out_arduino *)st1);
  st->payload = (uint8_t *)malloc(st->size_payload);
  init_tab(st->payload, st->size_payload);
  st->payload[0] = MODBUS_FUNCTION_READ;
  st->payload[2] = get_begin(st1);
  st->payload[4] = size;

  log_payload(st);
}

void trap()
{
  while (1)
    ;
}

void verif_trame(uint8_t size_real, uint8_t size)
{
#if ACTLOGLEVEL >= LOG_DEBUG
  LogObject.uart_send_str(F(" size:"));
  LogObject.print(size);
  LogObject.uart_send_str(F(" size_real:"));
  LogObject.print(size_real);
#endif
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.uart_send_str(F(" count send:"));
  LogObject.print(counter_trame_send);
  LogObject.uart_send_str(F(" count receive:"));
  LogObject.print(counter_trame_receive);
#endif
  if (size_real == size)
  {
    if (counter_trame_receive == counter_trame_send)
    {
      connexion_state = state::OK;
      count_500ms = 0;
    }
    else
    {
      connexion_state = state::CONNECTED;
    }
  }
  else
  {
    connexion_state = state::ERROR;
  }
}

void init_pin()
{
  //2,3,4 INPUT PULL_UP
  //5,6 PWMout use of analogWrite();
  //7,8,9 OUTPUT
  //10,11,12,13 SPI pins for ENC28J60
  //A0,A1,A2,A3,A4,A5,A6 analog input
  uint8_t i;
  uint8_t begin = digital_in.first_offset_on_server;
  uint8_t end = digital_in.size + begin;
  for (i = begin; i < end; i++)
  {
    pinMode(i, INPUT_PULLUP);
  }
  begin = digital_out.first_offset_on_server;
  end = digital_out.size + begin;
  for (i = begin; i < end; i++)
  {
    pinMode(i, OUTPUT);
  }
}

int read_from_server(uint8_t *msg, uint8_t size)
{
  uint8_t size_real = 0;
  long next = millis() + TIME_OUT;
  while ((client.available() == 0) && (((long)millis()) < next))
  {
  }
  while ((size = client.available()) > 0)
  {
    size = client.read(msg, size);
    size_real += size;
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_str(F("  R:"));
    log_array(msg, size);
    counter_trame_receive = (((uint16_t)msg[0]) << 8) + msg[1];
#endif
  }
  return size_real;
}

void try_to_reconnect()
{

  uint8_t cpt_connect = 0;
  boolean end_of_setup = false;
  while (end_of_setup == false)
  {
    if (((signed long)(millis() - wait)) > 0)
    {
      cpt_connect++;
      wait = millis() + WAITING_TO_NEXT_CONNECT;
      // replace hostname with name of machine running tcpserver.pl
      //      if (client.connect("server.local",5000))
#if ACTLOGLEVEL >= LOG_INFO
      LogObject.uart_send_strln(F("setup : test connexion"));
#endif
      if (client.connect(IPAddress(IP_MODBUS_SERVER), TCP_MODBUS_PORT))
      {
        end_of_setup = true; // get out from setup
        wait = 0;
#if ACTLOGLEVEL >= LOG_INFO
        LogObject.uart_send_strln(F("Client connected"));
#endif
      }
      else
      {
#if ACTLOGLEVEL >= LOG_INFO
        LogObject.uart_send_str(F("Client connect failed :"));
        LogObject.uart_send_decln(cpt_connect);
#endif
      }
    }
  }
}
void create_trame(uint8_t *new_trame, struct trame *st, struct in_out_arduino *data)
{
  uint8_t i;
  counter_trame_send++;
  header_modbus[0] = (counter_trame_send >> 8) & 0xFF; //8 bits de poids fort
  header_modbus[1] = counter_trame_send & 0xFF;        //8 bits de poids faible
  header_modbus[5] = st->size_payload + 1;             //taille de la transaction
  for (i = 0; i < SIZE_OF_HEADER; i++)
    new_trame[i] = header_modbus[i];
  for (i = 0; i < st->size_payload; i++)
    new_trame[i + SIZE_OF_HEADER] = st->payload[i];
}
void update_payload(struct trame *st, struct in_out_arduino *data)
{
  for (uint8_t i = 0; i < (data->size) * 2; i++)
    st->payload[(SIZE_PAYLOAD_WRITE_TO_SERVER + i)] = data->tab[i];
}

uint8_t send_to_server(struct trame *st, struct in_out_arduino *data)
{
  uint8_t size_trame = (st->size_payload) + SIZE_OF_HEADER;
  uint8_t *trame = (uint8_t *)malloc(size_trame);
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.uart_send_str(F(" S:"));
#endif
  log_payload(st);
  create_trame(trame, st, data);
  client.write(trame, size_trame);
#if ACTLOGLEVEL >= LOG_INFO
  log_array(trame, size_trame);
#endif
  free(trame);
}

//broches 2,3,4
void read_digital_input()
{
  uint8_t i;
  uint8_t begin = digital_in.first_offset_on_server;
  for (i = 0; i < digital_in.size; i++)
  {
    digital_in.tab[2 * i] = 0;                          //on commence à la broche 2 on met dans tableau indice 0
    digital_in.tab[2 * i + 1] = digitalRead(begin + i); //on commence à la broche 2 on met dans tableau indice 0
  }
}
void read_analog_input()
{
  uint8_t i;
  uint8_t begin = analog_in.first_offset_on_server;
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.print("in:");
#endif
  for (i = 0; i < analog_in.size; i++)
  {
    int data = analogRead(i + begin);
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.print(data);
    LogObject.print(" ");
#endif
    analog_in.tab[2 * i] = (data >> 8) & 0xFF; //on commence à la broche 2 on met dans tableau indice 0
    analog_in.tab[2 * i + 1] = data & 0xFF;
  }
  LogObject.print(" ");
}
void write_digital_output(char *msg, uint8_t size)
{
  uint8_t i, bit_courant;
  uint8_t begin = digital_out.first_offset_on_server;
  log_array(msg, size);
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.uart_send_str(F("write: "));
#endif
  for (i = 0; i < digital_out.size; i++)
  {
    bit_courant = msg[SIZE_OF_HEADER + SIZE_PAYLOAD_READ_TO_SERVER + 2 * i + 1];
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.print(bit_courant, HEX);
    LogObject.uart_send_str(F(" "));
#endif
    digitalWrite(begin + i, bit_courant); //3 bits 7,8,9
  }
}

void write_analog_output(char *msg, uint8_t size)
{
  uint8_t i, bit_courant;
  uint8_t begin = analog_out.first_offset_on_server;
  log_array(msg, size);
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.uart_send_str(F("write: "));
#endif
  for (i = 0; i < analog_out.size; i++)
  {
    bit_courant = msg[SIZE_OF_HEADER + SIZE_PAYLOAD_READ_TO_SERVER + 2 * i + 1];
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.print(bit_courant, HEX);
    LogObject.uart_send_str(F(" "));
#endif
    analogWrite(begin + i, bit_courant); //3 bits 7,8,9
  }
}

void setup()
{
  Timer1.initialize(500000);             //500ms
  Timer1.attachInterrupt(state_machine); // blinkLED to run every 0.15 seconds
#if ACTLOGLEVEL > LOG_NONE
  LogObject.begin(UARTBAUD);
#endif
  init_struct_in_out();
  init_pin();
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  init_struct_trame_send(&trame_digital_in_send, &digital_in);
  newLine();
  init_struct_trame_send(&trame_analog_in_send, &analog_in);
  newLine();
  init_struct_trame_get(&trame_digital_out_get, &digital_out);
  newLine();
  init_struct_trame_get(&trame_analog_out_get, &analog_out);
  newLine();

  // initialize the ethernet device
  Ethernet.begin(mac, myIP);
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.uart_send_str(F("localIP: "));
  LogObject.println(Ethernet.localIP());
  LogObject.uart_send_str(F("subnetMask: "));
  LogObject.println(Ethernet.subnetMask());
#endif
#if ACTLOGLEVEL >= LOG_INFO
  LogObject.uart_send_str(F("try to connect to TCP/MODBUS server : "));
#endif
  connexion_state = state::CONNECTING;
  try_to_reconnect();
  connexion_state = state::CONNECTED;
}

void loop()
{
  uint8_t error, size, size_real;
  uint8_t *msg_receive;
  if (flag_send==true)
  {
    flag_send=false;
#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_str(F("analogIn: "));
#endif
    read_analog_input();
#if ACTLOGLEVEL >= LOG_INFO
    log_data(&analog_in);
#endif
    update_payload(&trame_analog_in_send, &analog_in);
    error = send_to_server(&trame_analog_in_send, &analog_in);
    size = SIZE_OF_HEADER + SIZE_PAYLOAD_WRITE_FROM_SERVER;
    log_size(size);
    msg_receive = (uint8_t *)malloc(size);
    size_real = read_from_server(msg_receive, size);
    log_size_real(size_real);
    verif_trame(size_real, size);
    newLine();
    free(msg_receive);

#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_str(F("digitalIn: "));
#endif
    read_digital_input();
#if ACTLOGLEVEL >= LOG_INFO
    log_data(&digital_in);
#endif
    update_payload(&trame_digital_in_send, &digital_in);
    error = send_to_server(&trame_digital_in_send, &digital_in);
    size = SIZE_OF_HEADER + SIZE_PAYLOAD_WRITE_FROM_SERVER;
    log_size(size);
    msg_receive = (uint8_t *)malloc(size);
    size_real = read_from_server(msg_receive, size);
    log_size_real(size_real);
    verif_trame(size_real, size);
    newLine();
    free(msg_receive);

#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_str(F("digitalOut: "));
#endif

    error = send_to_server(&trame_digital_out_get, &digital_out);
    size = SIZE_OF_HEADER + SIZE_PAYLOAD_READ_TO_SERVER + get_size(&digital_out) * 2;
    log_size(size);
    msg_receive = (uint8_t *)malloc(size);
    size_real = read_from_server(msg_receive, size);
    log_size_real(size_real);
    if (size_real > 0)
      write_digital_output(msg_receive, size);
    free(msg_receive);
    verif_trame(size_real, size);
    newLine();

#if ACTLOGLEVEL >= LOG_INFO
    LogObject.uart_send_str(F("analogOut: "));
#endif
    error = send_to_server(&trame_analog_out_get, &analog_out);
    size = SIZE_OF_HEADER + SIZE_PAYLOAD_READ_TO_SERVER + get_size(&analog_out) * 2;
    log_size(size);
    msg_receive = (uint8_t *)malloc(size);
    size_real = read_from_server(msg_receive, size);
    log_size_real(size_real);
    if (size_real > 0)
      write_analog_output(msg_receive, size);
    free(msg_receive);
    verif_trame(size_real, size);
    newLine();
  }
}
