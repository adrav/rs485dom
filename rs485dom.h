/**
 * @file      rs485dom.h
 * @version   1.0
 * @date      2018.12.24
 * @author    Rafal Bednarz
 * @contact   arduino@rbednarz.pl
 * 
 * @description
 *  Arduino library for multi-master communicating (with ACK) over RS485 line using ASCII characters. Could be connected to 
 *  RaspberryPI (over USB-RS485) or another arduino.
 *  Main features:
 *  - communication could be initialized from all sides (not only from master like Modbus)
 *  - communication is possible between any node on the line
 *  - messages could be send with ACK requirement
 *  - CRC in every message
 * 
 *  Why ASCII ?
 *   Supporting ASCII communicating is more difficult than binary in coding but it is more efficient in debugging and using. 
 *   You could simply turn on serial communication program such as "minicom" or "cu" and transmit or receive messages.
 *  
 *  Types of messages.
 *   There are two main types of messages. Message could be send to the wire without any confirmation from 
 *   the other side (like UDP package) but it could also be sent to another device with retries and wait for confirmation (aka TCP).
 *   Example:
 *     <G,i,tempout,31,7#8705,0,75 - message (info) from Arduino called "G" about temperature with 31 value (and 7). This messages
 *      does not require acknowledgment. 8705 is message counter with 0 retry. 75 is CRC
 *     
 *     <G,n,water,1,0#8704,2,65    - message (notify) from Arduino called "G" about water meter (one liter). This message requires 
 *      confirmation and will be re-sent couple times as ACK arises (or RS485_MAX_RETRIES times). "2" means that this is third retries.
 *  
 *  CRC checking:
 *   Every message (IN and OUT) contains checksum. If message does not contain valid checksum it is dropped.
 * 
 *  Wiring (example):
 *    Arduino PIN 10   <->  PIN RO    - MAX485 RS485 transceiver 
 *    Arduino PIN 11   <->  PIN DI    - MAX485 RS485 transceiver 
 *    Arduino PIN  3   <->  PIN DE+RE - MAX485 RS485 transceiver 
 *    MAX485 RS485 transceiver  Line A and B (twisted pair)
 *    Power (from arduino) +5V
 *    LED indicators to any PIN
 *    
 *    
 *  Implementation:
 *    This library has been created for my smart home system. Every PIR detectors, water meter, power meter etc is connected to 
 *    arduino which sends data to RaspberryPI. Arduinos is located in different places in my house so I need communication.
 *  
 *  It has been tested with Raspberry PI (using USB-RS485 with CH341 module).
 */
#include <SoftwareSerial.h>

#ifndef RS485DOM_H
#define RS485DOM_H

#define DEBUG



enum RS485 {
  RS485_MAX_LINE_LENGTH           = 250,  // Maximum line length. Package will be dropped with ERR_LINE_OVERFLOW error.
  RS485_MAX_NAMEFIELD_LENGTH      = 30,   // Maximum character count for "name" field in package. Please remember to add \0 char at the end.
  RS485_MIN_LINE_LENGTH           = 16,   // Minimum line length for input. Package with less characters will be dropped with ERR_LINE_TOO_SHORT error
  RS485_MAX_RETRIES               = 10,   // How many times package with ackRequired bit set will be send and arduino will wait for acknowledgement. Afterwards it is assumed that packed has been delivered (no more tries).
  RS485_HEARTBEAT_DURATION        = 60,   // Duration beetween heartbeat [s].
  RS485_QUEUE_SIZE                = 5,    // Output queue size (stack). Every message is stored in queue and dequeue() method pulls from the stack and send. ERR_QUEUE_OVERFLOW will be returned if there is no space in stack.
  RS485_MAX_INPUT_TIME            = 1,    // All parts (characters) of incoming message must be delivered in specified time [s]. Check code for details.
  RS485_PIN_ACTIVITY_IN_DURATION  = 50    // Defines how long LED for incoming characters must be turned on [ms]
};


enum RS485_COMMANDS {
   COMMAND_SET    = 's',  // set - requires ACK from master
   COMMAND_GET    = 'g',  // get - no ACK required
   COMMAND_ACK    = 'a',  // ack - ack package
   COMMAND_INFO   = 'i',  // info - no ACK required
   COMMAND_NOTIFY = 'n',  // notify - requires ACK from master
   COMMAND_TEST   = 'x',  // only for test
   COMMAND_TEST2  = 'y',  // only for test
   COMMAND_ERROR  = 'e',  // error 
   COMMAND_DELME  = '-'   // internal value - such package will be deleted from queue (used after ACK send)
};

/** 
 *  Main structure of package
 *  This package will be sent in ASCII
 */
typedef struct RS485message_ {
  char system[3];                         // two characters + \0. First character define direction "<" or ">" and second system name
  char command;                           // command from RS485_COMMANDS
  char name[RS485_MAX_NAMEFIELD_LENGTH];  // any data in ASCII termiated with \0
  int32_t value;                          // any value in signed 4bytes int
  int32_t opt;                            // any value in signed 4bytes int

  uint16_t counter;                       // every output message contains counter (incremented)
  uint8_t retry;                          // for ACK required packed - every "tries" contains +1 retry counter (retry < RS485_MAX_RETRIES)
  uint8_t crc;                            // crc counted for all data in package
  bool ackRequired;                       // if ACK is required
} RS485message; 


enum RS485_PROCESS_CODES {
  LINE_OK            = 0,   // Input line parsed properly
  ACK_OK             = 1,   // ACK sent properly
  LINE_STILL_READING = 10,  // Arduino is still reading characters from input line (it will finish when \n comes or RS485_MAX_LINE_LENGTH). At last one character has been delivered.
  LINE_EMPTY         = 11,  // Nothing on a line. No messages is being reading.
  NOT_FOR_ME         = 30,  // This messages is not for me
  ERR_LINE_OVERFLOW  = 40,  // Too many characters in input line
  ERR_LINE_TOO_SHORT = 42,  // Too few characters read from the line
  ERR_PARSE_FIELD1   = 51,
  ERR_PARSE_FIELD2   = 52,
  ERR_PARSE_FIELD3   = 53,
  ERR_PARSE_FIELD4   = 54,
  ERR_PARSE_FIELD5   = 55,
  ERR_PARSE_FIELD6   = 56,
  ERR_PARSE_FIELD7   = 57,
  ERR_PARSE_FIELD8   = 58,
  ERR_PARSE_FIELD9   = 59,
  ERR_WRONG_CRC      = 50,  // Package contains wrong CRC
  ERR_ACK_REQUIRED   = 60,  // ACK expected but other package came
  ERR_WRONG_ACK      = 61,  // ACK delivered but for wrong package than we expected
  ERR_SET_HANDLER    = 62,  // if SET handler could not perform operation - it means that 'set' command has not been executed properly
  ERR_QUEUE_OVERFLOW = 70   // No space in output queue
};


/**
 * RS485Dom (House) class
 */
class RS485Dom {
  
  private:

    SoftwareSerial *port = NULL;

    /**
     * Global system identifier
     */
    char systemName;

    /**
     * LED indicator for input data
     */
    uint8_t activityPINin;

    /**
     * Jak przychodza dane to chcemy mignac dioda - potrzebny jest timeout by nia zaswiecic na chwile
     */
    uint32_t activityPINinTimeout = 0;
    
    /**
     * LED indicator for output data
     */
    uint8_t activityPINout;

    /**
     * directionControlPIN (must be HIGH when transmit, and LOW when waiting for data)
     */
    uint8_t directionControlPIN;

    const uint8_t RS485Transmit = HIGH;
    const uint8_t RS485Receive  = LOW;


    /**
     * Incoming message
     */
    RS485message inMessage;

    /**
     * Outgoing messages
     * This is queue for outgoing messages
     */
    RS485message outMessage[RS485_QUEUE_SIZE]; 

    /**
     * Current index in outMessage table
     */
    int8_t outMessageIdx = -1;

    /**
     * Buffer for characters (for input and output)
     */
    char ioBuffer[RS485_MAX_LINE_LENGTH];
    
    /**
     * Current index in ioBuffer table
     */
    uint8_t ioBufferIdx = 0;

    
    /**
     * Global counter.
     */
    uint16_t globalCounter = 0;

    /**
     * Time when last message has been sent
     */
    uint32_t lastMessageMillis = 0;

    /**
     * Time when last heartbeat message has been sent
     */
    uint32_t lastHBMillis = 0;

    /**
     * Incomming characters timer
     * The input message must be delivered in specified time (RS485_MAX_INPUT_TIME)
     */
    uint32_t inputTimeout = 0;

    bool(*commandSETHandlerFunction)(RS485message *m);
    bool(*commandGETHandlerFunction)(RS485message *m);
    void(*commandINFOHandlerFunction)(RS485message *m);
    void(*commandNOTIFYHandlerFunction)(RS485message *m);

    /**
     * Command ERROR function Handler - this is the local callback to notify that some commands failed. I.e. queue count overflow
     */
    void (*commandERRORHandlerFunction)(uint8_t errorCode);

    uint8_t CRCiterate(char *ptr, uint8_t *c);
    uint8_t packageChecksum(RS485message *p);
   
    void doSendMessage(RS485message *m);
    void sendACKmessage();
    void sendErrorMessage(uint8_t errCode);
    void sendQueueOverflowErrorMessage();

    void constructMessage(RS485message *m, uint8_t checksum);
    void sendMessage();
    void sendWaterTrigger();
    void sendTemperature();
    void sendHeartbeatMessage();
    void addToSendQueue(char command, const char* name, uint32_t value, uint32_t opt, bool ackRequired);
    void dequeue();
    uint16_t getNextGlobalCounter();
    int8_t readFromPort();
    int8_t processData();

    int freeRam ();

    void cleanOutMessageQueue();

    
    

    
  public:
    RS485Dom(char _systemName, SoftwareSerial *serial, uint8_t _activityPINin, uint8_t _activityPINout, uint8_t _directionControlPIN);

    void sendStartupMesssage();

    void sendKONTRMessage(uint32_t kontr_code);
    
    void sendCustomNotifyMessage(char name[RS485_MAX_NAMEFIELD_LENGTH], int32_t value, int32_t opt);
    void sendCustomInfoMessage(char name[RS485_MAX_NAMEFIELD_LENGTH], int32_t value, int32_t opt);
    void sendCustomGetMessage(const char name[RS485_MAX_NAMEFIELD_LENGTH], int32_t value, int32_t opt);

    void setCommandSETHandler(bool(*function)(RS485message *m));
    void setCommandGETHandler(bool(*function)(RS485message *m));
    void setCommandINFOHandler(void(*function)(RS485message *m));
    void setCommandNOTIFYHandler(void(*function)(RS485message *m));
    

    void setERRORHandler(void (*function)(uint8_t errorCode));


    uint8_t loop();

};

/**
 * Main constructor.
 *  serial - could be NULL - than Hardware "Serial" will used
 */
RS485Dom::RS485Dom(char _systemName, SoftwareSerial *serial, uint8_t _activityPINin, uint8_t _activityPINout, uint8_t _directionControlPIN) {
  
  port = serial;

  #ifdef DEBUG_PORT_OVERRIDE
    port = NULL; //to tylko dla testow - wtedy przyjmuje dane ze zwyklego portu
  #endif

  systemName = _systemName;

  
  activityPINin = _activityPINin;
  activityPINout = _activityPINout;
  directionControlPIN = _directionControlPIN;

  digitalWrite(directionControlPIN, RS485Receive);  // Init Transceiver

}

/**
 * Auxiliary function
 */
uint8_t RS485Dom::CRCiterate(char *ptr, uint8_t *c) {

    while (*ptr != '\0') {
      *c ^= *(ptr++);
    }

}

/**
 * Count package checksum (CRC)
 */
uint8_t RS485Dom::packageChecksum(RS485message *p)
{

  uint8_t c = 0;

  CRCiterate(p->system, &c);
  c ^= p->command;
  CRCiterate(p->name, &c);

  c ^= (p->value >> 0) & 0xff;
  c ^= (p->value >> 8) & 0xff;
  c ^= (p->value >> 16) & 0xff;
  c ^= (p->value >> 24) & 0xff;

  c ^= (p->opt >> 0) & 0xff;
  c ^= (p->opt >> 8) & 0xff;
  c ^= (p->opt >> 16) & 0xff;
  c ^= (p->opt >> 24) & 0xff;

  c ^= (p->counter >> 0) & 0xff;
  c ^= (p->counter >> 8) & 0xff;

  c ^= p->retry;

  return c;

}

/**
 * Prepare message for sending
 */
void RS485Dom::constructMessage(RS485message *m, uint8_t checksum) {

  ioBufferIdx = 0;

  snprintf(ioBuffer, RS485_MAX_LINE_LENGTH, "%s,%c,%s,%ld,%ld,%d#%ld,%d,%d\n", 
    m->system, m->command, m->name, m->value, m->opt, m->counter, m->retry, m->crc);

}

/**
 * Parse input line and store in ioBuffer
 * input struct:
 *  >G,x,yyy,0,1#A,B,C
 *  where:
 *    >   - in message (< for Out)
 *    G   - system identifier
 *    x   - COMMAND - set/get/notify/ack
 *    yyy - whatever
 *    0   - value
 *    1   - opt
 *    A   - counter
 *    B   - retry counter
 *    C   - checksum
 *    
 * example:
 * >G,s,abc,0,0#1,0,107
 * >G,x,test,0,0#1,0,22
 * >G,y,test,0,0#1,0,23
 * 
 * >G,a,water,1,0#1,2,111
 * 
 */
int8_t RS485Dom::processData()
{

  if (ioBufferIdx < RS485_MIN_LINE_LENGTH) {
    #if RS485_HW_SERIAL != Serial
      Serial.println(ioBuffer);
    #endif
    return ERR_LINE_TOO_SHORT;

  }
    

  char *token;
  char comma[2] = ",";
  char hash[2]  = "#";
  uint8_t countedCRC;

  #if RS485_HW_SERIAL != Serial
    Serial.print(F("#"));
    Serial.print(systemName);
    Serial.print(": ");
    Serial.println(ioBuffer);
  #endif


  token = strtok(ioBuffer, comma);
  if (token != NULL) {
    strncpy(inMessage.system, token, sizeof(inMessage.system)-1);
    inMessage.system[sizeof(inMessage.system)-1] = 0;
  } else {
    return ERR_PARSE_FIELD1;
  }

  char s[] = {'>', systemName, 0};
  if (strcmp(inMessage.system, s) != 0) 
    return NOT_FOR_ME;

  token = strtok(NULL, comma);
  if (token != NULL) {
    inMessage.command = token[0];
  } else {
    return ERR_PARSE_FIELD2;
  }

  token = strtok(NULL, comma);
  if (token != NULL) {
    strncpy(inMessage.name, token, sizeof(inMessage.name)-1);
    inMessage.name[sizeof(inMessage.name)-1] = 0;

  } else {
    return ERR_PARSE_FIELD3;
  }

  token = strtok(NULL, comma);
  if (token != NULL) {
    inMessage.value = strtol(token, NULL, 10);
  } else {
    return ERR_PARSE_FIELD4;
  }

  token = strtok(NULL, hash);
  if (token != NULL) {
    inMessage.opt = strtol(token, NULL, 10);
  } else {
    return ERR_PARSE_FIELD5;
  }

  token = strtok(NULL, comma);
  if (token != NULL) {
    inMessage.counter = strtol(token, NULL, 10);
  } else {
    return ERR_PARSE_FIELD6;
  }

  token = strtok(NULL, comma);
  if (token != NULL) {
    inMessage.retry = atoi(token);
  } else {
    return ERR_PARSE_FIELD7;
  }

  token = strtok(NULL, comma);
  if (token != NULL) {
    inMessage.crc = atoi(token);
  } else {
    return ERR_PARSE_FIELD8;
  }

  countedCRC = packageChecksum(&inMessage);

#ifdef DEBUG
//  Serial.println(F("Rozparsowany:"));
//  Serial.println(inMessage.system);
//  Serial.println(inMessage.command);
//  Serial.println(inMessage.name);
//  Serial.println(inMessage.value);
//  Serial.println(inMessage.opt);
//  Serial.println(inMessage.counter);
//  Serial.println(inMessage.retry);
//  Serial.println(inMessage.crc);
#endif

  if (inMessage.crc != countedCRC) {
    //Serial.print(F("#G counted crc: "));
    //Serial.println(countedCRC);

    return ERR_WRONG_CRC;
  }

  // w tym miejscu mamy poprawnie rozparsowana wiadomosc przychodzaca
  // sprawdzamy czy wiadomosc ktora przyszla jest ack'iem na 

  switch (inMessage.command) {
    case COMMAND_ACK:
        // szukamy czy jest pakiet do ktorego ten ack pasuje

      for (uint8_t i=0; i <= outMessageIdx; i++) {
        if (outMessage[i].counter == inMessage.value) { //znalezlismy pakiet na ktory przyszlo potwierdzenie
          outMessage[i].command = COMMAND_DELME; // set as garbage - ten pakiet z outputa zostanie usuniety przez funkcje dequeue
          return ACK_OK;        
        }
      }

      return ERR_WRONG_ACK;

    case COMMAND_SET:

      // perform operation
      if (commandSETHandlerFunction)  {
        if (! commandSETHandlerFunction(&inMessage)) { //if this function returns false, the ACK should not be sent
          return ERR_SET_HANDLER;
        }
      }

      sendACKmessage();
      break;

    case COMMAND_INFO:
      if (commandINFOHandlerFunction)  {
        commandINFOHandlerFunction(&inMessage);
      };
      break;

    case COMMAND_NOTIFY:
      if (commandNOTIFYHandlerFunction)  {
        commandNOTIFYHandlerFunction(&inMessage);
      };
      break;

    case COMMAND_GET:
      if (commandGETHandlerFunction)  {
        commandGETHandlerFunction(&inMessage);
      };
      break;

    case COMMAND_TEST:
      sendWaterTrigger();
      sendWaterTrigger();
      sendTemperature();
      break;

    case COMMAND_TEST2:
      sendTemperature();
      sendTemperature();
      break;
  }

  return LINE_OK;


}


/**
 * Reading from RS485 (or Serial)
 */
int8_t RS485Dom::readFromPort()
{

  uint8_t byteReceived;

  if (port != NULL) {
    byteReceived = port->read();    // Read received byte
  } else {
    byteReceived = RS485_HW_SERIAL.read();    // Read received byte
  } 

  #if RS485_HW_SERIAL != Serial
  Serial.print((char)byteReceived);
  #endif

  uint8_t result = LINE_STILL_READING;


  if (ioBufferIdx > 0 && ((millis() - inputTimeout) > RS485_MAX_INPUT_TIME*1000)) { // if character has been delivered after RS485_MAX_INPUT_TIME milliseconds after previous - let's start from the beginning
      ioBufferIdx = 0;
      ioBuffer[0] = 0;
  }
  
  switch (byteReceived) {
    case '\r':
      break;
      
    case '\n':
      result = processData();
      ioBufferIdx = 0;
      ioBuffer[0] = 0;
      #if RS485_HW_SERIAL != Serial
        Serial.print("");
      #endif
      
      break;

    default:
      inputTimeout = millis(); //every character reset inputTimeout watchdog

      if ((ioBufferIdx+1) < RS485_MAX_LINE_LENGTH) {
        ioBuffer[ioBufferIdx++] = byteReceived;
        ioBuffer[ioBufferIdx] = 0;
      } else {
        ioBuffer[0] = 0;
        ioBufferIdx = 0;
        //Serial.println(ioBuffer);
        return ERR_LINE_OVERFLOW;
      }
      break;       
  }

  return result;

}



void RS485Dom::doSendMessage(RS485message *m) {

  char textout[RS485_MAX_LINE_LENGTH];

  snprintf(textout, RS485_MAX_LINE_LENGTH-1, "%s,%c,%s,%ld,%ld#%u,%u,%u\n\r", 
    m->system, m->command, m->name, m->value, m->opt, m->counter, m->retry, m->crc);

  digitalWrite(directionControlPIN, RS485Transmit);  // RS485 Transmit enable

  if (port != NULL) {
    port->write(textout);          // Send message to RS485 line
  } else {
    RS485_HW_SERIAL.print(textout);
    RS485_HW_SERIAL.flush();
  }

  digitalWrite(activityPINout, HIGH);  // Show activity    

  delay(10);

  digitalWrite(directionControlPIN, RS485Receive);  // RS485 Transmit disable

  digitalWrite(activityPINout, LOW);  // Show activity

}

/**
 * Sends ACK message for the package that requires ACK
 * We are using inMessage structure (with small modifications)
 */
void RS485Dom::sendACKmessage() 
{

  inMessage.command = COMMAND_ACK;

  char s[] = {'<', systemName, 0};
  strcpy(inMessage.system, s);
  inMessage.value = inMessage.counter;
  inMessage.counter = getNextGlobalCounter();

  inMessage.crc = packageChecksum(&inMessage);

  doSendMessage(&inMessage);
  
}

/**
 * Get the next counter. It wraps after 60000
 */
uint16_t RS485Dom::getNextGlobalCounter() {
  globalCounter++;

  if (globalCounter > 60000)
    globalCounter = 0;

  return globalCounter;
}

/**
 * ackRequired - czy wymagane jest potwierdzenie
 */
void RS485Dom::sendMessage()
{

  if (outMessageIdx < 0) {
    return;
  }

  outMessage[0].crc = packageChecksum(&outMessage[0]);

  lastMessageMillis = millis();

  doSendMessage(&(outMessage[0]));

  if (false == outMessage[0].ackRequired)
    outMessage[0].command = COMMAND_DELME;  //jezeli nie wymagamy ack to oznaczamy do usuniecia


}

void RS485Dom::sendQueueOverflowErrorMessage()
{

  inMessage.command = COMMAND_ERROR;
 
  char s[] = {'<', systemName, 0};
  strcpy(inMessage.system, s);
  strcpy(inMessage.name, "queueoverflow");
  inMessage.value = ERR_QUEUE_OVERFLOW;
  inMessage.opt = RS485_QUEUE_SIZE;
  inMessage.retry = 0;
  inMessage.counter = getNextGlobalCounter();

  inMessage.crc = packageChecksum(&inMessage);

  doSendMessage(&inMessage);  
  
}


/**
 * Sends error information to the line
 */
void RS485Dom::sendErrorMessage(uint8_t errCode) 
{

  inMessage.command = COMMAND_ERROR;
  
  char s[] = {'<', systemName, 0};
  strcpy(inMessage.system, s);
  inMessage.value = errCode;
  inMessage.opt = 0;
  inMessage.retry = 0;
 
  inMessage.counter = getNextGlobalCounter();

  inMessage.crc = packageChecksum(&inMessage);

  doSendMessage(&inMessage);  
}


void RS485Dom::sendKONTRMessage(uint32_t kontr_code) 
{

  char cmd[] = "K";
  addToSendQueue(COMMAND_NOTIFY, cmd, kontr_code, 0, true);  
  
}


void RS485Dom::sendCustomNotifyMessage(char name[RS485_MAX_NAMEFIELD_LENGTH], int32_t value, int32_t opt) {
  addToSendQueue(COMMAND_NOTIFY, name, value, opt, true);  
}

void RS485Dom::sendCustomInfoMessage(char name[RS485_MAX_NAMEFIELD_LENGTH], int32_t value, int32_t opt) {
  addToSendQueue(COMMAND_INFO, name, value, opt, false);  
}

void RS485Dom::sendCustomGetMessage(const char name[RS485_MAX_NAMEFIELD_LENGTH], int32_t value, int32_t opt) {
  addToSendQueue(COMMAND_GET, name, value, opt, false);  
}



/**
 * Main loop method - it should be invoked in arduino loop() function
 */
uint8_t RS485Dom::loop()
{

    #if RS485_HW_SERIAL != Serial
      if (Serial.available()) {
        Serial.println("Smieci: ");
        Serial.println(Serial.read()); // clean Serial buffer. If we are using SoftwareSerial (RS485) we do not need Serial line. 
      }
    #endif

  dequeue();

  uint8_t isData;
  
  if (port != NULL) {
    isData = port->available();
  } else {
    isData = RS485_HW_SERIAL.available();
  }

  int8_t r = LINE_EMPTY;
  
  if (isData != 0) {
    digitalWrite(activityPINin, HIGH); // Turn ON "input data" LED indicator. Here we are set it to high, and start inTimeout. This LED will be turned off when timer times out (after RS485_PIN_ACTIVITY_IN_DURATION milliseconds).
    activityPINinTimeout = millis();

    r = readFromPort();
    //Serial.println(r);
  }

  if ((millis() - activityPINinTimeout) >  RS485_PIN_ACTIVITY_IN_DURATION) {
    digitalWrite(activityPINin, LOW);
 
  }

  

  if (((millis() - lastHBMillis)/1000) > (RS485_HEARTBEAT_DURATION)) {
    lastHBMillis = millis();
    sendHeartbeatMessage();
  }

  switch (r) {
    case ERR_PARSE_FIELD2:
    case ERR_PARSE_FIELD3:
    case ERR_PARSE_FIELD4:
    case ERR_PARSE_FIELD5:
    case ERR_PARSE_FIELD6:
    case ERR_PARSE_FIELD7:
    case ERR_PARSE_FIELD8:
    case ERR_PARSE_FIELD9:
    case ERR_WRONG_CRC:
    case ERR_ACK_REQUIRED:
    case ERR_WRONG_ACK:
    case ERR_SET_HANDLER:
      sendErrorMessage(r);
      if (commandERRORHandlerFunction) {
        commandERRORHandlerFunction(r);
      }

      break;
  }

  return r;
    
  
}


void RS485Dom::setCommandSETHandler(bool(*func)(RS485message *m)) {
  commandSETHandlerFunction = func;
}

void RS485Dom::setCommandGETHandler(bool(*func)(RS485message *m)) {
  commandGETHandlerFunction = func;
}

void RS485Dom::setCommandINFOHandler(void(*func)(RS485message *m)) {
  commandINFOHandlerFunction = func;
}
void RS485Dom::setCommandNOTIFYHandler(void(*func)(RS485message *m)) {
  commandNOTIFYHandlerFunction = func;
}


void RS485Dom::setERRORHandler(void (*func)(uint8_t errorCode)) {
  commandERRORHandlerFunction = func;  
}





void RS485Dom::sendStartupMesssage() {

  char cmd[] = "start";
  addToSendQueue(COMMAND_INFO, cmd, 0, 0, false);
 
}



void RS485Dom::sendHeartbeatMessage() {

  char cmd[] = "hb";
  addToSendQueue(COMMAND_INFO, cmd, lastHBMillis/1000, freeRam(), false);
  
}

/**
 * Wysyla informacje o impulsie na wodomierzu
 */
void RS485Dom::sendWaterTrigger() {

  char cmd[] = "water";
  addToSendQueue(COMMAND_NOTIFY, cmd, 1, 0, true);

}

void RS485Dom::dequeue()
{

//  if (COMMAND_DELME == outMessage[outMessageIdx].command) {
//    outMessageIdx--;
//    return;
//  }

  cleanOutMessageQueue();

  if (outMessageIdx < 0)
    return;

//  Serial.println(outMessageIdx);
  
  switch (outMessage[0].ackRequired) { //dotyczy ostatniej wiadomosci:
    case true: //jezeli czekamy na ack'a i nie dostalismy go w ciagu ostatnich X milisekund - to ponow wysylke
      if ((millis() - lastMessageMillis) > 1000) {

        // increments retry counter of last message
        outMessage[0].retry++ ;

        if (outMessage[0].retry >= RS485_MAX_RETRIES) { 
          // assume that message has been delivered if ACK has not been sent so far.
          // TODO: maybe some LED should be turn on in this situation?
          
          outMessage[0].command = COMMAND_DELME;
          return ; // no more tries
        }
      
        lastMessageMillis = millis();
        sendMessage();

      }
      break;

    case false:    
      sendMessage();
      break;
    
     
  }
 
  
}

/**
 * Put the message on the output queue
 * It will be sent in dequeue() method
 */
void RS485Dom::addToSendQueue(char command, const char* name, uint32_t value, uint32_t opt, bool ackRequired)
{

#ifdef DEBUG
  #if RS485_HW_SERIAL != Serial
    Serial.print(F("#Add2queue: "));
    Serial.print(command);
    Serial.print(" ");
    Serial.print(name);
    Serial.print(" ");
    Serial.print(value);
    Serial.print(" ");
    Serial.println(opt);
  #endif

  
#endif
  
  if (outMessageIdx + 1 >=  RS485_QUEUE_SIZE) {
    //Serial.println(F("KOLEJKA PRZEKROCZONA"));

    sendQueueOverflowErrorMessage();

    if (commandERRORHandlerFunction) {
      commandERRORHandlerFunction(ERR_QUEUE_OVERFLOW);
    }
    
    return;
  }
    
  outMessageIdx++;

  char s[] = {'<', systemName, 0};
  strcpy(outMessage[outMessageIdx].system, s);
  outMessage[outMessageIdx].command = command;
  strcpy(outMessage[outMessageIdx].name, name);
  outMessage[outMessageIdx].value = value;
  outMessage[outMessageIdx].opt = opt;
  outMessage[outMessageIdx].counter = getNextGlobalCounter();
  outMessage[outMessageIdx].retry = 0;
  outMessage[outMessageIdx].ackRequired = ackRequired;

 
}

/**
 * Wysyla informacje o temperaturze - nie wymaga potwierdzenia
 */
void RS485Dom::sendTemperature() {

  char cmd[] = "tempout";
  addToSendQueue(COMMAND_INFO, cmd, 31, 7, false);
  
}

/**
 * Get the free space in SRAM
 */
int RS485Dom::freeRam () {
   extern int __heap_start, *__brkval;
   int v;
   return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/**
 * Clean out message queue. All "COMMAND_DELME" entries are being removed.
 * It is required to make queue FIFO
 */
void RS485Dom::cleanOutMessageQueue()
{
  uint8_t newIdx = -1;

  for (uint8_t i=0; i <= outMessageIdx; i++) {

    if (COMMAND_DELME == outMessage[i].command) {
      for (uint8_t j=i+1; j<=outMessageIdx; j++) {

        if (COMMAND_DELME != outMessage[j].command) {


          memcpy(&outMessage[i], &outMessage[j], sizeof(RS485message));

          outMessage[j].command = COMMAND_DELME;
          newIdx = i;
          break;
        }
        
      }
    } else {
      newIdx = i;
    }
    
  }

  outMessageIdx = newIdx;

 
}

#endif
