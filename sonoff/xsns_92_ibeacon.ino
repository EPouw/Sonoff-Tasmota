/*
  xsns_92_ibeacon.ino - Support for HM17 BLE Module + ibeacon reader

  Copyright (C) 2019  Gerhard Mutz and Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_IBEACON

#define IB_UPDATE_TIME 10

#define XSNS_92                          92

#define HM17_BAUDRATE 9600

// keyfob expires after N seconds
#define IB_TIMEOUT 30

uint8_t hm17_found,hm17_cmd,hm17_debug=0,hm_17_flag;

// 78 is max serial response
#define HM17_BSIZ 128
char hm17_sbuffer[HM17_BSIZ];
uint8_t hm17_sindex,hm17_result;
uint32_t hm17_lastms,upd_interval,tout_interval;

enum {HM17_TEST,HM17_ROLE,HM17_IMME,HM17_DISI,HM17_IBEA,HM17_SCAN,HM17_DISC,HM17_RESET,HM17_RENEW};
#define HM17_SUCESS 99

struct IBEACON {
  char FACID[8];
  char UID[32];
  char MAJOR[4];
  char MINOR[4];
  char PWR[2];
  char MAC[12];
  char RSSI[4];
};

#define MAX_IBEACONS 16

struct IBEACON_UID {
  char MAC[12];
  char RSSI[4];
  uint8_t FLAGS;
  uint8_t TIME;
} ibeacons[MAX_IBEACONS];


void IBEACON_Init() {
  ClaimSerial();
  SetSerialBaudrate(HM17_BAUDRATE);
  hm17_sendcmd(HM17_TEST);
  hm17_lastms=millis();
  upd_interval=IB_UPDATE_TIME;
  tout_interval=IB_TIMEOUT;
}

void hm17_every_second(void) {
  if (hm17_found) {
    if (upd_interval && (uptime%upd_interval==0)) {
      if (hm17_cmd!=99) {
        hm17_sendcmd(HM17_DISI);
        //if (hm_17_flag&1) hm17_sendcmd(HM17_DISI);
        //else hm17_sendcmd(HM17_DISC);
        //hm_17_flag^=1;
      }
    }
    for (uint32_t cnt=0;cnt<MAX_IBEACONS;cnt++) {
      if (ibeacons[cnt].FLAGS) {
        ibeacons[cnt].TIME++;
        if (ibeacons[cnt].TIME>tout_interval) {
          ibeacons[cnt].FLAGS=0;
          ibeacon_mqtt(ibeacons[cnt].MAC,"0000");
        }
      }
    }
  } else {
    if (uptime%20==0) {
      hm17_sendcmd(HM17_TEST);
    }
  }
}

void hm17_sbclr(void) {
  memset(hm17_sbuffer,0,HM17_BSIZ);
  hm17_sindex=0;
  Serial.flush();
}

void hm17_sendcmd(uint8_t cmd) {
  hm17_sbclr();
  hm17_cmd=cmd;
  if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("hm17cmd %d"),cmd);
  switch (cmd) {
    case HM17_TEST:
      Serial.write("AT");
      break;
    case HM17_ROLE:
      Serial.write("AT+ROLE1");
      break;
    case HM17_IMME:
      Serial.write("AT+IMME1");
      break;
    case HM17_DISI:
      Serial.write("AT+DISI?");
      break;
    case HM17_IBEA:
      Serial.write("AT+IBEA1");
      break;
    case HM17_RESET:
      Serial.write("AT+RESET");
      break;
    case HM17_RENEW:
      Serial.write("AT+RENEW");
      break;
    case HM17_SCAN:
      Serial.write("AT+SCAN5");
      break;
    case HM17_DISC:
      Serial.write("AT+DISC?");
      break;
  }
}

uint32_t ibeacon_add(struct IBEACON *ib) {
  // keyfob starts with ffff
  if (strncmp(ib->MAC,"FFFF",4)) return 0;

  for (uint32_t cnt=0;cnt<MAX_IBEACONS;cnt++) {
    if (!ibeacons[cnt].FLAGS) break;
    if (!strncmp(ibeacons[cnt].MAC,ib->MAC,12)) {
      // exists
      memcpy(ibeacons[cnt].RSSI,ib->RSSI,4);
      ibeacons[cnt].TIME=0;
      return 1;
    }
  }
  for (uint32_t cnt=0;cnt<MAX_IBEACONS;cnt++) {
    if (!ibeacons[cnt].FLAGS) {
      memcpy(ibeacons[cnt].MAC,ib->MAC,12);
      memcpy(ibeacons[cnt].RSSI,ib->RSSI,4);
      ibeacons[cnt].FLAGS=1;
      ibeacons[cnt].TIME=0;
      return 1;
    }
  }
  return 0;
}



void hm17_decode(void) {
  struct IBEACON ib;
  switch (hm17_cmd) {
    case HM17_TEST:
      if (!strncmp(hm17_sbuffer,"OK",2)) {
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("AT OK"));
        hm17_sbclr();
        hm17_result=HM17_SUCESS;
        hm17_found=1;
        //hm17_sendcmd(HM17_IBEA);
      }
      break;
    case HM17_ROLE:
      // OK+Set:1
      if (!strncmp(hm17_sbuffer,"OK+Set:1",8)) {
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("ROLE OK"));
        hm17_sbclr();
        hm17_result=HM17_SUCESS;
        //hm17_sendcmd(HM17_IMME);
      }
      break;
    case HM17_IMME:
      // OK+Set:1
      if (!strncmp(hm17_sbuffer,"OK+Set:1",8)) {
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("IMME OK"));
        hm17_sbclr();
        hm17_result=HM17_SUCESS;
        //hm17_found=1;
      }
      break;
    case HM17_IBEA:
      // OK+Set:1
      if (!strncmp(hm17_sbuffer,"OK+Set:1",8)) {
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("IBEA OK"));
        hm17_sbclr();
        hm17_result=HM17_SUCESS;
      }
      break;
    case HM17_SCAN:
        // OK+Set:1
        if (!strncmp(hm17_sbuffer,"OK+Set:5",8)) {
          if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("SCAN OK"));
          hm17_sbclr();
          hm17_result=HM17_SUCESS;
        }
        break;
    case HM17_RESET:
      // OK+Set:1
      if (!strncmp(hm17_sbuffer,"OK+RESET",8)) {
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("RESET OK"));
        hm17_sbclr();
        hm17_result=HM17_SUCESS;
      }
      break;
    case HM17_RENEW:
      // OK+Set:1
      if (!strncmp(hm17_sbuffer,"OK+RENEW",8)) {
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("RENEW OK"));
        hm17_sbclr();
        hm17_result=HM17_SUCESS;
      }
      break;
    case HM17_DISI:
    case HM17_DISC:
      if (!strncmp(hm17_sbuffer,"OK+DISCS",8)) {
        hm17_sbclr();
        hm17_result=1;
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("DISCS OK"));
        break;
      }
      if (!strncmp(hm17_sbuffer,"OK+DISCE",8)) {
        hm17_sbclr();
        hm17_result=HM17_SUCESS;
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("DISCE OK"));
        break;
      }
      if (!strncmp(hm17_sbuffer,"OK+NAME:",8)) {
        //if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("NAME OK"));
        //if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR(">>%s"),&hm17_sbuffer[8]);
        /*
        char hbuff[512];
        char *cp=hbuff;
        for (uint16_t cnt=0;cnt<hm17_sindex;cnt++) {
          sprintf(cp,"%s%02X",cp,hm17_sbuffer[cnt]);
        }
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR(">>%s"),hbuff);
        */
        //if ((hm17_sbuffer[hm17_sindex]==0x0a) && (hm17_sbuffer[hm17_sindex-1]==0x0d)) {

        if (hm17_sbuffer[hm17_sindex-1]=='\n') {
          hm17_result=HM17_SUCESS;
          if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("NAME OK"));
          if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR(">>%s"),&hm17_sbuffer[8]);
          hm17_sbclr();
        }
        break;
      }
      if (!strncmp(hm17_sbuffer,"OK+DIS0:",8)) {
        if (hm17_sindex==20) {
          hm17_result=HM17_SUCESS;
          if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("DIS0 OK"));
          if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR(">>%s"),&hm17_sbuffer[8]);
          hm17_sbclr();
        }
        break;
      }
      if (!strncmp(hm17_sbuffer,"OK+DISC:",8)) {
        if (hm17_cmd==HM17_DISI) {
          if (hm17_sindex==78) {
            if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR("DISC: OK"));
            //OK+DISC:4C 000C0E:003 A9144081A8 3B16849611 862EC1005: 0B1CE7485D :4DB4E940F C0E:-078
            if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR(">>%s"),&hm17_sbuffer[8]);
            memcpy(ib.FACID,&hm17_sbuffer[8],8);
            memcpy(ib.UID,&hm17_sbuffer[8+8+1],32);
            memcpy(ib.MAJOR,&hm17_sbuffer[8+8+1+32+1],4);
            memcpy(ib.MINOR,&hm17_sbuffer[8+8+1+32+1+4],4);
            memcpy(ib.PWR,&hm17_sbuffer[8+8+1+32+1+4+4],2);
            memcpy(ib.MAC,&hm17_sbuffer[8+8+1+32+1+4+4+2+1],12);
            memcpy(ib.RSSI,&hm17_sbuffer[8+8+1+32+1+4+4+2+1+12+1],4);

            //if (strncmp(ib.FACID,"00000000",8)) {
              if (ibeacon_add(&ib)) {
                ibeacon_mqtt(ib.MAC,ib.RSSI);
              }
            //}
            hm17_sbclr();
            hm17_result=1;
          }
        } else {
          if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR(">>%s"),&hm17_sbuffer[8]);
        }
        break;
      }
      /*
      if (!strncmp(hm17_sbuffer,"OK+DISC",7)) {
        if (hm17_debug) AddLog_P2(LOG_LEVEL_INFO, PSTR(">>%s"),&hm17_sbuffer[8]);
        break;
      }*/

  }
}

void IBEACON_loop() {
uint32_t difftime=millis()-hm17_lastms;

  while (Serial.available()) {
    hm17_lastms=millis();
    // shift in
    if (hm17_sindex<HM17_BSIZ) {
      hm17_sbuffer[hm17_sindex]=Serial.read();
      hm17_sindex++;
      hm17_decode();
    } else {
      hm17_sindex=0;
      break;
    }
  }

  if (hm17_cmd==99) {
   if (hm17_sindex>=HM17_BSIZ-2 || (hm17_sindex && (difftime>100))) {
     AddLog_P2(LOG_LEVEL_INFO, PSTR("%s"),hm17_sbuffer);
     hm17_sbclr();
   }
  }

}

#ifdef USE_WEBSERVER
const char HTTP_IBEACON[] PROGMEM =
 "{s}IBEACON-UID : %s" " - RSSI : %s" "{m}{e}";

void IBEACON_Show(void) {
char mac[14];
char rssi[6];

  for (uint32_t cnt=0;cnt<MAX_IBEACONS;cnt++) {
    if (ibeacons[cnt].FLAGS) {
      memcpy(mac,ibeacons[cnt].MAC,12);
      mac[12]=0;
      memcpy(rssi,ibeacons[cnt].RSSI,4);
      rssi[4]=0;
      WSContentSend_PD(HTTP_IBEACON,mac,rssi);
    }
  }

}
#endif  // USE_WEBSERVER

/*
commands sensor92
uT = sets update interval in seconds (scan tags every T seonds)
tT = sets timeout interval in seconds (after T seconds if tag is not detected send rssi=0)
c  = clears sensor list
s AT+xxx  = send native cmds to module
e.g. s AT+CONFFFF3D1B1E9D connects to module with ID, then send s AT to disconnect which activates the beeper in the TAG 
*/
bool XSNS_92_cmd(void) {
  bool serviced = true;
  const char S_JSON_IBEACON[] = "{\"" D_CMND_SENSOR "%d\":%s:%d}";
  const char S_JSON_IBEACON1[] = "{\"" D_CMND_SENSOR "%d\":%s:%s}";
  uint16_t len=XdrvMailbox.data_len;
  if (len > 0) {
      char *cp=XdrvMailbox.data;
      if (*cp>='0' && *cp<='8') {
        hm17_sendcmd(*cp&7);
        snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_IBEACON, XSNS_92,"hm17cmd",*cp&7);
      } else if (*cp=='s') {
        cp++;
        len--;
        while (*cp==' ') {
          len--;
          cp++;
        }
        Serial.write((uint8_t*)cp,len);
        hm17_cmd=99;
        snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_IBEACON1, XSNS_92,"hm17cmd",cp);
      } else if (*cp=='u') {
        cp++;
        if (*cp) upd_interval=atoi(cp);
        snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_IBEACON, XSNS_92,"uintv",upd_interval);
      } else if (*cp=='t') {
        cp++;
        if (*cp) tout_interval=atoi(cp);
        snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_IBEACON, XSNS_92,"lintv",tout_interval);
      } else if (*cp=='c') {
        for (uint32_t cnt=0;cnt<MAX_IBEACONS;cnt++) ibeacons[cnt].FLAGS=0;
        snprintf_P(mqtt_data, sizeof(mqtt_data), S_JSON_IBEACON1, XSNS_92,"clr list","");
      }
  } else {
    serviced=false;
  }
  return serviced;
}

void ibeacon_mqtt(const char *mac,const char *rssi) {
  char s_mac[14];
  char s_rssi[6];
  memcpy(s_mac,mac,12);
  s_mac[12]=0;
  memcpy(s_rssi,rssi,4);
  s_rssi[4]=0;
  int16_t n_rssi=atoi(s_rssi);
  Response_P(PSTR("{\"" D_JSON_TIME "\":\"%s\""), GetDateAndTime(DT_LOCAL).c_str());
  ResponseAppend_P(PSTR(",\"IBEACON_%s\":{\"RSSI\":%d}}"),s_mac,n_rssi);
  MqttPublishPrefixTopic_P(TELE, PSTR(D_RSLT_SENSOR), Settings.flag.mqtt_sensor_retain);
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns92(byte function)
{
  bool result = false;

    switch (function) {
      case FUNC_INIT:
        IBEACON_Init();
        break;
      case FUNC_LOOP:
        IBEACON_loop();
        break;
      case FUNC_EVERY_SECOND:
        hm17_every_second();
        break;
      case FUNC_COMMAND_SENSOR:
        if (XSNS_92 == XdrvMailbox.index) {
          result = XSNS_92_cmd();
        }
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        if (hm17_found) IBEACON_Show();
        break;
#endif  // USE_WEBSERVER
    }
    return result;
}

#endif  // USE_IBEACON
