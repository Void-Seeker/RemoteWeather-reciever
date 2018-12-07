#include <ArduinoJson.h>
#include <ArduinoSTL.h>
#include <list>
#include <algorithm>
#include <math.h>
#include <avr/wdt.h>
#include <RH_ASK.h>
#include <RHDatagram.h>
#include <SPI.h>
#define UART_SPEED 57600
//#define DEBUG
//#define DEBUG1
#define RXPIN 13
#define PROTOCOL 3579
#define SERVER_ADDRESS 2
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length()-1;

    for(int i=0; i<=maxIndex && found<=index; i++) {
        if(data.charAt(i)==separator || i==maxIndex) {
            found++;
            strIndex[0] = strIndex[1]+1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }

    return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
class Sensor
{
    int address, voltage, temperature, humidity, carboxy_level;
    long pressure;
    bool carboxy_present;
public:
    Sensor(String csv)
    {
        String buf;
        buf=getValue(csv,',',0);
        address = (buf!="")? buf.toInt():-1;
        buf=getValue(csv,',',1);
        voltage = (buf!="")? buf.toInt():0;
        buf=getValue(csv,',',2);
        temperature = (buf!="")? buf.toInt():0;
        buf=getValue(csv,',',3);
        pressure = (buf!="")? buf.toInt():0;
        buf=getValue(csv,',',4);
        humidity = (buf!="")? buf.toInt():0;
        buf=getValue(csv,',',5);
        carboxy_level = (buf!="")? buf.toInt():0;
        carboxy_present = (buf!="");
    }
    Sensor(int addr)
    {
        address = addr;
        voltage = temperature = pressure = humidity = carboxy_level = 0;
        carboxy_present = false;
    }
    void updateValues(const int volt,const int temp, const long pres, const int hum, const int carboxy = 0)
    {
        voltage=volt;
        temperature=temp;
        pressure=pres;
        humidity=hum;
        if (carboxy_present) carboxy_level=carboxy;
        return;
    }
    void updateValues(const Sensor s)
    {
        voltage=s.getVoltage();
        temperature=s.getTempC();
        pressure=s.getPressure();
        humidity = s.getHumidity();
        carboxy_present = s.carboxySensorPresent();
        carboxy_level = s.getCarboxyLevel();
        return;
    }
    int getAddress()
    {
        return address;
    }
    int getVoltage()
    {
        return voltage;
    }
    float getVoltageF()
    {
        return voltage/100.0;
    }
    int getTempC()
    {
        return temperature;
    }
    float getTempCF()
    {
        return temperature/100.0;
    }
    int getTempF()
    {
        return roundf(100.0*(getTempCF() * (9.0/5.0) + 32.0));
    }
    float getTempFF()
    {
        return getTempCF() * (9.0/5.0) + 32.0;
    }
    long getPressure()
    {
        return pressure;
    }
    float getPressureF()
    {
        return pressure/100.0;
    }
    int getHumidity()
    {
        return humidity;
    }
    float getHumidityF()
    {
        return (float)humidity;
    }
    bool carboxySensorPresent()
    {
        return carboxy_present;
    }
    int getCarboxyLevel()
    {
        return carboxy_level;
    }
    int getCarboxyLevelF()
    {
        return carboxy_level/100.0;
    }
    JsonObject getJSON()
    {
        StaticJsonDocument<200> doc;
        JsonObject buf = doc.to<JsonObject>();
        buf["SensorAddress"] = getAddress();
        buf["Voltage"] = getVoltageF();
        buf["Temperature"] = getTempF();
        buf["Pressure"] = getPressureF();
        buf["Humidity"] = getHumidity();
        if (carboxy_present) buf["CarboxyLevel"] = getCarboxyLevelF();
        return buf;
    }
    String getJSONString(const bool pretty = false)
    {
        StaticJsonDocument<200> doc;
        JsonObject buf = doc.to<JsonObject>();
        buf["SensorAddress"] = getAddress();
        buf["Voltage"] = getVoltageF();
        buf["Temperature"] = getTempCF();
        buf["Pressure"] = getPressureF();
        buf["Humidity"] = getHumidity();
        if (carboxy_present) buf["CarboxyLevel"] = getCarboxyLevelF();
        String out;
        if (pretty) serializeJsonPretty(buf,out);
        else serializeJson(buf,out);
        return out;
    }
    String getAPRSString()
    {
        char msg[42];
        sprintf(msg, ".../...g...t%03dr...p...P...h%02db%05ld",(int)roundf(getTempFF()),getHumidity(),(long)lround(getPressure()/10.0));
        return String(msg);
    }
    String getAPRSTelemetry()
    {
        //TODO: implement APRS Telemetry generating.
        return;
    }
    bool operator<(Sensor other) const
    {
        return address < other.address;
    }
    bool operator==(Sensor other) const
    {
        return address == other.address;
    }
    bool operator!=(Sensor other) const
    {
        return address != other.address;
    }
};
std::list <Sensor> sensorlist;
RH_ASK driver(2000,RXPIN);
RHDatagram manager(driver, SERVER_ADDRESS);
char buffer [32];
int cnt = 0;
boolean ready = false;
void ParseCommand(char* buffer)
{
    //TODO: Add automatic remorting mode and configuring over UART
    String buf = buffer;
    String newbuf;
    do {
        newbuf = buf;
        buf.replace("  "," ");
    } while (buf != newbuf);
    buf.trim();
    String command=(buf=="g")?"g":getValue(buf,' ',0);
    if (command == "g") {
        int addr = getValue(buf,' ',1).toInt();
        String mode = getValue(buf,' ',2);
        mode.toLowerCase();
        if (mode == "") mode = "json";
        if (addr == 0) {
            bool found=false;
            for(std::list<Sensor>::iterator i = sensorlist.begin(); i!=sensorlist.end(); ++i) {
                found = true;
                if(mode  == "json") Serial.println(i->getJSONString());
                if(mode  == "aprs") Serial.println(i->getAPRSString());
            }
            if (!found) Serial.println(F("No sensors found."));
        } else {
            std::list<Sensor>::iterator i = std::find(sensorlist.begin(), sensorlist.end(), Sensor(addr));
            if (i != sensorlist.end()) {
                if(mode  == "json") Serial.println(i->getJSONString());
                if(mode  == "aprs") Serial.println(i->getAPRSString());
            } else {
                Serial.println(F("error: sensor not found"));
            }
        }
    }
}
void setup()
{
    Serial.begin(UART_SPEED);   // Debugging only
    if (!manager.init())
#ifdef DEBUG
    {
        Serial.println("init failed");
        while (1);
    }
#else
        while (1);
#endif
    wdt_enable(WDTO_4S);
}
void loop()
{
    if (manager.available()) { // Non-blocking
        char buf[RH_ASK_MAX_MESSAGE_LEN];
        uint8_t buflen = sizeof(buf);
        if (manager.recvfrom(buf, &buflen)) {
            String packet = String(buf).substring(0,buflen);
#ifdef DEBUG
            Serial.print(F("Packet recieved:"));
            Serial.println(packet);
#endif
            if (getValue(packet,',',0).toInt() == PROTOCOL) {
                Sensor sens = Sensor(packet.substring(packet.indexOf(',')+1,packet.length()));
#ifdef DEBUG1
                Serial.println(sens.getJSONString(true));
#endif
                std::list<Sensor>::iterator it;
                it = std::find(sensorlist.begin(), sensorlist.end(), sens);
                if (it == sensorlist.end()) {
                    sensorlist.push_back(sens);
                } else {
                    it->updateValues(sens);
                }
#ifdef DEBUG2
                Serial.println(F("Current list:"));
                for(std::list<Sensor>::iterator i = sensorlist.begin(); i!=sensorlist.end(); ++i) {
                    Serial.println(i->getJSONString(true));
                }
                Serial.println(F("End of list."));
#endif
            }
        }
    }
    if (ready) {
        ParseCommand(buffer);
        ready = false;
    } else while (Serial.available()) {
            char c = Serial.read();
            buffer[cnt++] = c;
            if ((c == '\n') || (cnt == sizeof(buffer)-1)) {
                buffer[cnt] = '\0';
                cnt = 0;
                ready = true;
            }
        }
    wdt_reset();
}
