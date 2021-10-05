
#include "influxdb.h"

static AsyncClient *influx_Client = NULL;
static int influx_port;
static String influx_host;
static const char invalidChars[] = "$&+,/:;=?@ <>#%{}|\\^~[]`";

//Send a few influx packets and keep track so we send the next batch on following calls
static uint8_t influx_StartModule = 0;

char hex_digit(char c)
{
    return "0123456789ABCDEF"[c & 0x0F];
}

String urlEncode(const char *src)
{
    int n = 0;
    char c, *s = (char *)src;
    while ((c = *s++))
    {
        if (strchr(invalidChars, c))
        {
            n++;
        }
    }
    String ret;
    ret.reserve(strlen(src) + 2 * n + 1);
    s = (char *)src;
    while ((c = *s++))
    {
        if (strchr(invalidChars, c))
        {
            ret += '%';
            ret += hex_digit(c >> 4);
            ret += hex_digit(c);
        }
        else
            ret += c;
    }
    return ret;
}

void influxdb_onData(void *arg, AsyncClient *client, void *data, size_t len)
{
    ESP_LOGD(TAG, "Influx data received");

    //TODO: We should be checking for a HTTP return code of 204 here

    //Serial.printf("\n data received from %s \n", client->remoteIP().toString().c_str());
    //Serial.write((uint8_t*)data, len);
    //ESP_LOGI(TAG, "Influx reply %s", (uint8_t *)data);
};

void influxdb_onError(void *arg, AsyncClient *client, err_t error)
{
    ESP_LOGE(TAG, "Influx connect error");
    influx_Client = NULL;
    delete client;
};

void influxdb_onDisconnect(void *arg, AsyncClient *client)
{
    ESP_LOGI(TAG, "Influx disconnected");
    influx_Client = NULL;
    delete client;
};

//Called when the TCP stack connects to INFLUXDB server
void influxdb_onConnect(void *arg, AsyncClient *client)
{
    ESP_LOGD(TAG, "Influx connected");

    influx_Client->onError(NULL, NULL);
    influx_Client->onData(&influxdb_onData, influx_Client);
    influx_Client->onDisconnect(&influxdb_onDisconnect, influx_Client);

    if (influx_StartModule > (TotalNumberOfCells() - 1))
    {
        influx_StartModule = 0;
    }

    String poststring;

    uint8_t counter = 0;
    uint8_t i = influx_StartModule;

    while (i < TotalNumberOfCells() && counter < 16)
    {
        //ESP_LOGD(TAG, "Send Influx for module %u", i);
        //Only send valid module data
        if (cmi[i].valid)
        {
            uint8_t bank = i / mysettings.totalNumberOfSeriesModules;
            uint8_t module = i - (bank * mysettings.totalNumberOfSeriesModules);

            //Data in LINE PROTOCOL format https://docs.influxdata.com/influxdb/v2.0/reference/syntax/line-protocol/

            // Example
            //  myMeasurement,tag1=value1,tag2=value2 fieldKey="fieldValue" 1556813561098000000
            poststring = poststring + "cells," + "cell=" + String(bank) + "_" + String(module) + " v=" + String((float)cmi[i].voltagemV / 1000.0, 3) + ",i=" + String(cmi[i].internalTemp) + "i" + ",e=" + String(cmi[i].externalTemp) + "i" + ",b=" + (cmi[i].inBypass ? String("true") : String("false")) + "\n";
        }

        counter++;

        i++;
    }

    //After transmitting this many packets to Influx, store our current state
    //this prevents flooding the ESP controllers wifi stack and potentially causing reboots/fatal exceptions
    influx_StartModule = i;

    String url = String(mysettings.influxdb_serverurl) + String("?org=") + urlEncode(mysettings.influxdb_orgid) + String("&bucket=") + urlEncode(mysettings.influxdb_databasebucket);
    String token = String("Authorization: Token ") + String(mysettings.influxdb_apitoken);

    //TODO: Need to URLEncode these values
    String header = "POST " + url + " HTTP/1.1\r\n" + "Host: " + influx_host + "\r\n" + "Connection: close\r\n" + token + "\r\nContent-Length: " + poststring.length() + "\r\n" + "Content-Type: text/plain\r\n" + "\r\n";

    //SERIAL_DEBUG.println(header.c_str());
    //SERIAL_DEBUG.println(poststring.c_str());

    client->write(header.c_str());
    client->write(poststring.c_str());

    ESP_LOGD(TAG, "Influx data sent");
}

void influx_task_action()
{
    //client already exists, so don't do this again
    if (influx_Client)
    {
        ESP_LOGE(TAG, "Client already exists");
        return;
    }

    if (influx_Client == NULL)
    {
        ESP_LOGD(TAG, "Create new client");

        influx_StartModule = 0;

        String url = String(mysettings.influxdb_serverurl);

        // check for : (http: or https:
        int index = url.indexOf(':');
        if (index < 0)
        {
            ESP_LOGE(TAG, "Failed to parse protocol");
        }
        else
        {
            String _protocol = url.substring(0, index);

            influx_port = 80;

            url.remove(0, (index + 3)); // remove http:// or https://

            index = url.indexOf('/');
            String host = url.substring(0, index);
            url.remove(0, index); // remove host part

            // get port
            index = host.indexOf(':');
            if (index >= 0)
            {
                influx_host = host.substring(0, index); // hostname
                host.remove(0, (index + 1));            // remove hostname + :
                influx_port = host.toInt();             // get port
            }
            else
            {
                influx_host = host;
            }

            ESP_LOGI(TAG, "Influx host %s port %i", influx_host.c_str(), influx_port);

            influx_Client = new AsyncClient();

            if (!influx_Client)
            {
                ESP_LOGE(TAG, "Unable to create Client");
                return;
            }

            influx_Client->onConnect(&influxdb_onConnect, influx_Client);
            influx_Client->onError(&influxdb_onError, influx_Client);
        }
    }

    if (influx_Client != NULL)
    {
        influx_Client->setAckTimeout(2000);
        influx_Client->setRxTimeout(2);

        //Now trigger the connection, and then send the data
        if (!influx_Client->connect(influx_host.c_str(), influx_port))
        {
            ESP_LOGE(TAG, "Influxdb connect fail");
            AsyncClient *client = influx_Client;
            influx_Client = NULL;
            delete client;
        }
    }
}