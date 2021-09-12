
/* esp32-cam camera application jpeg. sensor = ov2640

This file contains:
- main camera application
- esp32 wifi handler
- wifi ssid/password storage in NVS flash memory
- wifi credentials configuration via serial interface on startup.

This software is based on esp-idf version 4.3, 4.4
september 2021, Thomas Krueger, Hofgeismar Germany (all rights reserved)
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/gpio.h"
#include "esp_camera.h"

// the camera config structure to be used with the camera driver
static camera_config_t camera_config =
{
    .pin_pwdn  = 32,
    .pin_reset = -1,
    .pin_xclk = 0,
    .pin_sscb_sda = 26,
    .pin_sscb_scl = 27,

    .pin_d7 = 35,
    .pin_d6 = 34,
    .pin_d5 = 39,
    .pin_d4 = 36,
    .pin_d3 = 21,
    .pin_d2 = 19,
    .pin_d1 = 18,
    .pin_d0 = 5,
    .pin_vsync = 25,
    .pin_href = 23,
    .pin_pclk = 22,

    //XCLK 20MHz(or 10MHz for OV2640 double FPS by setting pll x 4)
    .xclk_freq_hz = 20000000,
//   .ledc_timer = LEDC_TIMER_0,  // not using LED dimming
//   .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG       this is the cams output format!!!!!!!!!!!!
    .frame_size = FRAMESIZE_UXGA,//first init with largest framesize to get biggest framebuffers in camera driver

    .jpeg_quality = 10, //0-63 lower number means higher quality
    .fb_count = 2 //number of framebuffers to use for capturing, if > 1, i2s runs in continuous mode.
};


static const char *TAG = "espcam";

/* wifi_status:
0=not initialized;
1= normal run, we are connected;
2=disconnected due to invalid ssid/pass;
3=disconnected due to connection loss
4=undefined, we issued a connect() and are waiting for an event to happen
*/
uint8_t wifi_status=0;

//protos:
void camserver(void);


void wifi_connect(char *ssid, char *passwd);
int wifi_try(char *ssid, char *passwd);
int wifi_startup(void);


void editlogintab(void);
void strcopy( char *s, char *d, int len);
//serial io
void putcc(char c);
char getcc(void);
void getss(char *buf);
void putss(const char *ps);



void app_main(void)
{
    esp_err_t ret;
// init:
    gpio_set_direction(33, GPIO_MODE_OUTPUT); // Debug Led
    gpio_set_level(33, 0); // turn debug led on during boot

    ret = nvs_flash_init(); //needed for wifi and PHY and stored AP logins
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Hit SPACE to enter +++LoginEdit+++...2secs");
    vTaskDelay(2000/portTICK_PERIOD_MS);
    if (fgetc(stdin) == 0x20) editlogintab();

// get the camera going
    ESP_LOGI(TAG, "Init Camera.........");
    ret=esp_camera_init(&camera_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG,"Boot: camera init failed....Restarting System now...............>>>>>>\n");
        fflush(stdout);
        esp_restart(); // try to resolve by doing a reset
    }

// set framesize to standard VGA
    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, (framesize_t)FRAMESIZE_VGA);


    if (wifi_startup() == 0) //wifi takes a bit with its logging output...
    {
        ESP_LOGE(TAG, "Could not find a known Wifi Network!!!");
        vTaskDelay(10000/portTICK_PERIOD_MS);
        esp_restart(); // keep trying to find a known wifi router by reseting the system
    }

    gpio_set_level(33, 1); // turn debug led off, boot done, we are connected

    camserver(); //this task becomes the webserver for control

    // we shouldnt get here.
}



//event handler wifi new. this function executes in a different task!
void wifi_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    wifi_event_sta_disconnected_t* disconnect = event_data;

    if (event_base == WIFI_EVENT)
    {
        switch(event_id)
        {
        case WIFI_EVENT_STA_START: // we entered STA wifi mode
            esp_wifi_connect(); // start scanning...and connect...or sta_disconnect
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG,"STA_DISCONNECTED,reason:%d\n",disconnect->reason);
            if (wifi_status != 3)
            {
                switch (disconnect->reason)
                {

                case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: // 15
                case WIFI_REASON_NO_AP_FOUND: //201
                //15=invalid password or not present; 201=ssid not found
				// if ssid is 0, we will not get here as wifi will just scan.

                    wifi_status=2; // we disconnected due to invalid ssid or passwd...try another ssid/passwd
                    break;
                default:
                    wifi_status=3; // we were connected but lost connection to AP, giving up reset
                    break;

                }
            }

            if (wifi_status == 3) // lost connection. just reset, any reconnect upsets the lwip stack
            {
            ESP_LOGI(TAG,"-- lost connection, restarting system ...");
                vTaskDelay(10000/portTICK_PERIOD_MS);
                esp_restart(); // we just reset the board as the TCPserver will likely hang. also we can connect to different AP then.

            }

            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        // we got ip
        ESP_LOGI(TAG, "+++CONNECTED+++: got ip:" IPSTR , IP2STR(&((ip_event_got_ip_t*)(event_data))->ip_info.ip));
        wifi_status=1; //online
    }

}

// wifi connect to router with credentials supplied in nvs table
void wifi_connect(char *ssid, char *passwd)
{
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    wifi_config_t wifi_config;
    int ret;

    // from here, we set the given APs credentials and start the connection process.
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    snprintf((char*)wifi_config.sta.ssid, 32, "%s", ssid);
    snprintf((char*)wifi_config.sta.password, 64, "%s", passwd);
    ESP_LOGI(TAG, "...connecting to AP SSID:%s PASSWD:%s",ssid, passwd);

    if (wifi_status) // wifi task is already running, so here we just set the changed ssid/passwd parameters and (re)connect to that AP
    {
        // we assume we are disconnected at this stage!
        if (esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)  != ESP_OK) // set ssid and passwd of wanted AccessPoint to connect to
        {
            ret=6;
            goto wifierr;
        }

        if (esp_wifi_connect() != ESP_OK) // connect with a new ssid/passwd
        {
            ret=7;
            goto wifierr;
        }
        wifi_status=4; //undefined

        return;
    }

// do first time wifi init stuff to start the wifi task:


// Setup the Wifi eventloop and define 2 events for "got_ip" and "other_stuff"
    if (esp_event_loop_create_default() != ESP_OK) // start the event loop task
    {
        ret=1;
        goto wifierr;
    }
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_handler, NULL, &instance_got_ip);

    esp_netif_init(); // init lwIP/TCPIP stack. from 4.1 its esp_netif_init().
    esp_netif_create_default_wifi_sta(); // create default station context

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) // WiFi control structure, RX/TX buffer, WiFi NVS structure etc, start WiFi task.
    {
        ret=2;
        goto wifierr;
    }
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) // set Station mode(clientmode)
    {
        ret=3;
        goto wifierr;
    }

    if (esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)  != ESP_OK) // set ssid and passwd of wanted AccessPoint to connect to
    {
        ret=4;
        goto wifierr;
    }
    if (esp_wifi_start() != ESP_OK) // startup the Wifi process..ie. it starts scanning and will connect
    {
        ret=5;
        goto wifierr;
    }
    esp_wifi_set_ps(WIFI_PS_NONE); // disable powersave. this will make ping go much faster: DEFAULT POWERMODE IS.WIFI_PS_MIN_MODEM
    wifi_status=4; //undefined. event_handler will set the correct status
    return;
wifierr:
    ESP_LOGE(TAG, "Wifi connect error : %d",ret);
}

/* try to connect to an AccessPoint with given credentials
exit:
0=fail;
1=OK, we are connected
*/
int wifi_try(char *ssid, char *passwd)
{
    int count;

    count=100; // 10 sec timeout

    wifi_connect(ssid,passwd);
    while (count--) //try for 10 secs
    {
        vTaskDelay(100/portTICK_PERIOD_MS);
        switch (wifi_status) // did event_handler set new status?
        {
        case 1:
            return 1; //online
        case 2:
        case 3:
            return 0; // disconnected
        default:
            break;
        }
    }
    return 0;
}

/* wifi_startup
This is the main function to connect to a wifi accesspoint.

uses WIFI login credentials saved in NVS:

The NVS-interface is similar to operating files on a computer:
open the file (nvs_open), write a file (nvs_set_xxx), save document(nvs_commit), close the file (nvs_close)
open the file (nvs_open), reading the file (nvs_get_xxx), close the file (nvs_close)

This function reads a table of ssid_name/password pairs from nvs.
"lastindex"  will supply lastused logindata tableindex, as its most likely to be used again.
else or on fail, it will interate through all stored name/passwd entrys until successfull wifi connect happens, and store its index in "lastindex".

if all fail,  it will remain in a loop, with onboard statusLED on (indicating NoWifiConnect).
The user must then update the logintable via serial interface.

exit: 0=fail,1=ok, wifi is up
*/
#define FILENAME "LoginData"
#define DATANAME "LoginTab"
#define nentrys 10
#define nlength 32
//tableformat: tab[number of strings][length of each string]
struct logintable
{
    int lastindex; // range: 0 to n-1
    // 10 entrys of name/passwd Login-pairs of max nlength each. (last char is 0-terminator)
    char name[nentrys][nlength]; //array of 10 names at nlength
    char pass[nentrys][nlength]; // array of 10 passwds at nlength
};

int wifi_startup(void)
{
    nvs_handle filehandle=0;
    size_t bytes = 0;
    esp_err_t ret;
    void *pmem=NULL;
    struct logintable *ptab;
    int i;

    // Open
    ret = nvs_open(FILENAME, NVS_READWRITE, &filehandle); // open a file in nvs, existing or not.
    if (ret != ESP_OK) 		goto reterror; // this should not happen as we allready inited nvs in main.

    pmem = malloc(sizeof(struct logintable));

    if (!pmem)
    {
        ret = -1;
        goto reterror; // malloc failed
    }

    ret = nvs_get_blob(filehandle, DATANAME, NULL, &bytes); // just get the size of the stored filedata. This fails if the record is not found

    if ((bytes != sizeof(struct logintable))||(ret !=ESP_OK)) // if DATANAME hasnt been stored before(0) or has different size
    {
        memset(pmem,0,sizeof(struct logintable)); // clear memory
        bytes = sizeof(struct logintable);
        ret = nvs_set_blob(filehandle, DATANAME, pmem, bytes); // save empty table
        if (ret != ESP_OK) goto reterror;
    }
    else // read in the table
    {
        ret = nvs_get_blob(filehandle, DATANAME, pmem, &bytes);
        if (ret != ESP_OK) goto reterror;
    }
    nvs_close(filehandle); // we need to close it, as wifi will also uses NVS
    filehandle = 0;

    // now the table is in memory (or empty), lets check if last used AP credentials work.
    ptab = pmem;
    if (ptab->lastindex < nentrys)
    {
        i = ptab->lastindex;
        if (ptab->name[i][0]) //...and name exists!
        {
            ret = wifi_try(&ptab->name[i][0], &ptab->pass[i][0]); // connect to last known AP
            if (ret) goto retok; // success
        }
    }

    // now walk through the table and test each entry for valid wifi connect
    for (i=0; i<nentrys; i++)
    {
        if (ptab->name[i][0]) // if first char is not zero, so there is a name in it
        {
            ret = wifi_try(&ptab->name[i][0], &ptab->pass[i][0]);
            if (ret) goto retok; // success
        }
    }

    ret = 0;// we could not connect the wifi from the table and give up here!

    // all entrys failed, now try the name/pass given by the configuration or sourcecode, if there

    goto doreturn;


retok:
//check, lastindex is different to i, ie. a different login was used as last time. update NVS then.
    if (ptab->lastindex != i)
    {
        ptab->lastindex = i;
        ret = nvs_open(FILENAME, NVS_READWRITE, &filehandle);
        if (ret != ESP_OK) 		goto reterror;
        bytes = sizeof(struct logintable);
        ret = nvs_set_blob(filehandle, DATANAME, pmem, bytes); // save empty table
        if (ret != ESP_OK) goto reterror;
        ESP_LOGI(TAG, "updated lastindex!!");
    }

    ret = 1; //OK
    goto doreturn;
reterror:
    if (ret != ESP_OK) ESP_LOGE(TAG, "Fatal error occurred in wifi_startup(): %d ; %s",ret, esp_err_to_name(ret));
    ret = 0; //error
doreturn:
    if (pmem) free(pmem);
    if (filehandle) nvs_close(filehandle);
    return ret;

}



void putcc(char c)
{
    fputc(c, stdout);
}

char getcc(void)
{
    char c;

    while (1)
    {
        vTaskDelay(10/portTICK_PERIOD_MS); //min 10 for watchdog not to trigger!
        c = fgetc(stdin);
        if (c!=0xFF) break;
    }
    putcc(c);
    return c;
}

void getss(char *buf)
{
    char *ps, c;
    ps=buf;
    while (1)
    {
        c=getcc();
        if (( c == 0x0d)||(c==0x0a)) // CR LF
            break;
        *ps++=c;
    }
    *ps=0;
    putcc(0x0d);
    putcc(0x0a);
}

void putss(const char *ps)
{
    while (*ps)
    {
        putcc(*ps++);
    }
}

// string copy where len is the total fieldlength including 0.
void strcopy( char *s, char *d, int len)
{
    len--;
    while (len--&&*s) *d++=*s++;
    *d=0;
}


const char *menue="***Menue***\n"
                  "l-list entrys\n"
                  "e<num>-edit entry\n"
                  "d<num>-delete entry\n"
                  "s-save changes\n"
                  "q-quit\n"
                  ;
const char *prompt="\nespcam>";


/* edit the NVS table for ssid/passwd's
This is called on startup if a SPACE kay was pressed after Reset.
*/
void editlogintab(void)
{
    char buf[100],c;

    nvs_handle filehandle=0;
    size_t bytes = 0;
    esp_err_t ret;
    void *pmem=NULL;
    struct logintable *ptab;
    int i,flag=0;

    // Open
    ret = nvs_open(FILENAME, NVS_READWRITE, &filehandle); // open a file in nvs, existing or not.
    if (ret != ESP_OK) 		ESP_LOGI(TAG, "nvs_open failed!!!");

    pmem = malloc(sizeof(struct logintable));

    if (!pmem)
    {
        ESP_LOGI(TAG, "malloc failed!!!");
    }

    ret = nvs_get_blob(filehandle, DATANAME, NULL, &bytes); // just get the size of the stored filedata. This fails if the record is not found

    if ((bytes != sizeof(struct logintable))||(ret !=ESP_OK)) // if DATANAME hasnt been stored before(0) or has different size
    {
        memset(pmem,0,sizeof(struct logintable)); // clear memory
        bytes = sizeof(struct logintable);
        ret = nvs_set_blob(filehandle, DATANAME, pmem, bytes); // save empty table
        if (ret != ESP_OK) ESP_LOGI(TAG, "nvs_set_blob failed!!!");
    }
    else // read in the table
    {
        ret = nvs_get_blob(filehandle, DATANAME, pmem, &bytes);
        if (ret != ESP_OK) 	ESP_LOGI(TAG, "nvs_get_blob failed!!!");
    }
    ptab = pmem;

    putss(menue);
    putss(prompt);

    while(1) // menue loop
    {
        c = getcc();
        switch(c)
        {
        case 'l': //list
            putss("\nList Logintable:\n");
            sprintf(buf,"LastUsed Entry: %d\n",ptab->lastindex);
            putss(buf);
            for (i=0; i<nentrys; i++)
            {
                sprintf(buf,"%d: %s   %s\n",i,&ptab->name[i][0], &ptab->pass[i][0]);
                putss(buf);
            }
            break;

        case 'e': //edit
            putss("\nEdit:Enter entry (0 to 9):");
            c = getcc();
            c -= 0x30;
            if (c>9) break;
            i=c;
            putss("\nEnter SSID:");
            getss(buf);
            strcopy(buf,&ptab->name[i][0],nlength);
            putss("Enter PASSWD:");
            getss(buf);
            strcopy(buf,&ptab->pass[i][0],nlength);
            ptab->lastindex = 0xff; // invalidate after edit
            break;

        case 'd': //delete
            putss("\nDelete:Enter entry (0 to 9):");
            c = getcc();
            c -= 0x30;
            if (c>9) break;
            i=c;
            ptab->name[i][0]=0;
            ptab->pass[i][0]=0;
            ptab->lastindex = 0xff; // invalidate after edit
            putss("Deleted");
            break;

        case 's': //save
            bytes = sizeof(struct logintable);
            ret = nvs_set_blob(filehandle, DATANAME, pmem, bytes); // save changed table
            if (ret != ESP_OK) ESP_LOGI(TAG, "nvs_set_blob failed!!!");
            putss("\nSaved!");
            break;

        case 'q': //quit
            flag=1;
            break;

        default:
            putss(menue);
        }
        putss(prompt);


        if (flag) break;
    }

    if (pmem) free(pmem);
    if (filehandle) nvs_close(filehandle);

}



