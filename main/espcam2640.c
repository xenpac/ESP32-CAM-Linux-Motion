
/* esp32-cam camera application.

This file contains:
- main camera application
- esp32 wifi handler
- wifi ssid/password storage in NVS flash memory
- wifi credentials configuration via serial interface on startup.

This software is based on esp-idf version 3.3.1, the long term support version until 2022....i love it:-)
So please use the LTS version! to be compatible!
This uses the make utility! old style.

esp32 flash memmap:
0x1000 = 2ndstage bootloader
0x8000 = partitiontable
0x10000= application

esptool.py commandline for flashing binarys:
python esp-idf/components/esptool_py/esptool/esptool.py
--chip esp32 --port "/dev/ttyUSB0" --baud 921600 --before "default_reset" --after "hard_reset"
write_flash -z --flash_mode "dio" --flash_freq "80m" --flash_size detect
0x1000 espcam2640/build/bootloader/bootloader.bin
0x10000 espcam2640/build/espcam.bin
0x8000 espcam2640/build/partitions.bin


september 2020, Thomas Krueger, Hofgeismar Germany (all rights reserved)
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "esp_eth.h"
#include "tcpip_adapter.h"
#include "eth_phy/phy_lan8720.h"

#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"


//camera pin config esp32-cam
//esp32-cam PIN Map
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HSYNC   23
#define CAM_PIN_PCLK    22

// the camera config structure to be used with the camera driver
static camera_config_t camera_config =
{
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HSYNC,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz(or 10MHz for OV2640 double FPS by setting pll x 4)
    .xclk_freq_hz = 20000000,
//   .ledc_timer = LEDC_TIMER_0,  // not using LED dimming
//   .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG       this is the output format!!!!!!!!!!!!
    .frame_size = FRAMESIZE_UXGA,//first init with largest framesize to get biggest framebuffers in camera driver

    .jpeg_quality = 10, //0-63 lower number means higher quality
    .fb_count = 2 //number of framebuffers to use for capturing, if > 1, i2s runs in continuous mode.
};

// you may specify your APs credential here (or from sysconfig) being hardcoded !! ?? if you want that.Otherwise just edit logintable via serial on startup!
//#define CONFIG_ESP_WIFI_SSID "dummy"
//#define CONFIG_ESP_WIFI_PASSWORD "bunny"

static const char *TAG = "espcam";
uint8_t wifi_retrys = 0;
/* wifi_status:
0=not initialized;
1= normal run, we are connected;
2=disconnected due to invalid ssid/pass;
3=disconnected to to connection loss
4=undefined, we issued a connect() and are waiting for an event to happen
*/
uint8_t wifi_status=0;


//protos:
void camserver(void);

esp_err_t wifi_handler(void *ctx, system_event_t *event);
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
        esp_restart();
    }

// set framesize to standard VGA
    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, (framesize_t)FRAMESIZE_VGA);

//    vTaskDelay(1000/portTICK_PERIOD_MS);  // wait a little before starting wifi to get the debug print done

    if (wifi_startup() == 0) //wifi takes a bit with its logging output...
    {
        ESP_LOGE(TAG, "Could not find a known Wifi Network!!!");
        vTaskDelay(10000/portTICK_PERIOD_MS);
        esp_restart(); // keep trying by reseting the system
    }
    gpio_set_level(33, 1); // turn debug led off, boot done

    camserver(); //this task becomes the webserver for control

    // we shouldnt get here.
}



//event handler wifi. this function executes in a different task!
esp_err_t wifi_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id)
    {
    case SYSTEM_EVENT_STA_START: // we entered STA wifi mode
        esp_wifi_connect(); // start scanning...and connect...or sta_disconnect
        break;
    case SYSTEM_EVENT_STA_GOT_IP:  // if connect, we got ip
        ESP_LOGI(TAG, "+++CONNECTED+++: got ip:%s",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        wifi_retrys = 0;
        wifi_status=1; //online
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG,"STA_DISCONNECTED,reason:%d\n", event->event_info.disconnected.reason);

        if (wifi_status != 3)
        {
            switch (event->event_info.disconnected.reason)
            {
            case 201:	//15=invalid password or not present; 201=ssid not found
            case 15:	// if ssid is 0, we will not get here as wifi will just scan.
                wifi_status=2; // we disconnected due to invalid ssid or passwd
                break;
            default:
                wifi_status=3; // we lost connection to AP, lets retry. or esp_wifi_disconnect() was called
                break;

            }
        }

        if (wifi_status == 3) // try to reconnect 5 times if we lost connection, maybe out of reach
        {
            if (wifi_retrys < 5)
            {
                esp_wifi_connect();
                wifi_retrys++;
                ESP_LOGI(TAG,"trying to reconnect...");
            }
            else
            {
                vTaskDelay(10000/portTICK_PERIOD_MS);
                esp_restart(); // we just reset the board as the TCPserver will likely hang. also we can connect to different AP then.
            }

        }

        break;
    default:
        //ESP_LOGI(TAG,"got unknown wifi event:%d!!",event->event_id); // some events are happening, nothing to worry.
        break;
    }
    return ESP_OK;
}

// wifi connect to router with credentials supplied in config
void wifi_connect(char *ssid, char *passwd)
{
    wifi_config_t wifi_config;
    int ret;

    // from here, we set the APs credentials and start the connection process.
    // to change ssid/passwd and reconnect
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    snprintf((char*)wifi_config.sta.ssid, 32, "%s", ssid);
    snprintf((char*)wifi_config.sta.password, 64, "%s", passwd);
    ESP_LOGI(TAG, "...connecting to AP SSID:%s PASSWD:%s",ssid, passwd);

    if (wifi_status) // wifi is already running, so here we just set the changed ssid/passwd parameters and (re)connect to that AP
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

// do first time wifi init stuff:

    if (esp_event_loop_init(wifi_handler, NULL) != ESP_OK) // start the event loop task
    {
        ret=1;
        goto wifierr;
    }

    tcpip_adapter_init(); // init lwIP/TCPIP stack. from 4.1 its esp_netif_init().

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
    wifi_status=4; //undefined
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
    while (count--) //try for 1 secs
    {
        vTaskDelay(100/portTICK_PERIOD_MS);
        switch (wifi_status)
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

/* WIFI login credentials saved in NVS:

The NVS-interface is similar to operating files on a computer:
open the file (nvs_open), write a file (nvs_set_xxx), save document(nvs_commit), close the file (nvs_close)
open the file (nvs_open), reading the file (nvs_get_xxx), close the file (nvs_close)

This function reads a table of ssid_name/password pairs from nvs.
"lastindex"  will supply its logindata to wifi_connect, as its most likely to be used again.
else or on fail, it will interate through all stored name/passwd entrys until successfull wifi connect happens, and store its index in "lastindex".

if all fail, it will supply the name/passwd from the config data.
if that fails too, it will remain in a loop, blinking the onboard statusLED (indicating NoWifiConnect).
The user must then update the logintable via serial interface.

exit: 0=fail,1=ok, wifi is up
on fail, we just run without wifi.?!
*/
#define FILENAME "LoginData"
#define DATANAME "LoginTab"
#define nentrys 10
#define nlength 32
//tableformat: tab[number of strings][length of each string]
struct logintable
{
    int lastindex; // range: 0 to n-1
    // 10 entrys of name/passwd Login-pairs of max length 19 each. (last char is 0-terminator)
    char name[nentrys][nlength]; //array of 10 names at 20chars length
    char pass[nentrys][nlength]; // array of 10 passwds at 20chars length
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

#ifdef 	CONFIG_ESP_WIFI_SSID
    if (strlen(CONFIG_ESP_WIFI_SSID))
    {
        ESP_LOGI(TAG, "trying to connect to hardcoded ssid");

        if (wifi_try(CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD))
            ret = 1;
    }
#endif

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



