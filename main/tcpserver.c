/* tcp server for jpeg camera application

esp32-cam TCP server using lwip directly via BSD Socket API.

OV2640 using jpeg only!

This file contains:
- camcontrol webserver
  - the onboard LED to be used as flashlight (snapshots) or streaming light.
    This LED draws a higher current and gets quite hot.(no dimming is used)
  - nightmode
  - Status/Framerate display
  - Reset option processor

- camstreaming webserver
  - reduced framerate to balance network load on multible camera usage.(linux motion)

NOTES: esp32-cam 5V supply should be increased to min. 5.4V (upto 6V) for stable operation.

september 2020, Thomas Krueger, Hofgeismar Germany (all rights reserved)
*/

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "esp_wifi.h"
#include <lwip/netdb.h>
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_log.h"


//protos:
void streamtask(void *param);
int tcpserver(int port);
int http_response(int port, char *req, int connection);
int http_stream(int connection);
int get_frame(uint8_t **buf, size_t *len);
static uint16_t set_register(char *uri);
static int set_control(char *uri);
static int get_camstatus(void);
static int get_status(char *uri);
void stream_speed(int full);
void night_mode(int on);

//globals:
camera_fb_t *fb=NULL;					 
char iobuf[1024]; // for control processing
int flashlight, streamlight, streamspeed, nightmode, IsStreaming;

//framerate stuff
TimerHandle_t tmr;
void timerCallBack( TimerHandle_t xTimer );

int NetFPS,HwFPS, I2sFPS, NetFrameCnt, resetflag=0;
int HwFrameCnt, I2sFrameCnt,DMAerrors,JPGerrors;

int uptime; // in seconds
int rssi;

static const char *TAG = "tcpserver";


/* server main function.
starts a task for the streamserver on port 81
then goes into control server on port 80 for camera control
*/
void camserver(void)
{
    TaskHandle_t servertask;

    //NOTE: the RTOS tick is configured to 10ms. so if you set timerperiod to 1, then its actually 10ms!!
    tmr = xTimerCreate("SecTimer", 100, pdTRUE, (void *)0, &timerCallBack); // create a 1Sec software timer
    xTimerStart(tmr,0);  // and start it

    flashlight=streamlight=IsStreaming=streamspeed=nightmode=0;

// init LED (only for AI-Thinker board!!)
    gpio_set_direction(4, GPIO_MODE_OUTPUT); // set portpin to output. HighPower LED

    // give visual indication that a successfull reset occured by turning LED briefly on
//    gpio_set_level(4, 1); // turn led on
//    vTaskDelay(400/portTICK_PERIOD_MS);  // wait a little to get camera exposure settle to new light conditions
    gpio_set_level(4, 0); // turn led off

// set starting streamspeed to slow=9fps(=1Mbit-stream at 640*480) for motion to not overload the wifi network	with 4 cameras
    stream_speed(0);

// start streaming task on port 81
    if (!xTaskCreatePinnedToCore(&streamtask, "streamserver", 8192, NULL, tskIDLE_PRIORITY+5, &servertask, 1))
    {
        ESP_LOGE(TAG, "***Failed to create stream servertask task");
    }

    tcpserver(80); // this task becomes the server on port 80 to serve camera controls and stills

// we should never get here!
    xTimerDelete( tmr,0 );

}

void streamtask(void *param)
{
    tcpserver(81);
}

//  tcp webserver. This may be a task!
int tcpserver(int port)
{
    char request[400];
    int serverSocket, clientConn, ret,cnt=0;
    //setup the socket address struct:
    struct sockaddr_in IpAddress;  // this is an overlay for the struct sockaddr, that eases the portnumber entry.ie. overlays char sa_data[14] with WORD port, ULONG address
    IpAddress.sin_family = AF_INET;
    IpAddress.sin_port = htons(port); // the port to listen on   !!
    IpAddress.sin_addr.s_addr = INADDR_ANY;// INADDR_ANY, server gets IP of the machine its running on.(see bind)
    socklen_t socklen = sizeof(IpAddress);

    // open internet socket/endpoint for HTTP communication. return file handle or -1=error
    serverSocket = socket(
                       AF_INET,      // Domain: IPv4 Internet protocols
                       SOCK_STREAM,  // Communication-Type:  SOCK_STREAM=TCP; SOCK_DGRAM=UDP
                       IPPROTO_TCP   // select TCP.   (was 0: Protocol: 0=IP,internet protocol, pseudo protocol number.TCP and UDP)
                   );
    if (serverSocket < 0)
    {
        ESP_LOGE(TAG,"\nsocket failed");
        return -1;
    }


    // assign a specific internet address and port to the socket using sockaddr-struct from above. return 0=OK, -1=error
    // normally the local loopback address is assigned(0.0.0.0).
    ret=bind(serverSocket, (struct sockaddr *) &IpAddress, socklen );
    if (ret)
    {
        ESP_LOGE(TAG,"\nbind failed");
        return -1;
    }
    // start listening on the socket. returns 0=OK, -1=error
    // The second parameter sets the queue_len for incoming requests.ie. MaxRequests.
    ret = listen(serverSocket, 5);
    if (ret)
    {
        ESP_LOGE(TAG,"\nlisten failed");
        return -1;
    }


    ESP_LOGI(TAG,"Server started on:%s:%u    running on CPUCore:%d", inet_ntoa(IpAddress.sin_addr),ntohs(IpAddress.sin_port),xPortGetCoreID() );

    // wait for connection. We only support 5 connection requests waiting at a time. See listen() above
    // while there are connection requests in the input queue of the socket, process then.
    while(1)
    {
        // wait forever for next tcp connection request from the input queue.
        clientConn = accept(serverSocket, (struct sockaddr *) &IpAddress, &socklen); //this blocks !!
        //printf( "Client connect from: %s:%u\n", inet_ntoa(IpAddress.sin_addr),ntohs(IpAddress.sin_port) );

        // connection is established. loop until closed. So we only allow one connection at a time!
        while (1)
        {
            cnt++;
            // use also poll or select to monitor connection!
            ret = read(clientConn,request,sizeof(request)); //wait for new data on the connection. This blocks!!
            if (ret <= 0)
            {
                //printf("read failed!\n");
                break; // connection lost.  a 0 indicates an orderly disconnect by client; -1 some error occured.
            }
            request[ret]=0; // invalidate last request string
            //printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Got:\n %sR-EOT\n",request);

            // process response here.....
            ret=http_response(port, request, clientConn);
            //printf("End of Transaction %d <<<<<<<<<<<<<<<<<<<<<<<<\n",cnt);

            if (!ret) break; //close

        }

        //printf("Connection closed\n");
        close(clientConn); // close current tcp connection

    } // endwhile

    return 0;
}

/*
Request-Line = Method SPACE Request-URI SPACE HTTP-Version CRLF
we only support GET requests!
The request URI contains options on which item is requested!
HTTP-Version: always HTTP1.1.

entry: the complete request string
This routine builds and sends the response!
exit: 1= keep connection; 0=drop connection!
*/
const char *resp_index="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nContent-Encoding: gzip\r\n\r\n";
const char *resp_basic="HTTP/1.1 %s\r\n\r\n";
// changed to jpg as we only have jpeg.
const char *resp_attach="HTTP/1.1 200 OK\r\nContent-Disposition: attachment; filename=\"frame.jpg\"\r\nContent-Length: %d\r\n\r\n";
const char *resp_capture="HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\nContent-Disposition: inline; filename=capture.jpg\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *resp_status="HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *resp_control="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nAccess-Control-Allow-Origin: *\r\n\r\n";

int http_response(int port, char *req, int connection)
{
    // the webpage source is included in the program! This will get the start/end address ..and the length
    extern const unsigned char index_ov2640_html_gz_start[] asm("_binary_index_ov2640_html_gz_start");
    extern const unsigned char index_ov2640_html_gz_end[] asm("_binary_index_ov2640_html_gz_end");
    int indexlength = index_ov2640_html_gz_end - index_ov2640_html_gz_start;
    uint8_t *pb;
    size_t len;
    char response[1024];

    char request[10], uri[100];
    int ret,keepalive=1;
    int more=0;
    uint16_t regval;
    //printf("\n\nLength:%d\n",strlen(req));
    // check for GET: "GET / HTTP/1.1CRLF"
    ret=sscanf(req,"%s %s ",request,uri);
    //printf("req:%s uri:%s ret:%d\n",request,uri,ret);
    if (ret != 2) // some strange request
    {
        sprintf(response,resp_basic,"400 Bad Request");
        goto sendresponse;
    }

    if (strcmp(request,"GET")) // not a GET request
    {
        sprintf(response,resp_basic,"501 Not Implemented");
        goto sendresponse;
    }

    if (port == 80) // control port
    {
        //we are now at GET
        if (!strcmp(uri,"/")||!strcmp(uri,"/index.html")) // request for index.html
        {
            // send the webpage
            sprintf(response,resp_index,indexlength);
            pb=(uint8_t*)index_ov2640_html_gz_start;
            len=indexlength;
            goto sendmore;
        }


        // send status
        if (!strcmp(uri,"/status"))
        {
            if (!get_camstatus()) iobuf[0]=0;
            sprintf(response,resp_status,strlen(iobuf));
            strcat(response,iobuf);
            goto sendresponse;
        }


        // set control
        if (!strncmp(uri,"/control",8))
        {
            ret = set_control(uri);
//        if (ret != 1) goto send404;  webpage freezes if 404 is returned, so dont do it
            sprintf(response,resp_control,0);
            goto sendresponse;
        }

        // Set/Get register value
        if ( !strncmp(uri,"/reg",4) ) // set a register
        {
            regval=set_register(uri);
            sprintf(response,resp_control,0);
            goto sendresponse;

        }

        if ( !strncmp(uri,"/greg",5) ) // get a register
        {
            regval=set_register(uri);
            sprintf(iobuf,"%u",regval);
            sprintf(response,resp_status,strlen(iobuf));
            strcat(response,iobuf);
            goto sendresponse;
        }

        if ( !strncmp(uri,"/getstatus",10) ) // getstatus from server
        {
            ret = get_status(uri);
            sprintf(response,resp_control,strlen(iobuf));
            strcat(response,iobuf);
            goto sendresponse;
        }


        // download raw image!! usually yuv422 like on ov7670, but jpg on ov2640.
        if (!strcmp(uri,"/download"))
        {
            if (!IsStreaming) //if currently streaming 0 Bytes will be downloaded!
            {
                ESP_LOGI(TAG,"Downloading full cam-img as frame.jpg");
                if (flashlight)
                {
                    gpio_set_level(4, 1); // turn led on
                    vTaskDelay(400/portTICK_PERIOD_MS);  // wait a little to get camera exposure settle to new light conditions
                }
				get_frame(&pb,&len); // skip previous frame, it contains old light settings
                if (!get_frame(&pb,&len)) len=0; // get raw image
                gpio_set_level(4, 0); // turn led off
            }
            else
                len = 0;
            sprintf(response,resp_attach,len);
            goto sendmore;

        }

        // capture image!! only if not streaming, as the camera driver gets confused when calling get_frame in 2 tasks at the same time!
        if (!strncmp(uri,"/capture",8))
        {
            if (!IsStreaming)
            {
                ESP_LOGI(TAG,"Get Still");

                if (flashlight)
                {
                    gpio_set_level(4, 1); // turn led on
                    vTaskDelay(400/portTICK_PERIOD_MS);  // wait a little to get camera exposure settle to new light conditions
                }
				get_frame(&pb,&len); // skip previous frame, it contains old light settings
                ret=get_frame(&pb,&len);

                gpio_set_level(4, 0); // turn led off
                if (!ret) len=0;
            }
            else len=0;
            // printf("--pbuf:0x%08x len:%d\n",(uint32_t)pb,len);
            sprintf(response,resp_capture,len);

            goto sendmore;


        }


    } // endif control port 80

// this if we are the streaming server!
    if (port == 81)
    {
        // http stream
        if (!strncmp(uri,"/stream",7))
            return(http_stream(connection));
    }

//default, nothing has catched: send 404 not found/supported----this upsets the client as it waits forever,blocks other controls ...maybe just send http ok??!!
// so we just send a dummy response. The html page does not support catching the error codes!
    ESP_LOGE(TAG,"Unknown GET request: %s",uri);
//    sprintf(response,resp_basic,"404 not found");
    sprintf(response,resp_control,0); // dummy OK

    goto sendresponse;

sendmore:
    more=1; // send data also
    keepalive=1; // keep connection
sendresponse:
    //printf(">>>>send response:\n%sT-EOT\n",response);
    if (more)
    {
        ret = MSG_MORE;
    }
    else ret = 0;

    // send the response text
    ret=send(connection, response, strlen(response),ret);// this blocks until data is sent. ret contains optional MSG_MORE flag to delay sending
    if (ret <= 0)
        return 0; // connection closed. broken connection
    if (ret != strlen(response)) ESP_LOGE(TAG,"send1, not all bytes sent:%d",ret);
    if (more)
    {
        //printf("sending data...\n");
        ret = send(connection, pb, len, 0);// this blocks until data is sent
        if (ret <= 0) // connection closed. broken connection
            return 0;
        if (ret != len) ESP_LOGE(TAG,"send2, not all bytes sent:%d",ret);

    }

// check if reset command was given:
    if (resetflag) 	esp_restart();  // we die from here

    return keepalive;	//in http1.1, always keep connection alive, unless someone hangs up. so always return 1.
}


const char *resp_stream="HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=ESP32CAM_ServerPush\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *frame_header ="\r\n--ESP32CAM_ServerPush\r\nContent-Type:image/jpeg\r\nContent-Length:%d\r\n\r\n";
//const char *frame_header ="--ESP32CAM_ServerPush\r\n\r\n";
/* keep a streaming video until remote client hangs up
The content type multipart/x-mixed-replace was developed as part of a technology to emulate server push and streaming over HTTP.
This implements "The Multipart Content-Type" over HTTP Protocol using boundary-identifier.
This is not to be confused with chunked!!
The identifier can be any string you like;) must stay the same of corse.

returns 0 = close connection
*/
int http_stream(int connection)
{
    uint8_t *pb;
    size_t len;
    char response[512];

    int ret;

    ESP_LOGI(TAG,"Stream Start....");

    ret=send(connection, resp_stream, strlen(resp_stream),0);
    if (ret <= 0) return 0; //client hanged up

    IsStreaming=1;


    while (1)
    {
        if (streamlight) gpio_set_level(4, 1); // turn led on
        else gpio_set_level(4, 0); // turn led off

        ret=get_frame(&pb,&len);
        if (!ret) // error message is printed in driver if fails
        {
            // something went wrong in the camera/driver, just reset the thing trying to resolve it.
            ESP_LOGE(TAG,"Frame Capture failed....Restarting System now...............>>>>>>\n");
            fflush(stdout);
            esp_restart();
            break;
        }

        sprintf(response,frame_header,len);
        ret=send(connection, response, strlen(response),0);
        if (ret <= 0) //connection closed by client
        {

            break;
        }
        ret = send(connection, pb, len, 0);// this blocks until data is sent
        NetFrameCnt++; // calc FPS
        if (ret <= 0) break; //connection closed by client
        else if (ret != len) ESP_LOGE(TAG,"sendjpg, not all bytes sent:%d errno:%d",ret,errno);
    }

    IsStreaming=0;
    gpio_set_level(4, 0); // turn led off
    ESP_LOGI(TAG,"....Stream Stop");

    return 0; // hangup
}


/*
get a frame from the camera
uses global pointer to  fb_struct (camera framebuffer)
entry:
- address of pointer to receive the resulting framebuffer address.
- address of len variable receiving the length of framebuffer data.
exit:
1=OK, 0=capture  failed.
The buffer with data and the length is returned to caller using pointers!!
*/
int get_frame(uint8_t **buf, size_t *len)
{
    esp_camera_fb_return(fb); //release a possible last used framebuffer. we should work with at least 2 framebuffers!

											   
    fb = esp_camera_fb_get(); // get a new framebuffer with current picture data
    if (!fb)
    {
        ESP_LOGE(TAG,"CamCapture failed");
        return 0;
    }
    else
    {
        // save data from the framebuffer
        *buf=fb->buf;
        *len=fb->len;
    }

    return 1;
}


/* process a register get/set command from client:
ov2640 has byte registers!
If you click a control button on the webpage, it will send the changed control value to us.
We need to forward it to the camera.
entry:
- uri-string containing the json formatted request: fe. URI: "/reg?reg=12296&mask=255&val=2" or "/greg?reg=12296&mask=255"
exit:
  the value returned or 0=OK. no error checking!!!
*/
static uint16_t set_register(char *uri)
{
    char *pfunction, *preg, *pvalue;
    int value=-1,reg,mask,setflag=0;
    sensor_t *s;
    // get json parameters from uri
    uri++; //skip leading /
    pfunction=strtok(uri, "?"); //returns "reg" or "greg"
    strtok(NULL, "=");//returns: "reg"
    preg=strtok(NULL, "&");//  returns: "12296"
    reg=atoi(preg);
    strtok(NULL, "="); // returns: "mask",
    preg=strtok(NULL, "&");//  returns: "255"
    mask=atoi(preg);

    if (!strcmp(pfunction,"reg"))
    {
        setflag=1; // we are setting a register
        strtok(NULL, "="); // returns: "val"
        pvalue=strtok(NULL, "="); // returns: "2". didnt find '=' but returns the last string
        value=atoi(pvalue);
    }
    ESP_LOGI(TAG, "Register: %s reg=0x%02x mask=0x%02x value:0x%02x", pfunction, reg, mask, value);

    s = esp_camera_sensor_get(); // get the cameras function list

    if (setflag)
    {
        if (s->set_reg) return(s->set_reg(s,reg,mask, value)); // set register with value. mask always 0xff
    }
    else
    {
        if (s->get_reg) return(s->get_reg(s,reg,mask)); // get register with value. mask always 0xff
    }
    ESP_LOGI(TAG, "register function not supported!");
    return 0;
}


/* process a set control command from client:
This usually is used to set some parameter in the camera, or to set some functionality on the server side.
entry:
- uri-string containing the json formatted request: fe. URI: /control?var=streamlight&val=0
exit:
- 1 = OK
- 0 = fail, control not found.
-1 = set function failed.
*/
static int set_control(char *uri)
{
    char *variable, *ps;
    int value,ret;
    sensor_t *s;
    int  (*func)(sensor_t *sensor, int val)=NULL;
    // get json parameters from uri
    strtok(uri, "=&"); //goto first & or = .tell strtok to use string uri. returns: "/control?var"
    variable=strtok(NULL, "=&");// we are now at '='.  from last = find next = or & and put a /0 there. returns: "streamlight"
    strtok(NULL, "="); // returns: "val", skip it
    ps=strtok(NULL, "="); // returns: "0". didnt find '=' but returns the last string being the value
    value=atoi(ps);
    ESP_LOGI(TAG, "Control: %s = %d", variable, value);

//first check for internal commands for the server:
    ret=0;
    if (!strcmp(variable, "flashlight"))
    {
        flashlight=value;
        ret=1;
    }
    else if (!strcmp(variable, "streamlight"))
    {
        streamlight=value;
        ret=1;
    }
    else if (!strcmp(variable, "streamspeed"))
    {
        stream_speed(value);
        ret=1;
    }
    else if (!strcmp(variable, "nightmode"))
    {
        night_mode(value);
        if (value==0) streamspeed =1;
        ret=1;
    }
    else if (!strcmp(variable, "esp32reset"))
    {
        resetflag = 1;
        ret=1;
    }

    if (ret) return 1; //OK

    // its a camera setting command:
    s  = esp_camera_sensor_get(); // get the cameras function list

    if (!strcmp(variable, "framesize"))
    {
         func=(void*)s->set_framesize;
        streamspeed=1;
        nightmode=0;
        JPGerrors=DMAerrors=0; // clear errors after framesize change for better readability
    }
    else if (!strcmp(variable, "quality")) func = s->set_quality;
    else if (!strcmp(variable, "brightness")) func = s->set_brightness;
    else if (!strcmp(variable, "contrast")) func = s->set_contrast;
    else if (!strcmp(variable, "saturation")) func = s->set_saturation;
    else if (!strcmp(variable, "special_effect")) func = s->set_special_effect;
    else if (!strcmp(variable, "awb")) func = s->set_whitebal;
    else if (!strcmp(variable, "wb_mode")) func = s->set_wb_mode;
    else if (!strcmp(variable, "awb_gain")) func = s->set_awb_gain;
    else if (!strcmp(variable, "aec")) func = s->set_exposure_ctrl;
    else if (!strcmp(variable, "aec_value")) func = s->set_aec_value;
    else if (!strcmp(variable, "ae_level")) func = s->set_ae_level;
    else if (!strcmp(variable, "aec2")) func = s->set_aec2;
    else if (!strcmp(variable, "agc")) func = s->set_gain_ctrl;
    else if (!strcmp(variable, "agc_gain")) func = s->set_agc_gain;
    else if (!strcmp(variable, "gainceiling")) func = (void*)s->set_gainceiling;
    else if (!strcmp(variable, "raw_gma")) func = s->set_raw_gma;
    else if (!strcmp(variable, "lenc")) func = s->set_lenc;
    else if (!strcmp(variable, "hmirror")) func = s->set_hmirror;
    else if (!strcmp(variable, "vflip")) func = s->set_vflip;
    else if (!strcmp(variable, "colorbar")) func = s->set_colorbar;
    else if (!strcmp(variable, "wpc")) func = s->set_wpc;
    else if (!strcmp(variable, "dcw")) func = s->set_dcw;
    else if (!strcmp(variable, "bpc")) func = s->set_bpc;
    //else the function is not supported: s->function=NULL
    if (func == NULL)
    {
        ESP_LOGE(TAG,"Control not supported");
        return 0; //setting not supported by camera
    }

//call the function:
    if ((*func)(s,value) != 0)
    {
        ESP_LOGE(TAG,"Camera Control failed");
        return -1; // set value failed
    }


    return 1; // OK
}



/* get a status info from the server:
This is a new feature to deliver data to the webpage from the server like current framerate
entry:
- uri-string containing the json formatted request: fe. URI: /getstatus?var=framerate
The var identifiys the parameter requested, here its framerate.
exit:
returns the requested  data in text form. it uses global iobuf to return the response text
- 1 = OK
- 0 = fail, '-1' is returned as text to webpage

*/
static int get_status(char *uri)
{
    char *variable;
    // get json parameters from uri
    strtok(uri, "=&"); //goto first & or = .tell strtok to use string uri. returns: "/getstatus?var"
    variable=strtok(NULL, "=&");// we are now at '='.  from last = find next = or & and put a /0 there. returns: "framerate"


    ESP_LOGI(TAG, "getstatus: %s", variable);

//check 'variable' for the request parameter
    if (!strcmp(variable, "framerate"))
    {
        sprintf(iobuf,"- NetFPS:%d CamFPS:%d I2sFPS:%d - QUEerrors:%d JPGerrors:%d - UpTime(hrs):%d - Rssi:%d",NetFPS,HwFPS,I2sFPS,DMAerrors,JPGerrors,(uptime/3600), rssi);
        return 1; //OK, answer in iobuf
    }

    sprintf(iobuf,"%d",-1);
    return 0; // no parameter-name match
}



/*
get current camera settings status. To be used to init the webpage after retrieval.
exit:
returns the requested  data in text form. it uses global iobuf to return the response text
- 1 = OK
- 0 = fail
*/
static int get_camstatus(void)
{
    sensor_t *s  =  esp_camera_sensor_get(); // get the status of camera controls from camera
    if (s == NULL) return 0;
    char *p = iobuf;
    // assemlbe them into a string
    *p++ = '{';

    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u,", s->status.quality);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
    //internal server maintained settings:
    p += sprintf(p, ",\"nightmode\":%d", nightmode);
    p += sprintf(p, ",\"streamspeed\":%d", streamspeed);
    p += sprintf(p, ",\"flashlight\":%d", flashlight);
    p += sprintf(p, ",\"streamlight\":%d", streamlight);

    *p++ = '}';
    *p++ = 0;
    return 1;
}

/* This function will set the stream fps to either fullspeed or reduced
by changing the xclk divisor value.
It actually changes the internal camera clock to lower value(slow) with the benefit, that
the camera exposure time increases and thus getting more light sensitive in darker situautions.
Also the amount of frames being transfered over the network decreases and thus gives less bandwidth load,
especially usefull if you have several of such cameras on the net.
*/
void stream_speed(int full)
{
    sensor_t *s  =  esp_camera_sensor_get(); // get the cameras function list

    if (full) // set full speed=max possible at current setting=default reset setting
    {
        s->set_reg(s,0x111,0x3f, 0x00); //about 25 fps at 640*480. set divider to 1
        streamspeed=1;
    }
    else
    {
        s->set_reg(s,0x111,0x3f, 0x02); //about 10 fps at 640*480. set divider to 2
        streamspeed=0;
    }

}

/* turn nightmode on/off
The advantage of nightmode is, that exposuretime is much longer, so you get less noise compared to
just increasing the gain, but at lower framerates.

example: 640*480 at normal constant framerate of 25fps:

if on: the framerate varys from 3 fps(dark) to 25fps(light).
	   here longer exposure times are used and exposuretime can be longer than 1 frame ie. 7frames long .
	   thus the framerate is reduced if longer exposuretimes are needed.
if off: framerate is returned to normal constant state of fe. 25 fps, also in dark, so less sensitive.
        exposuretime can only be max 1 frame then.
		NOTE: AEC and AGC must be ON for nightmode to work as its the exposure/light-control engine.
		      (AEC and AGC are working closely together!)
		      Use AE-Level to adjust to best brightness.
			  The clock is changed to full speed after nightmode usage!
*/
void night_mode(int on)
{
    sensor_t *s  =  esp_camera_sensor_get(); // get the cameras function list

    s->set_reg(s,0x111,0xff, 0x00); // first switch to full speed clock
    vTaskDelay(200/portTICK_PERIOD_MS);

    if (on) //turn on
    {
        s->set_reg(s,0x10f,0xff, 0x4b); //undocumented register!! enable extended exposuretimes by inserting dummyframes and lines.
        s->set_reg(s,0x103,0xff, 0xcf); //COM1, allow upto 7 dummyframes, allow additional lines being inserted at start/End of frame
        nightmode=1;
    }
    else //turn off, is abit complicated
    {
        s->set_reg(s,0x103,0xff, 0x0a); //COM1, only allow aditional lines at start of frame
        s->set_reg(s,0x10f,0xff, 0x43);
        s->set_reg(s,0x10f,0xff, 0x4b); //changes are taken at rising edge bit 3
        vTaskDelay(1000/portTICK_PERIOD_MS); //it needs some settle time
        s->set_reg(s,0x10f,0xff, 0x43);
        nightmode=0;
        streamspeed=1;
    }

}

// 1 sec peridic timer
void timerCallBack( TimerHandle_t xTimer )
{
	wifi_ap_record_t ap;
	
    NetFPS=NetFrameCnt;
    NetFrameCnt=0;
    HwFPS=HwFrameCnt;
    HwFrameCnt=0;
    I2sFPS=I2sFrameCnt;
    I2sFrameCnt=0;
    uptime++;
	

esp_wifi_sta_get_ap_info(&ap);
rssi= ap.rssi;

}
