#include "esp_camera.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"

constexpr uint32_t SERVO_TIMEBASE_RESOLUTION_HZ = 1000000;
constexpr uint32_t SERVO_TIMEBASE_PERIOD = 20000;

#ifdef ESP32S3BOX
static constexpr gpio_num_t SERVO1_PIN = GPIO_NUM_19;
static constexpr gpio_num_t SERVO2_PIN = GPIO_NUM_20;
static constexpr gpio_num_t LED_GPIO_NUM = GPIO_NUM_14;

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 11
#define SIOD_GPIO_NUM 17
#define SIOC_GPIO_NUM 41
#define Y9_GPIO_NUM 13
#define Y8_GPIO_NUM 4
#define Y7_GPIO_NUM 10
#define Y6_GPIO_NUM 5
#define Y5_GPIO_NUM 7
#define Y4_GPIO_NUM 16
#define Y3_GPIO_NUM 15
#define Y2_GPIO_NUM 6
#define VSYNC_GPIO_NUM 42
#define HREF_GPIO_NUM 18
#define PCLK_GPIO_NUM 12

#define LED_GPIO_NUM (gpio_num_t) 14

#elif defined(SEEED_XIAO_ESP32S3)
static constexpr gpio_num_t SERVO1_PIN = GPIO_NUM_3;
static constexpr gpio_num_t SERVO2_PIN = GPIO_NUM_4;
static constexpr gpio_num_t LED_GPIO_NUM = GPIO_NUM_21;

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

#endif

static const char *TAG = "cams3rx";

typedef struct
{
    uint32_t packet_id;
    uint16_t servo[2];
} input_t;


const size_t max_buf = 8192;

typedef struct
{
    size_t len;                                    /*!< Length of the buffer in bytes */
    size_t width;                                  /*!< Width of the buffer in pixels */
    size_t height;                                 /*!< Height of the buffer in pixels */
    pixformat_t format;                            /*!< Format of the pixel data */
    struct timeval camera_timestamp;               /*!< Timestamp since boot of the first DMA buffer of the frame */
    struct timeval last_received_packet_timestamp; /*!< Timestamp of packet reception */
    uint32_t last_received_packet_id;              /*!< Packet ID */
    uint8_t image[max_buf];                          /*!< data */
} output_t;


// load wifi configuration from environment variables
#if defined(CAMS3RX_WIFI_MODE_AP)
    constexpr wifi_mode_t WIFI_MODE = WIFI_MODE_AP;
#elif defined(CAMS3RX_WIFI_MODE_STA)
    constexpr wifi_mode_t WIFI_MODE = WIFI_MODE_STA;
#else
    // or set the wifi mode here
    // wifi mode (WIFI_MODE_AP or WIFI_MODE_STA)
    constexpr wifi_mode_t WIFI_MODE = WIFI_MODE_AP;
#endif

// load wifi configuration from environment variables
static const char *WIFI_SSID = CAMS3RX_WIFI_SSID;
static const char *WIFI_PASSWORD = CAMS3RX_WIFI_PASSWORD;
// or set the wifi ssid and password here
// (set blank password for open networks)
// const char WIFI_SSID[] = "your_ssid";
// const char WIFI_PASSWORD[] = "";

static esp_netif_t *sta_netif = NULL;
static esp_netif_t *ap_netif = NULL;

const char *NETIF_DESC_STA = "cams3rx_sta";
const char *NETIF_DESC_AP = "cams3rx_ap";

struct sockaddr_in udp_addr;

static int udp_socket_recv;
static int udp_socket_send;
static struct sockaddr_storage source_addr;
static socklen_t socklen = sizeof(source_addr);
static uint64_t last_received = 0;

const unsigned int udp_recv_port = 12345;
const unsigned int udp_send_port = 12346;

static SemaphoreHandle_t s_semph_get_ip_addrs = NULL;
EventGroupHandle_t EventConnected;
static volatile bool is_broadcast = true;

input_t input_dat;
mcpwm_cmpr_handle_t comparator_1 = NULL;
mcpwm_cmpr_handle_t comparator_2 = NULL;

static int s_retry_num = 0;

static QueueHandle_t vsync_queue = NULL;

static void IRAM_ATTR vsync_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(vsync_queue, &gpio_num, NULL);
}

void init_vsync_interrupt() {
    vsync_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << VSYNC_GPIO_NUM);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t) VSYNC_GPIO_NUM, vsync_isr_handler, (void*) VSYNC_GPIO_NUM);
}


static void handler_on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                                               int32_t event_id, void *event_data)
{
    s_retry_num++;

    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED)
    {
        return;
    }
    ESP_ERROR_CHECK(err);
}

static void example_handler_on_wifi_connect(void *esp_netif, esp_event_base_t event_base,
                                            int32_t event_id, void *event_data)
{
}

bool example_is_our_netif(const char *prefix, esp_netif_t *netif)
{
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
}

static void example_handler_on_sta_got_ip(void *arg, esp_event_base_t event_base,
                                          int32_t event_id, void *event_data)
{
    s_retry_num = 0;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!example_is_our_netif(NETIF_DESC_STA, event->esp_netif))
    {
        return;
    }
    ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    if (s_semph_get_ip_addrs)
    {
        xSemaphoreGive(s_semph_get_ip_addrs);
    }
    else
    {
        ESP_LOGI(TAG, "- IPv4 address: " IPSTR ",", IP2STR(&event->ip_info.ip));
    }
}

void example_wifi_stop(void)
{
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT)
    {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(sta_netif));
    esp_netif_destroy(sta_netif);
    sta_netif = NULL;
}

esp_err_t example_wifi_sta_do_disconnect(void)
{
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &handler_on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect));

    if (s_semph_get_ip_addrs)
    {
        vSemaphoreDelete(s_semph_get_ip_addrs);
    }
    return esp_wifi_disconnect();
}

void example_wifi_shutdown(void)
{
    example_wifi_sta_do_disconnect();
    example_wifi_stop();
}

static esp_err_t print_all_ips_tcpip(void *ctx)
{
    const char *prefix = static_cast<const char *>(ctx);
    // iterate over active interfaces, and print out IPs of "our" netifs
    esp_netif_t *netif = NULL;
    while ((netif = esp_netif_next_unsafe(netif)) != NULL)
    {
        if (example_is_our_netif(prefix, netif))
        {
            ESP_LOGI(TAG, "Connected to %s", esp_netif_get_desc(netif));
            esp_netif_ip_info_t ip;
            ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip));

            ESP_LOGI(TAG, "- IPv4 address: " IPSTR ",", IP2STR(&ip.ip));
        }
    }
    return ESP_OK;
}

void example_print_all_netif_ips(const char *prefix)
{
    // Print all IPs in TCPIP context to avoid potential races of removing/adding netifs when iterating over the list
    esp_netif_tcpip_exec(print_all_ips_tcpip, (void *)prefix);
}

static void example_handler_on_ap_sta_connected(void *arg, esp_event_base_t event_base,
                                                int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "A station connected to the AP.");
    xEventGroupSetBits(EventConnected, BIT0);
}

static void example_handler_on_ap_sta_disconnected(void *arg, esp_event_base_t event_base,
                                                   int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "A station disconnected from the AP.");
    xEventGroupClearBits(EventConnected, BIT0);
}

static output_t sensor_dat;
SemaphoreHandle_t xOutputMutex;

inline int process_camera_frame()
{
    static int count = 0;
    count = (count + 1) % 107;

    int64_t start_time = esp_timer_get_time();

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        return -1;
    }

    if(fb->len > max_buf){
        ESP_LOGE(TAG, "Buffer overflow: %d > %d", fb->len, max_buf);
        esp_camera_fb_return(fb);
        return -1;
    }
    sensor_dat.len = fb->len;
    sensor_dat.width = fb->width;
    sensor_dat.height = fb->height;
    sensor_dat.format = fb->format;
    sensor_dat.camera_timestamp = fb->timestamp;
    memcpy(sensor_dat.image, fb->buf, fb->len);

    size_t fb_size = fb->len;
    esp_camera_fb_return(fb);
    int64_t end_time = esp_timer_get_time();
    if (count == 1)
    {
        ESP_LOGI(TAG, "process_frame() took %lld us, size=%d", end_time - start_time, fb_size);
    }
    return 0;
}

static void camera_task(void *pvParameters)
{
    int err;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in broadcast_addr;
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(udp_send_port);
    int led_state = 0;

    udp_socket_send = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (udp_socket_send < 0)
    {
        ESP_LOGE(TAG, "Unable to create send socket: errno %d", errno);
        close(udp_socket_send);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Send Socket created");
    int broadcast = 1;
    setsockopt(udp_socket_send, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    // Set IP Precedence to AC_VO (Voice) to prevent packets to be aggregated
    const int ip_precedence_vi = 7;
    const int ip_precedence_offset = 5;
    int priority = (ip_precedence_vi << ip_precedence_offset);
    setsockopt(udp_socket_send, IPPROTO_IP, IP_TOS, &priority, sizeof(priority));

    // Set non-blocking mode
    int flags = fcntl(udp_socket_send, F_GETFL, 0);
    err = fcntl(udp_socket_send, F_SETFL, flags | O_NONBLOCK);
    if (err < 0)
    {
        ESP_LOGE(TAG, "Send Socket unable to set non-blocking mode: errno %d", errno);
        close(udp_socket_send);
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        if (xEventGroupGetBits(EventConnected) & BIT0)
        {
            uint32_t  io_num;
             if (xQueueReceive(vsync_queue, &io_num, portMAX_DELAY)) {
                int err = process_camera_frame();
                if(err<0){
                    continue;
                }

                struct sockaddr_in *target_addr = is_broadcast ? &broadcast_addr : (struct sockaddr_in *)&source_addr;
                target_addr->sin_port = htons(udp_send_port);

                err = sendto(udp_socket_send, &sensor_dat, sizeof(sensor_dat)/* - (max_buf - sensor_dat.len)*/, 0, (struct sockaddr *)target_addr, sizeof(*target_addr));

                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                if(is_broadcast){
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
             }
        }else{
            vTaskDelay(20);
        }
        gpio_set_level(LED_GPIO_NUM, led_state);
        led_state = !led_state;
    }
}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    struct sockaddr_in dest_addr;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    int err;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(udp_recv_port);

    udp_socket_recv = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (udp_socket_recv < 0)
    {
        ESP_LOGE(TAG, "Unable to create receive socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Receive Socket created");

    struct timeval tv;
    tv.tv_sec = 0;  
    tv.tv_usec = 100*1000; 
    if (setsockopt(udp_socket_recv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        ESP_LOGE(TAG, "Error setting socket timeout");
        close(udp_socket_recv);
        vTaskDelete(NULL);
        return;
    }

    err = bind(udp_socket_recv, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Receive Socket unable to bind: errno %d", errno);
        close(udp_socket_recv);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Receive Socket bound, port %d", ntohs(dest_addr.sin_port));

    while (1)
    {
        int len = recvfrom(udp_socket_recv, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);

        if (len > 0)
        {
            last_received = (uint64_t)esp_timer_get_time();
            struct timeval packet_timestamp = {
                .tv_sec = (time_t)(last_received / 1000000ULL),
                .tv_usec = (suseconds_t)(last_received % 1000000ULL)};
            sensor_dat.last_received_packet_timestamp = packet_timestamp;
            if (is_broadcast)
            {
                ESP_LOGI(TAG, "exit broadcast mode");
            }
            is_broadcast = false;
            memcpy(&input_dat, rx_buffer, sizeof(input_t));
            sensor_dat.last_received_packet_id = input_dat.packet_id;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_1, input_dat.servo[0]));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_2, input_dat.servo[1]));
        }
        if (len < 0){
            uint64_t now = (uint64_t)esp_timer_get_time();
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_1, 0));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_2, 0));

                if (!is_broadcast && now - last_received > 10*1000*1000)
                {
                    is_broadcast = true;
                    ESP_LOGI(TAG, "enter broadcast mode");
                }
            }
        }
    }

    close(udp_socket_recv);
    vTaskDelete(NULL);
}

inline esp_err_t wifi_connect_sta()
{
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    esp_netif_config.if_desc = NETIF_DESC_STA;
    esp_netif_config.route_prio = 128;
    sta_netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_config_t wifi_config = {};
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.rssi = -127;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASSWORD);

    s_semph_get_ip_addrs = xSemaphoreCreateBinary();
    if (s_semph_get_ip_addrs == NULL)
    {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }

    s_retry_num = 0;
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &handler_on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &example_handler_on_sta_got_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &example_handler_on_wifi_connect, sta_netif));

    ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi connect failed! ret:%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Waiting for IP(s)");
    xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);

    ESP_ERROR_CHECK(esp_register_shutdown_handler(&example_wifi_shutdown));
    example_print_all_netif_ips(NETIF_DESC_STA);
    xEventGroupSetBits(EventConnected, BIT0);
    return ESP_OK;
}

inline esp_err_t wifi_connect_ap()
{
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &example_handler_on_ap_sta_connected, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &example_handler_on_ap_sta_disconnected, NULL));

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();
    esp_netif_config.if_desc = NETIF_DESC_AP;
    esp_netif_config.route_prio = 128;
    ap_netif = esp_netif_create_wifi(WIFI_IF_AP, &esp_netif_config);
    esp_wifi_set_default_wifi_ap_handlers();

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.ap.ssid, WIFI_SSID);
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    strcpy((char *)wifi_config.ap.password, WIFI_PASSWORD);
    wifi_config.ap.max_connection = 1;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(WIFI_PASSWORD) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    ESP_LOGI(TAG, "WiFi AP started. SSID:%s password:%s", WIFI_SSID, WIFI_PASSWORD);

    // Set up the DHCP server
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    return ESP_OK;
}

void init_servo()
{
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .intr_priority = 0,
        .flags = {
            .update_period_on_empty = false, 
            .update_period_on_sync = false},
    };    
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = 0; // operator must be in the same group as the timer

    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");

    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_2));

    mcpwm_gen_handle_t generator_1 = NULL;
    mcpwm_gen_handle_t generator_2 = NULL;
    mcpwm_generator_config_t generator_config_1 = {
        .gen_gpio_num = SERVO1_PIN,
        .flags = {
            .invert_pwm = false,
            .io_loop_back = false,
            .io_od_mode = false,
            .pull_up = false,
            .pull_down = false,
        },
    };
    mcpwm_generator_config_t generator_config_2 = {
        .gen_gpio_num = SERVO2_PIN,
        .flags = {
            .invert_pwm = false,
            .io_loop_back = false,
            .io_od_mode = false,
            .pull_up = false,
            .pull_down = false,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config_1, &generator_1));
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config_2, &generator_2));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_1, 1500));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_2, 1500));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_1,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_2,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_1,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_1, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_2,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_2, MCPWM_GEN_ACTION_LOW)));
    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

extern "C" void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    uint32_t _caps = MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM;
    size_t free_heap = heap_caps_get_free_size(_caps);
    size_t largest_free_block = heap_caps_get_largest_free_block(_caps);
    ESP_LOGI(TAG, "_caps: %d, free heap size: %d, Largest free block: %d", (int)_caps, free_heap, largest_free_block);

    EventConnected = xEventGroupCreate();

    if (WIFI_MODE == WIFI_MODE_STA)
    {
        wifi_connect_sta();
    }
    else if (WIFI_MODE == WIFI_MODE_AP)
    {
        wifi_connect_ap();
    }

    // some boards have an issue with the default max power
    // esp_wifi_set_max_tx_power(55);

    init_servo();

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000L;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QCIF;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }
    xOutputMutex = xSemaphoreCreateMutex();
    xSemaphoreGive(xOutputMutex);
    xTaskCreatePinnedToCore(udp_server_task, "udp_server", 4096, (void *)AF_INET, 10, NULL, 0);
    init_vsync_interrupt();
    xTaskCreatePinnedToCore(camera_task, "camera_task", 4096, NULL, 10, NULL, 1);

    esp_rom_gpio_pad_select_gpio(LED_GPIO_NUM);
    gpio_set_direction(LED_GPIO_NUM, GPIO_MODE_OUTPUT);
}