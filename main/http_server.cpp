#include "http_server.h"
#include "telemetry.h"

#include "esp_http_server.h"
#include "esp_log.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static const char *TAG = "http";

// The dashboard page, linked into the binary by EMBED_TXTFILES in CMakeLists.
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

// Comfortably larger than the serialized snapshot. Heap rather than stack:
// httpd's task stack is not the place for a buffer this size.
#define JSON_BUF_BYTES 12288

static esp_err_t indexHandler(httpd_req_t *req)
{
    const size_t len = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html_start, len);
}

static esp_err_t dataHandler(httpd_req_t *req)
{
    char *buf = (char *)malloc(JSON_BUF_BYTES);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }

    size_t n = telemetryToJson(buf, JSON_BUF_BYTES);

    httpd_resp_set_type(req, "application/json");
    // The page polls this route continuously; a cached copy would freeze it.
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    esp_err_t err;
    if (n == 0) {
        err = httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                  "telemetry serialization failed");
    } else {
        err = httpd_resp_send(req, buf, n);
    }

    free(buf);
    return err;
}

// POST /api/reset
//
// POST rather than GET so a link preview, a prefetch or a stray refresh cannot
// reboot the gauge. The body must carry a confirmation token, and the token is
// deliberately different when Mfg Status Init says a reset would strip FET_EN:
// a client that does not know about that hazard cannot stumble into it.
static esp_err_t resetHandler(httpd_req_t *req)
{
    char body[32] = {0};
    const int len = req->content_len;
    if (len <= 0 || len >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing confirmation");
        return ESP_FAIL;
    }
    const int got = httpd_req_recv(req, body, len);
    if (got <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "could not read body");
        return ESP_FAIL;
    }
    body[got] = '\0';

    const bool  safe  = telemetryResetIsSafe();
    const char *token = safe ? "CONFIRM" : "CONFIRM-UNSAFE";

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    if (strcmp(body, token) != 0) {
        ESP_LOGW(TAG, "reset rejected: confirmation token did not match");
        httpd_resp_set_status(req, "409 Conflict");
        char msg[96];
        int n = snprintf(msg, sizeof(msg),
                         "{\"ok\":false,\"safe\":%s,\"error\":\"bad confirmation\"}",
                         safe ? "true" : "false");
        return httpd_resp_send(req, msg, n);
    }

    const esp_err_t err = telemetryRequestReset();

    char msg[128];
    int n = snprintf(msg, sizeof(msg),
                     "{\"ok\":%s,\"safe\":%s,\"error\":\"%s\"}",
                     err == ESP_OK ? "true" : "false",
                     safe ? "true" : "false",
                     err == ESP_OK ? "" : esp_err_to_name(err));
    if (err != ESP_OK) httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_send(req, msg, n);
}

esp_err_t httpServerStart()
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size      = 8192;   // JSON goes to the heap, but TLS-free httpd
    cfg.lru_purge_enable = true;  // still needs room for the socket machinery
    cfg.max_uri_handlers = 4;

    httpd_handle_t server = nullptr;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    // Zero-initialized then assigned field by field: httpd_uri_t grows extra
    // members when WebSocket support is enabled in Kconfig, and a designated
    // initializer listing them would not compile with it switched off.
    httpd_uri_t route = {};
    route.method   = HTTP_GET;
    route.user_ctx = nullptr;

    route.uri     = "/";
    route.handler = indexHandler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &route));

    route.uri     = "/api/data";
    route.handler = dataHandler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &route));

    route.method  = HTTP_POST;
    route.uri     = "/api/reset";
    route.handler = resetHandler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &route));

    ESP_LOGI(TAG, "server started on port %d", cfg.server_port);
    return ESP_OK;
}
