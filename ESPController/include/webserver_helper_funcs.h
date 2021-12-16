#ifndef DIYBMSWebServer_Helper_H_
#define DIYBMSWebServer_Helper_H_

#include <stdio.h>
#include <esp_http_server.h>
#include "defines.h"

bool GetTextFromKeyValue(const char *buffer, const char *key,  char *text, size_t textLength);
bool GetUint32FromKeyValue(const char *buffer, const char *key,  uint32_t *value);
bool GetBoolFromKeyValue(const char *buffer, const char *key,  bool *value);

bool getPostDataIntoBuffer(httpd_req_t *req);
void url_decode(char *str, char *buf);
char from_hex(char ch);

bool validateXSS(httpd_req_t *req);
bool HasURLEncodedHeader(httpd_req_t *req);

bool validateXSSWithPOST(httpd_req_t *req, const char *postbuffer);

void setCookieValue();
void setCookie(httpd_req_t *req);

// These are borrowed from the new ESP IDF framework, will need to be removed if framework is upgraded
esp_err_t httpd_req_get_cookie_val(httpd_req_t *req, const char *cookie_name, char *val, size_t *val_size);
esp_err_t httpd_cookie_key_value(const char *cookie_str, const char *key, char *val, size_t *val_size);

extern char httpbuf[BUFSIZE];

#endif