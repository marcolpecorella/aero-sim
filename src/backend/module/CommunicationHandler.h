#ifndef  COMMUNICATIONHANDLER_H
#define COMMUNICATIONHANDLER_H

#include <cpprest/http_listener.h>
#include <cpprest/json.h>

using namespace web;
using namespace web::http;
using namespace web::http::experimental::listener;

class CommunicationHandler {
public:

    explicit CommunicationHandler(const utility::string_t &url) : m_listener(url) {}

    ~CommunicationHandler() = default;

    void start();
    void stop();

private:
    static void add_cors_headers(http_response& response);
    static void handle_options(const http_request& request);

    void handle_get(const http_request& request) const;
    void handle_post(const http_request& request);
    bool handle_post_request(const json::value & value);

    http_listener m_listener;
};

#endif // COMMUNICATIONHANDLER_H