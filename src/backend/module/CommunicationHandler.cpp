#include "CommunicationHandler.h"

void CommunicationHandler::start() {
        m_listener.support(methods::OPTIONS, [this](auto && PH1) { handle_options(std::forward<decltype(PH1)>(PH1)); });
        m_listener.support(methods::GET, [this](auto && PH1) { handle_get(std::forward<decltype(PH1)>(PH1)); });
        m_listener.support(methods::POST, [this](auto && PH1) { handle_post(std::forward<decltype(PH1)>(PH1)); });

        try {
            m_listener.open().wait();
            std::cout << U("Listening on: ") << m_listener.uri().to_string() << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "Error starting m_listener: " << e.what() << std::endl;
        }
}

void CommunicationHandler::stop()  {
    m_listener.close().wait();
}

bool CommunicationHandler::handle_post_request(const json::value &value) {

    /**
     * TODO This function will be dedicated to all the possible post request received
     *
     **/
    return true;
}

void CommunicationHandler::add_cors_headers(http_response &response) {
        response.headers().add(U("Access-Control-Allow-Origin"), U("http://localhost:4200"));
        response.headers().add(U("Access-Control-Allow-Methods"), U("GET, POST, OPTIONS"));
        response.headers().add(U("Access-Control-Allow-Headers"), U("Content-Type"));
}

void CommunicationHandler::handle_options(const http_request& request) {
    http_response response(status_codes::OK);
    add_cors_headers(response);
    auto _ = request.reply(response);
}

void CommunicationHandler::handle_get(const http_request& request) const {
    http_response response(status_codes::OK);
    add_cors_headers(response);

    if (request.relative_uri() == "/...") {
        // This is the part of code in which there will be handled the request from the client side
    } else {
        response.set_status_code(status_codes::InternalError);
    }

    auto _ = request.reply(response);
}

void CommunicationHandler::handle_post(const http_request& request) {
    request.extract_json().then([=](const json::value& requestBody) {
        this->handle_post_request(requestBody);

        /* Handling response to request */
        http_response response(status_codes::OK);
        add_cors_headers(response);
        response.headers().set_content_type(U("application/json"));
        response.set_body(U("{\"message\":\"Data received\"}"));
        auto p = request.reply(response);
    }).then([=](const pplx::task<void>& t) {
        try {
            t.get();
        } catch (const std::exception &e) {
            http_response errorResponse(status_codes::InternalError);
            add_cors_headers(errorResponse);
            errorResponse.set_body(U("{\"error\":\"Internal server error\"}"));
            auto _ = request.reply(errorResponse);
            std::cerr << "Error processing POST: " << e.what() << std::endl;
        }
    }).wait();
}
