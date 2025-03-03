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
    response.headers().add(U("Access-Control-Allow-Origin"), U("*"));  // For development, you can use * to allow all origins
    response.headers().add(U("Access-Control-Allow-Methods"), U("GET, POST, OPTIONS"));
    response.headers().add(U("Access-Control-Allow-Headers"), U("Content-Type, Authorization"));
    response.headers().add(U("Access-Control-Max-Age"), U("3600"));
}

void CommunicationHandler::handle_options(const http_request& request) {
    http_response response(status_codes::OK);
    add_cors_headers(response);
    request.reply(response);
}

void CommunicationHandler::handle_get(const http_request& request) const {
    http_response response(status_codes::OK);
    add_cors_headers(response);

    if (request.relative_uri() == "/test_get") {
        std::cout << request.body() << std::endl;
    } else {
        response.set_status_code(status_codes::InternalError);
    }

    auto _ = request.reply(response);
}

void CommunicationHandler::handle_post(const http_request& request) {
    std::cout << "POST request received at: " << request.relative_uri().to_string() << std::endl;

    http_response response;
    add_cors_headers(response);
    response.headers().set_content_type(U("application/json"));

    request.extract_json().then([&](const json::value& requestBody) {
        const std::string path = request.relative_uri().path();
        std::cout << "Received request at path: " << path << std::endl;

        // Stampa il contenuto JSON ricevuto
        std::cout << "Content received: " << requestBody.serialize() << std::endl;

        // Se il path contiene "/position", ritorna OK, altrimenti BadRequest
        if (path.find("/position") != std::string::npos) {
            response.set_status_code(status_codes::OK);
        } else {
            std::cout << "Unknown endpoint: " << path << std::endl;
            response.set_status_code(status_codes::BadRequest);
        }

        return request.reply(response);
    }).then([&](const pplx::task<void>& t) {
        try {
            t.get();
        } catch (const std::exception &e) {
            http_response errorResponse(status_codes::InternalError);
            add_cors_headers(errorResponse);
            errorResponse.set_body(U("{\"error\":\"Internal server error\"}"));
            request.reply(errorResponse);
            std::cerr << "Error processing POST: " << e.what() << std::endl;
        }
    }).wait();
}
