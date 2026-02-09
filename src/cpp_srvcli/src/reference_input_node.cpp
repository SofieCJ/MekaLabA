#include "rclcpp/rclcpp.hpp"

#include "cpp_srvcli/srv/set_reference.hpp"

#include <iostream>
#include <memory>
#include <string>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("reference_input_node");

    // Klient mot service-navnet i oppgaven
    auto client = node->create_client<my_interfaces::srv::SetReference>("set_reference");

    // Vent til service finnes
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Avbrutt mens jeg ventet på service.");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Venter på service 'set_reference' ...");
    }

    RCLCPP_INFO(node->get_logger(), "Skriv inn ny referanse (tall). Ctrl+C for å avslutte.");

    while (rclcpp::ok()) {
        std::cout << "Sett inn ny referanse: ";
        std::string line;
        if (!std::getline(std::cin, line)) break;

        // Hopp over tom input
        if (line.empty()) continue;

        long long ref_value;
        try {
            ref_value = std::stoll(line);
        } catch (...) {
            std::cout << "Ugyldig tall. Prøv igjen.\n";
            continue;
        }

        auto request = std::make_shared<my_interfaces::srv::SetReference::Request>();
        request->reference = ref_value;   // antar felt heter "reference"

        auto future = client->async_send_request(request);

        // Vent på svar (og spinn mens vi venter)
        auto ret = rclcpp::spin_until_future_complete(node, future);
        if (ret == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(node->get_logger(), "Svar: success=%s, message='%s'",
                        response->success ? "true" : "false",
                        response->message.c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "Fikk ikke svar fra service.");
        }
    }

    rclcpp::shutdown();
    return 0;
}
