#include <rclcpp/rclcpp.hpp>

#include <iostream>

#include <fstream>

#include <sstream>

#include <vector>

#include <string>



struct Record {

    double x;

    double y;

    double z;

    std::string surface;

    std::string action;

};



class CsvReaderNode : public rclcpp::Node {

public:

    CsvReaderNode() : Node("csv_reader_node") {

        RCLCPP_INFO(this->get_logger(), "Spúšťam CSV Reader Node...");



        std::string filename = "mission_1_all.csv";

        std::ifstream file(filename);



        if (!file.is_open()) {

            RCLCPP_ERROR(this->get_logger(), "Nepodarilo sa otvoriť súbor: %s", filename.c_str());

            return;

        }



        std::string line;

        std::vector<Record> records;



        while (std::getline(file, line)) {

            std::stringstream ss(line);

            std::string token;

            Record r;



            // načítanie čísel

            std::getline(ss, token, ',');

            r.x = std::stod(token);

            std::getline(ss, token, ',');

            r.y = std::stod(token);

            std::getline(ss, token, ',');

            r.z = std::stod(token);



            // načítanie textových hodnôt

            std::getline(ss, r.surface, ',');

            std::getline(ss, r.action, ',');



            records.push_back(r);

        }



        file.close();



        // výpis dát

        for (const auto& r : records) {

            RCLCPP_INFO(this->get_logger(),

                        "x=%.3f, y=%.3f, z=%.3f, surface=%s, action=%s",

                        r.x, r.y, r.z, r.surface.c_str(), r.action.c_str());

        }



        RCLCPP_INFO(this->get_logger(), "Načítaných záznamov: %zu", records.size());

    }

};



int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CsvReaderNode>());

    rclcpp::shutdown();

    return 0;

}
