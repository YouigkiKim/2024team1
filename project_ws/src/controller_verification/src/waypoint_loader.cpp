#include "waypoint_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

std::vector<std::pair<double, double>> WaypointLoader::loadWaypointsFromFile(const std::string& filepath) {
    std::vector<std::pair<double, double>> waypoints;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cout << "Cannot open txt file in: " << filepath << std::endl;
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line)) {
        size_t location_pos = line.find("Location");
        if (location_pos != std::string::npos) {
            try {
                // x 좌표 파싱
                size_t start = line.find("x=", location_pos) + 2;
                size_t end = line.find(",", start);
                std::string x_str = line.substr(start, end - start);

                // y 좌표 파싱
                start = line.find("y=", end) + 2;
                end = line.find(",", start);
                std::string y_str = line.substr(start, end - start);

                // 문자열을 double로 변환
                double x = std::stod(x_str);
                double y = std::stod(y_str);

                waypoints.emplace_back(x, y);

                // 파싱된 좌표 출력
                std::cout << "Loaded waypoint: x = " << x << ", y = " << y << std::endl;
            } catch (const std::invalid_argument& e) {
                std::cerr << "숫자 변환 오류: " << e.what() << " in line: " << line << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "숫자 범위 오류: " << e.what() << " in line: " << line << std::endl;
            }
        }
    }

    file.close();
    std::cout << "Waypoints load 완료: " << waypoints.size() << " points loaded." << std::endl;
    return waypoints;
}
