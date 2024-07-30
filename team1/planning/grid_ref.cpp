#include <iostream>
#include <vector>
#include <cmath>

class DynamicGrid {
public:
    DynamicGrid(double initial_speed) {
        updateGrid(initial_speed);
    }

    void updateGrid(double speed) {
        // 속도에 따라 grid 크기와 해상도 조정
        double grid_size = calculateGridSize(speed);
        int resolution = calculateResolution(speed);

        // grid 초기화
        grid.clear();
        grid.resize(resolution, std::vector<int>(resolution, 0));

        std::cout << "Grid updated for speed: " << speed << " m/s\n";
        std::cout << "Grid size: " << grid_size << " meters\n";
        std::cout << "Grid resolution: " << resolution << "x" << resolution << "\n";
    }

    void printGrid() const {
        for (const auto& row : grid) {
            for (const auto& cell : row) {
                std::cout << cell << " ";
            }
            std::cout << std::endl;
        }
    }

private:
    std::vector<std::vector<int>> grid;

    double calculateGridSize(double speed) const {
        // 속도에 따라 grid 크기 결정 (예: 속도가 높을수록 더 큰 영역을 커버)
        return std::min(100.0, speed * 10.0); // 최대 100미터까지 커버
    }

    int calculateResolution(double speed) const {
        // 속도에 따라 해상도 결정 (예: 속도가 높을수록 해상도가 낮아짐)
        return std::max(10, static_cast<int>(50 - speed)); // 최소 해상도 10x10
    }
};

int main() {
    double speed;
    std::cout << "Enter the vehicle speed (m/s): ";
    std::cin >> speed;

    DynamicGrid grid(speed);
    grid.printGrid();

    // 속도가 변할 때 grid 업데이트 예시
    std::cout << "\nUpdating grid for new speed...\n";
    double new_speed;
    std::cout << "Enter the new vehicle speed (m/s): ";
    std::cin >> new_speed;

    grid.updateGrid(new_speed);
    grid.printGrid();

    return 0;
}
