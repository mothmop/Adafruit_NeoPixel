#include "StackingEngine.h"
#include <iostream>
#include <cstdlib>

int main(int argc, char* argv[]) {
    double range_mm = 4.0;
    int frame_count = 15;
    double ruler_cm = 3.5;

    if (argc >= 4) {
        range_mm = std::atof(argv[1]);
        frame_count = std::atoi(argv[2]);
        ruler_cm = std::atof(argv[3]);
    } else {
        std::cout << "Usage: " << argv[0] << " <range_mm> <frame_count> <ruler_cm>\n"
                  << "Using default parameters: range=4.0mm, frames=15, ruler=3.5cm\n";
    }

    StackingEngine engine("macro_calibration.csv", "stacking_engine.log");

    if (!engine.initializeSDK()) return 1;
    if (!engine.connectCamera()) {
        std::cerr << "Camera connection aborted. Check USB connection and PC Remote setting.\n";
        return 1;
    }

    engine.executeSweep(range_mm, frame_count, ruler_cm);
    return 0;
}
