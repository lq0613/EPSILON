#ifndef _COMMON_INC_COMMON_STATE_STATE_H__
#define _COMMON_INC_COMMON_STATE_STATE_H__

#include "common/basics/basics.h"
#include <iostream>
#include <fstream>

namespace common {
 template<typename T>
void writeToLogFile(const T& data) {
    const std::string filePath = "/home/liuqiao/project/log.txt";
    std::ofstream outputFile(filePath, std::ios::app);

    if (!outputFile.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return;
    }

    outputFile << data << std::endl;
    outputFile.close();
}

struct State {
  decimal_t time_stamp{0.0};
  Vecf<2> vec_position{Vecf<2>::Zero()};
  decimal_t angle{0.0};
  decimal_t curvature{0.0};
  decimal_t velocity{0.0};
  decimal_t acceleration{0.0};
  decimal_t steer{0.0};
  void print() const {
    printf("State:\n");
    printf(" -- time_stamp: %lf.\n", time_stamp);
    printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
    printf(" -- angle: %lf.\n", angle);
    printf(" -- curvature: %lf.\n", curvature);
    printf(" -- velocity: %lf.\n", velocity);
    printf(" -- acceleration: %lf.\n", acceleration);
    printf(" -- steer: %lf.\n", steer);
  }
  void output(const std::string& filename) const {
        std::ofstream outfile(filename, std::ios::app);

        if (outfile.is_open()) {
            outfile << "State:\n";
            outfile << " -- time_stamp: " << time_stamp << ".\n";
            outfile << " -- vec_position: (" << vec_position[0] << ", " << vec_position[1] << ").\n";
            outfile << " -- angle: " << angle << ".\n";
            outfile << " -- curvature: " << curvature << ".\n";
            outfile << " -- velocity: " << velocity << ".\n";
            outfile << " -- acceleration: " << acceleration << ".\n";
            outfile << " -- steer: " << steer << ".\n";
            outfile.close();
            std::cout << "Data has been written to " << filename << std::endl;
        } else {
            printf( "Unable to open file: ");
        }
    }

  Vec3f ToXYTheta() const {
    return Vec3f(vec_position(0), vec_position(1), angle);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common

#endif  // _COMMON_INC_COMMON_STATE_STATE_H__