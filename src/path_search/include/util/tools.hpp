#pragma once

#include <vector>

namespace Tools {
    // 采样函数
    template<typename T> std::vector<double> linspace(T start_in, T end_in, int num_in) {

        std::vector<double> linspaced;

        double start = static_cast<double>(start_in);
        double end   = static_cast<double>(end_in);
        double num   = static_cast<double>(num_in);

        if (num == 0) {
            return linspaced;
        }
        if (num == 1) {
            linspaced.push_back(start);
            return linspaced;
        }

        double delta = (end - start) / (num - 1);

        for (int i = 0; i < num - 1; ++i) {
            linspaced.push_back(start + delta * i);
        }
        linspaced.push_back(end);
        return linspaced;
    }
}  // namespace Tools
