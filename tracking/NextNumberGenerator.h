#pragma once
#include <stdexcept>

class NextNumberGenerator {
private:
    int numbers_[2] = {1, 2};
    int call_count_ = 0;

public:
    NextNumberGenerator(int input) {
        if (input < 1 || input > 3) {
            throw std::invalid_argument("Input must be 1, 2, or 3.");
        }
        switch (input) {
        case 1: numbers_[0] = 2; numbers_[1] = 3; break;
        case 2: numbers_[0] = 1; numbers_[1] = 3; break;
        case 3: numbers_[0] = 1; numbers_[1] = 2; break;
        }
    }

    int next() {
        int index = call_count_ % 2;
        call_count_++;
        return numbers_[index];
    }
};
