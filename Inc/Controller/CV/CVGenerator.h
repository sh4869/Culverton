#ifndef CVGENERATOR_H_
#define CVGENERATOR_H_

#include <memory>

#include "PIDController.h"

/**
 * @brief Base of Control Variable Generator
 * 
 */
class CVGenerator {
public:
    ~CVGenerator() = default;
    virtual float Update() = 0;
};

#endif