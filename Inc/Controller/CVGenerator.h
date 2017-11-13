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
    virtual ~CVGenerator();
    virtual float Update();
    virtual void Disable();
    virtual void Enable();
    virtual const bool isEnable();
};

#endif