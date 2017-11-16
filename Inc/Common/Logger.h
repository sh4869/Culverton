#ifndef LOGGER_H_
#define LOGGER_H_

#include <array>
#include "Encoder.h"
namespace Log {
    namespace Speed {
        static std::array<float, 1000> Log;
        static unsigned int index = 0;
        void Push(float data);
        float Get(int index);
    }  // namespace Speed
    namespace Target {
        static std::array<float, 1000> Log;
        static unsigned int index = 0;
        void Push(float data);
        float Get(int index);
    }  // namespace Target
    namespace Encoder {
        static std::array<EncoderVelocity, 1000> Log;
        static unsigned int index = 0;
        void Push(EncoderVelocity data);
        EncoderVelocity Get(int index);
    }  // namespace Encoder
    namespace Velocity {
        static std::array<float, 1000> Log;
        static unsigned int index = 0;
        void Push(float data);
        float Get(int index);
    }  // namespace Velocity
}  // namespace Log

#endif