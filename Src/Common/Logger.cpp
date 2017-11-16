#include "Logger.h"

namespace Log {
    namespace Speed {
        void Push(float data) {
            if (index < 1000 - 1) {
                index++;
                Log[index] = data;
            }
        }
        float Get(int index) {
            if (index > -1 && index < 1000) {
                return Log.at(index);
            } else {
                return 0;
            }
        }
    }  // namespace Speed
    namespace Target {
        void Push(float data) {
            if (index < 1000 - 1) {
                index++;
                Log[index] = data;
            }
        }
        float Get(int index) {
            if (index > -1 && index < 1000) {
                return Log.at(index);
            } else {
                return 0;
            }
        }
    }  // namespace Target
    namespace Encoder {
        void Push(EncoderVelocity data) {
            if (index < 1000 - 1) {
                index++;
                Log[index] = data;
            }
        }
        EncoderVelocity Get(int index) {
            if (index > -1 && index < 1000) {
                return Log.at(index);
            } else {
                return {};
            }
        }
    }  // namespace Encoder
    namespace Velocity {
        void Push(float data) {
            if (index < 1000 - 1) {
                index++;
                Log[index] = data;
            }
        }
        float Get(int index) {
            if (index > -1 && index < 1000) {
                return Log.at(index);
            } else {
                return 0;
            }
        }
    }  // namespace Velocity
}  // namespace Log
