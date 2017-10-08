#ifndef UTIL_H_
#define UTIL_H_

namespace Util {
void Delay(__IO uint32_t count) {
    for (; count != 0; count--)
        ;
}
}

#endif