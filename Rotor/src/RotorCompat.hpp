#ifndef ROTOR_COMPAT_HPP
#define ROTOR_COMPAT_HPP

#include <iostream>
#include <sstream>

#define SG_INPUT 0
#define SG_ALERT 0
#define SG_FLIGHT 0
#define SG_INFO 1

#define SG_LOG(domain, level, message)                                      \
    do {                                                                    \
        if((level) != SG_INFO) {                                            \
            std::ostringstream sg_log_stream__;                             \
            sg_log_stream__ << message;                                     \
            std::cerr << sg_log_stream__.str();                             \
        }                                                                   \
    } while(0)

#endif
