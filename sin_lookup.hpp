#ifndef SIN_LOOKUP_HPP
#define SIN_LOOKUP_HPP

#include <stdint.h>
#include <utility>

// Returns absolute fixed point sine of 8 bit 'circle' and sign as pair, i.e. y = 65535 * sin(2*pi*x/256), first = |round(y)|, second = y >= 0
std::pair<uint16_t, bool> sin_lookup(unsigned x);

#endif