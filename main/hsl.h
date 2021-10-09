#ifndef HSL_H_
#define HSL_H_

#include <stdint.h>

float hueToRgb(float p, float q, float t);
void hslToRgb(float h, float s, float l, float rgb[]);

#endif /* HSL_H_ */

