#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <float.h>

uint8_t depthToR[65536];
uint8_t depthToG[65536];
uint8_t depthToB[65536];

// periodicity constants as per the paper (end of sec 3)
float np = 512.0;
float w = 65536.0;
float p = 0.0078125;// np / w;

// RGB to depth

void buildLUT() {
  // implement depth to RGB as per http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf
  // we also convert to 420p as that's what WebRTC insists on

  uint8_t * dstR = depthToR;
  uint8_t * dstG = depthToG;
  uint8_t * dstB = depthToB;

  // assuming truedepth camera is giving us IEEE 754-2008 half-precision 16-bit floats, this means
  // that positives lie between 0.0 through 65504.0, which when cast to a uint16_t lie between 0 and 65403

  // build our depth->YUV LUT
  for (size_t d = 0; d < 65536; d++) {
    // the paper describes three colour components: L, Ha and Hb, which we map to BGR.
    // L is low-res depth data; H is high-res.

    float L = (d + 0.5) / w;

    float Ha = fmod(L / (p / 2.0), 2.0);
    if (Ha > 1.0) Ha = 2.0 - Ha;

    // we add 1.0 to avoid taking the modulus of a negative number
    float Hb = fmod((1.0 + L - (p / 4.0)) / (p / 2.0), 2.0);
    if (Hb > 1.0) Hb = 2.0 - Hb;

    // rescale L in order to increase its dynamic range, as in practice the data
    // we get from the truedepth camera seems to only be between 10K and 20K, rather
    // than the 0K-65K range we're considering here...
    L *= 4.0;
    L -= 0.3;

    *dstR = Hb * 255;
    *dstG = Ha * 255;
    *dstB = L * 255;

    //printf("%d,%d,%d\n", *dstR, *dstG, *dstB);

    dstR++;
    dstG++;
    dstB++;
  }
}

// Depth to RGB

int m(float L) {
    return (int)(fmod(floor((4.0 * (L / p)) - 0.5), 4.0));
}

float lzero(float L) {
    return L - fmod(L - (p / 8.0), p) + (((p / 4.0) * (float)m(L)) - (p / 8.0));
}

float delta(float L, float Ha, float Hb) {
    int mL = m(L);
    if (mL == 0) {
        return (p / 2.0) * Ha;
    } else if (mL == 1) {
        return (p / 2.0) * Hb;
    } else if (mL == 2) {
        return (p / 2.0) * (1.0 - Ha);
    } else if (mL == 3) {
        return (p / 2.0) * (1.0 - Hb);
    }
}

float d(float L, float Ha, float Hb) {
    printf("%f,%f,%f => %f\n", L, Ha, Hb, lzero(L) + delta(L, Ha, Hb));
    return w * (lzero(L) + delta(L, Ha, Hb));
}

float half2float(uint16_t d) {
  uint32_t out = ((((uint32_t)d & 0x8000) << 16) | 
                 ((((uint32_t)d & 0x7c00) + 0x1C000) << 13) | 
                  (((uint32_t)d & 0x03FF) << 13) );
  return *(float *)&out;
}

int main() {
  buildLUT();

  // read our file
  FILE * fp = fopen("depth-buffer.raw", "rb");
  size_t len = 2*640*480;
  uint8_t buf[len + 1];
  if (fp != NULL) {
    size_t newLen = fread(buf, sizeof(uint8_t), len, fp);
    if (ferror( fp ) != 0) {
      fputs("Error reading file", stderr);
    }
    fclose(fp);
  }

  uint16_t * src = (uint16_t *) buf;
  uint8_t out[640*480*2];
  uint16_t * dst = (uint16_t *)out;

  uint16_t minD = USHRT_MAX;
  uint16_t maxD = 0;
  float minF = FLT_MAX;
  float maxF = FLT_MIN;

  for (int y = 0; y < 480; y++) {
    for (int x = 0; x < 640; x++) {
        
      //uint16_t val = ((*src & 0xff) << 8) | ((*src & 0xff00) >> 8);
      uint16_t val = *src;
      float f = half2float(val);
      printf("%f\n", f);

      if (val < minD) minD = val;
      if (val > maxD) maxD = val;

      if (f < minF) minF = f;
      if (f > maxF) maxF = f;

      *dst = (uint16_t) d((float)depthToB[val] / 255.0, (float)depthToG[val] / 255.0, (float)depthToR[val] / 255.0);

      src++;
      dst++;
    }
  }

  printf("minD = %d, maxD = %d\n", minD, maxD);
  printf("minF = %f, maxF = %f\n", minF, maxF);

  fp = fopen("depth-buffer-out.raw", "wb");
  if (fp != NULL) {
    fwrite(out, 1, 640*480*2, fp);
  }
  fclose(fp);
}

