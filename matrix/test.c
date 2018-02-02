#include <stdio.h>
#include <stdint.h>
#include <math.h>

uint8_t depthToR[65536];
uint8_t depthToG[65536];
uint8_t depthToB[65536];

// periodicity constants as per the paper (end of sec 3)
double np = 512.0;
double w = 65536.0;
double p = np / w;

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

    double L = (d + 0.5) / w;

    double Ha = fmod(L / (p / 2.0), 2.0);
    if (Ha > 1.0) Ha = 2.0 - Ha;

    // we add 1.0 to avoid taking the modulus of a negative number
    double Hb = fmod((1.0 + L - (p / 4.0)) / (p / 2.0), 2.0);
    if (Hb > 1.0) Hb = 2.0 - Hb;

    // rescale L in order to increase its dynamic range, as in practice the data
    // we get from the truedepth camera seems to only be between 10K and 20K, rather
    // than the 0K-65K range we're considering here...
    L *= 4.0;
    L -= 0.3;

    *dstR = Hb * 255;
    *dstG = Ha * 255;
    *dstB = L * 255;

    printf("%d,%d,%d\n", *dstR, *dstG, *dstB);

    dstR++;
    dstG++;
    dstB++;
  }
}

// Depth to RGB

int m(double L) {
    return int(fmod(floor((4.0 * (L / p)) - 0.5), 4.0));
}

double lzero(double L) {
    return L - fmod(L - (p / 8.0), p) + (((p / 4.0) * (double)m(L)) - (p / 8.0));
}

double delta(double L, double Ha, double Hb) {
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

double d(double L, double Ha, double Hb) {
    return 2000.0 * (lzero(L) + delta(L, Ha, Hb));
}

// Main method

int main() {
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

  uint8_t out[640*480*3];
  dstR = out;
  dstG = out+1;
  dstB = out+2;

  for (int y = 0; y < 480; y++) {
    for (int x = 0; x < 640; x++) {
        
      //uint16_t val = ((*src & 0xff) << 8) | ((*src & 0xff00) >> 8);
      uint16_t val = *src;

      *dstR = depthToR[val];
      *dstG = depthToG[val];
      *dstB = depthToB[val];

      src++;
      dstR += 3;
      dstG += 3;
      dstB += 3;
    }
  }

  fp = fopen("depth-out.raw", "wb");
  if (fp != NULL) {
    fwrite(out, 1, 640*480*3, fp);
  }
  fclose(fp);

  // hit it with GLSL to roundtrip back again...
}

