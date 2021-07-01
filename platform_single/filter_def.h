#ifndef FILTER_DEF_H_
#define FILTER_DEF_H_

#define SPIKE_ADDR    0x00000000
#define APPROX_ADDR   0x00000004
#define DETAIL_ADDR   0x00000008

union word {
  int sint;
  unsigned int uint;
  unsigned char uc[4];
};

#endif
