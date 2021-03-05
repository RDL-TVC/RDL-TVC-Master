#ifndef TVC_Alt_h
#define TVC_Alt_h

#include "WProgram.h"

class TVC_Alt
{
  public:
    TVC_Alt();
    *float getAlt();

  private:
    float lastAlt;
	float alts[2];
};

#endif