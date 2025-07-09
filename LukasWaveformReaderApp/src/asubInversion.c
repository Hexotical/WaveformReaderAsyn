#include <registryFunction.h>
#include <epicsExport.h>
#include <aSubRecord.h>

static long invertWaveformArray(aSubRecord *pasub) {
    short i, no_of_elements = pasub->nea;
	short *input = (short *)pasub->a;
	short *output  = (short*)pasub->vala;

    for (i = 0; i < no_of_elements; i++) {
        output[i] = input[no_of_elements - i - 1];
    }

    return 0; /* process output links */
}

epicsRegisterFunction(invertWaveformArray);