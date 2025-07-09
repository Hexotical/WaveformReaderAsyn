#include <registryFunction.h>
#include <epicsExport.h>
#include <aSubRecord.h>

static long calcTimeArray(aSubRecord *pasub) {
    long i;
    double clkFreq, sum = 0.0, timeNs;
    // double debug;

    clkFreq = (double)(*(long *)pasub->a);
    // debug   = *(double *)pasub->b; /* Input: Debug Menu */

    /* Calculate the time in ns between each sample */
    timeNs = 1000.0 / (2.0 * clkFreq);

    // if (debug) {
    //     printf("\nX Axis Process Begins");
    //     printf("\nFrequency : %f ", clkFreq);
    //     printf("\nns spacing : %f ", timeNs);
    //     printf("\nCounter at Start: %i ", i);
    // }
    /* Then make an array with each element a distance of `timeNs` apart */
    for (i = 0; i < pasub->nova; i++) {
        ((double *)pasub->vala)[i] = sum;
        sum += timeNs;
        // if (debug) {
        //     if (i%100000 == 0) {
        //         printf("\n  nS : %f ", sum);
        //     }
        // }
    }
    // if (debug) {
    //     printf("\nFrequency : %f ", clkFreq);
    //     printf("\nCounter at End: %i ", i);
    //     printf("\nX Axis Process Ends");

    // }


    return 0; /* process output links */
}

epicsRegisterFunction(calcTimeArray);






// #include <registryFunction.h>
// #include <epicsExport.h>
// #include <aSubRecord.h>

// static long calcTimeArray(aSubRecord *pasub) {
//     long i;
//     double clkFreq, speed, sum = 0.0, timeNs, distance_per_sample = 0.0, length, no_of_time_units = 0;
//     // check data type of this, should be long/short/int?
//     // the time that it takes to get data from the entire range of the fiber, helps determine where extraneous data begins
//     long time_in_range = 0;
//     // double debug;

//     clkFreq = (double)(*(long *)pasub->a);
//     speed = *(double *)pasub->b;
//     length = *(double *)pasub->c;
//     // debug   = *(double *)pasub->b; /* Input: Debug Menu */

//     /* Calculate the time in ns between each sample */
//     timeNs = 1000.0 / (2.0 * clkFreq);

//     // distance traveled in 1 timeNs
//     distance_per_sample = speed * timeNs;
//     printf("WAS THIS EVER EXECUTED??????");
//     // if (debug) {
//     //     printf("\nX Axis Process Begins");
//     //     printf("\nFrequency : %f ", clkFreq);
//     //     printf("\nns spacing : %f ", timeNs);
//     //     printf("\nCounter at Start: %i ", i);
//     // }
//     /* Then make an array with each element a distance of `timeNs` apart */
//     for (i = 0; i < pasub->nova; i++) {
//         ((double *)pasub->vala)[i] = sum;
//         // sum += timeNs;
//         sum += distance_per_sample;
//         // CHECK OFF BY ONE ERROR FOR no_of_time_units
//         if (sum <= length) {time_in_range += timeNs;}
//         // if (debug) {
//         //     if (i%100000 == 0) {
//         //         printf("\n  nS : %f ", sum);
//         //     }
//         // }
//     }
//     // if (debug) {
//     //     printf("\nFrequency : %f ", clkFreq);
//     //     printf("\nCounter at End: %i ", i);
//     //     printf("\nX Axis Process Ends");

//     // }

//     // ((int *)pasub->valb)[0] = no_of_time_units;
//     (double)(*(long *)pasub->b) = time_in_range;

//     return 0; /* process output links */
// }

// epicsRegisterFunction(calcTimeArray);