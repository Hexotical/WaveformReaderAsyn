#include "WaveformReader.h"

/**
 * Computes and displays the results of a fast fourier transform on a section of the waveform data 
 * that contains the peak value
 * Uses the fftw library (https://www.fftw.org/)
 * 
 * @param waveformIndex index of waveform: 0, 1, and 2, for WAVEFORM:0, WAVEFORM:1, and WAVEFORM:2 respectively
 */
void WaveformReader::fft(int waveformIndex)
{
  int maxIndex = findMaxIndex(waveformIndex);
  int low, high;
  const int LOWER_LIMIT = 5;
  findRange(low, high, maxIndex, LOWER_LIMIT, waveformIndex);
  // Define the length of the complex arrays
  int n = high - low + 1;
  // Dynamically allocate the array because the size can change
  // Input array
  fftw_complex* x = new fftw_complex[n]; // This is equivalent to: double x[n][2];
  // Output array
  fftw_complex* y = new fftw_complex[n];
  // Samping frequency
  double sampling_frequency = 3.57142857143e8; 

  // Fill the first array with data from waveformData
  for (int i = 0; i < n; i++)
  {
      x[i][REAL] = waveform_map[(waveform_param_indices[waveformIndex])][i + low];
      x[i][IMAG] = 0;
  }
  //Plant the FFT and execute it
  fftw_plan plan = fftw_plan_dft_1d(n, x, y, FFTW_FORWARD, FFTW_ESTIMATE);
  fftw_execute(plan);
  //Do some cleaning
  fftw_destroy_plan(plan);
  fftw_cleanup();

  // Display the results
  int choice;
  std::string extra;
  std::cout << "Would you like to: \n1) Display the results of the fast fourier transform\n2) Output the results to a csv and display a graph of the same" << std::endl;
  while (true)
  {
    std::cout << "Enter choice (1 or 2): ";
    std::cin >> choice;

    if (std::cin.fail())
    {
      std::cout << "Invalid choice, try again." << std::endl;
      std::cin.clear();
      std::cin.ignore();
      std::getline(std::cin, extra); 
    }

    else if (choice == 1)
    {
      std::cout << "FFT = " << std::endl;
      double frequency;
      std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
      std::cout << "|    No.    | FREQUENCY (in Hz) |             FFT RESULT             |     MAGNITUDE     | PHASE (in radians) |" << std::endl;
      std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
      for (int i = 0; i < n; i++)
      {
          frequency = (i * sampling_frequency) / n;
          if (y[i][IMAG] < 0)
          {
            std::cout << "|" << std::setw(11) << i + 1 << "|" << std::setw(19) << std::right << frequency << "|" << std::setw(16) << y[i][REAL] << " - " << std::setw(16) << std::left << abs(y[i][IMAG]) << "i";
          }
          else
          {
            std::cout << "|" << std::setw(11) << i + 1 << "|" << std::setw(19) << std::right << frequency << "|" << std::setw(16) << y[i][REAL] << " + " << std::setw(16) << std::left << y[i][IMAG] << "i" ;
          }
          std::cout << "|" << std::setw(19) << std::right << sqrt(pow(y[i][REAL], 2) + pow((y[i][IMAG]), 2));
          std::cout << "|" << std::setw(20) << std::right << atan2(y[i][IMAG], y[i][REAL]) << "|" << std::endl;
          std::cout << "---------------------------------------------------------------------------------------------------------------" << std::endl;
      }
      break;
    }

    else if (choice == 2)
    {
      double frequency;
      for (int i = 0; i < n; i++)
      {
        frequency = (i * sampling_frequency) / n;
        std::cout << frequency << "," << sqrt(pow(y[i][REAL], 2) + pow((y[i][IMAG]), 2)) << "\n";
      }
      std::cout << "\nRun the plot.py script to generate the graph.\n" << std::endl;
      break;
    }
    else 
    {
      std::cout << "Invalid choice, try again." << std::endl;
      std::cin.clear();
      std::cin.ignore();
    }  
  }
}

//-------------------------------------------------------------------------------------
//IOCSH command
//-------------------------------------------------------------------------------------

static void fourierTransform(int waveformIndex)
{
  WaveformReader* bayManager = WaveformReader::getPortDriver();
  bayManager->fft(waveformIndex);
  return;
}

static const iocshArg fftArg0 = {"waveformIndex", iocshArgInt};
static const iocshArg * const fftArgs[] = {&fftArg0};
static const iocshFuncDef fftFuncDef = {"fourierTransform", 1, fftArgs};
static void fftCallFunc(const iocshArgBuf *args)
{
  fourierTransform(args[0].ival);
}

void fourierTransformRegister(void)
{
  iocshRegister(&fftFuncDef, fftCallFunc);
}

extern "C" {
  epicsExportRegistrar(fourierTransformRegister);
}