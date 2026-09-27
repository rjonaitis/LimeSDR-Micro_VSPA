// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
// Copyright 2017 - 2025   NXP Semiconductors

// ===========================================================================
//! @file            ditfft.h
//! @brief           DIT FFT library interface definitions.
//! @ingroup         GROUP_FFT
//!
//! The ditfft.h header defines the DIT FFT library application programming
//! interface.
//!
// ===========================================================================

#ifndef __DITFFT_H__
#define __DITFFT_H__

#include <stddef.h>

// -----------------------------------------------------------------------------
//! @defgroup		GROUP_FFT FFT Library
//! @brief          FFT function library
//!
//! This library contains function prototypes for radix 2 FFTs
//! The FFT Library provides the following functions:
//!   - 128 pt DIT FFT HFL:
//!      - fftDIT128_hfl(): 16-bit floating point in, 16-bit fixed-point out 128pt DIT FFT
//!   - 128 pt DIT IFFT HFL:
//!      - ifftDIT128_hfl(): 16-bit floating point in, 16-bit fixed-point out 128pt DIT IFFT
//!   - 128 pt DIT FFT SFL:
//!      - fftDIT128_sfl(): 16-bit floating point in, 32-bit floating-point out 128pt DIT FFT
//!   - 128 pt DIT IFFT SFL:
//!      - ifftDIT128_sfl(): 16-bit floating point in, 32-bit floating-point intermediate, 16-bit fixed-point out 128pt DIT IFFT
//!   - 512 pt DIT FFT HFL:
//!      - fftDIT512_hfl(): 16-bit floating point in, 16-bit fixed-point out 512pt DIT FFT
//!   - 512 pt DIT IFFT HFL:
//!      - ifftDIT512_hfl(): 16-bit floating point in, 16-bit fixed-point out 512pt DIT IFFT
//!   - 512 pt DIT FFT SFL:
//!      - fftDIT512_sfl(): 16-bit floating point in, 32-bit floating-point out 512pt DIT FFT
//!   - 512 pt DIT IFFT SFL:
//!      - ifftDIT512_sfl(): 16-bit floating point in, 32-bit floating-point intermediate, 16-bit fixed-point out 512pt DIT IFFT
//!   - 512 pt DIT FFT SFL SFL:
//!      - fftDIT512_sflsfl(): 32-bit floating point in, 32-bit floating-point out 512pt DIT FFT
//!   - 512 pt DIT IFFT SFL SFL:
//!      - ifftDIT512_sflsfl(): 32-bit floating point in, 32-bit floating-point out 512pt DIT IFFT
//!   - 1024 pt DIT FFT HFL:
//!      - fftDIT1024_hfl(): 16-bit floating point in, 16-bit fixed-point out 1024pt DIT FFT
//!   - 1024 pt DIT IFFT HFL:
//!      - ifftDIT1024_hfl(): 16-bit floating point in, 16-bit fixed-point out 1024pt DIT IFFT
//!   - 1024 pt DIT FFT SFL:
//!      - fftDIT1024_sfl(): 16-bit floating point in, 32-bit floating-point out 1024pt DIT FFT
//!   - 1024 pt DIT IFFT SFL:
//!      - ifftDIT1024_sfl(): 16-bit floating point in, 32-bit floating-point intermediate, 16-bit fixed-point out 1024pt DIT IFFT
//!   - 2048 pt DIT FFT HFL:
//!      - fftDIT2048_hfl(): 16-bit floating point in, 16-bit fixed-point out 2048pt DIT FFT
//!   - 2048 pt DIT IFFT HFL:
//!      - ifftDIT2048_hfl(): 16-bit floating point in, 16-bit fixed-point out 2048pt DIT IFFT
//!   - 2048 pt DIT FFT SFL:
//!      - fftDIT2048_sfl(): 16-bit floating point in, 32-bit floating-point out 2048pt DIT FFT
//!   - 2048 pt DIT IFFT SFL:
//!      - ifftDIT2048_sfl(): 16-bit floating point in, 32-bit floating-point intermediate, 16-bit fixed-point out 2048pt DIT IFFT
//!
//! @{
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// variable
// -----------------------------------------------------------------------------
extern unsigned int ifftDITScratchBuffer[4096] _VSPA_VECTOR_ALIGN;

// ---------------------------------------------------------------------------
//! @brief           128 pt HFX FFT using decimation in time approach for 16 bit floating point input data, 16 bit fixed point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 128 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = (1/N)*fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT128_hfl(vspa_complex_float16 const *pIn, // Input buffer pointer for holding half floating point complex values
                          vspa_complex_fixed16 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           128 pt HFX IFFT using decimation in time approach for 16 bit floating point input data, 16 bit fixed point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 128 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT128_hfl(vspa_complex_float16 const *pIn, // Circular input buffer pointer for holding half precision complex values
               vspa_complex_fixed16 *pOut // Output buffer pointer for holding half precision complex values.
);

// ---------------------------------------------------------------------------
//! @brief           128 pt HFX FFT using decimation in time approach for 16 bit floating point input data, 32 bit floating point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 128 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT128_sfl(vspa_complex_float16 const *pIn, // Input buffer pointer for holding half floating point complex values
                          vspa_complex_float32 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           128 pt HFX IFFT using decimation in time approach for 16 bit floating point input data, 32 bit floating point
//! intermediate data, 16 bit fixed point output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! Additionally, this function uses scratch memory of size 256 words at ifftDITScratchBuffer
//! This function calculates 128 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT128_sfl(vspa_complex_float16 const *pIn, // Circular input buffer pointer for holding half precision complex values
               vspa_complex_fixed16 *pOut // Output buffer pointer for holding half precision complex values.
);

// ---------------------------------------------------------------------------
//! @brief           512 pt HFX FFT using decimation in time approach for 16 bit floating point input data, 16 bit fixed point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 512 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = (1/N)*fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT512_hfl(vspa_complex_float16 const *pIn, // Input buffer pointer for holding half floating point complex values
                          vspa_complex_fixed16 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           512 pt HFX IFFT using decimation in time approach for 16 bit floating point input data, 16 bit fixed point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 512 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT512_hfl(vspa_complex_float16 const *pIn, // Circular input buffer pointer for holding half precision complex values
               vspa_complex_fixed16 *pOut // Output buffer pointer for holding half precision complex values.
);

// ---------------------------------------------------------------------------
//! @brief           512 pt HFX FFT using decimation in time approach for 16 bit floating point input data, 32 bit floating point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 512 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT512_sfl(vspa_complex_float16 const *pIn, // Input buffer pointer for holding half floating point complex values
                          vspa_complex_float32 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           512 pt HFX IFFT using decimation in time approach for 16 bit floating point input data, 32 bit floating point
//! intermediate data, 16 bit fixed point output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! Additionally, this function uses scratch memory of size 1024 words at ifftDITScratchBuffer
//! This function calculates 512 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT512_sfl(vspa_complex_float16 const *pIn, // Circular input buffer pointer for holding half precision complex values
               vspa_complex_fixed16 *pOut // Output buffer pointer for holding half precision complex values.
);

// ---------------------------------------------------------------------------
//! @brief           512 pt SFL FFT using decimation in time approach for 32 bit floating point input data, 32 bit floating point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 512 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT512_sflsfl(vspa_complex_float32 const *pIn, // Input buffer pointer for holding half floating point complex values
                             vspa_complex_float32 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           512 pt SFL IFFT using decimation in time approach for 32 bit floating point input data, 32 bit floating point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 512 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = 512*ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT512_sflsfl(vspa_complex_float32 const *pIn, // Circular input buffer pointer for holding half precision complex values
                  vspa_complex_float32 *pOut // Output buffer pointer for holding half precision complex values.
);

// ---------------------------------------------------------------------------
//! @brief           1024 pt HFX FFT using decimation in time approach for 16 bit floating point input data, 16 bit fixed point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 1024 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = (1/N)*fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT1024_hfl(vspa_complex_float16 const *pIn, // Input buffer pointer for holding half floating point complex values
                           vspa_complex_fixed16 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           1024 pt HFX IFFT using decimation in time approach for 16 bit floating point input data, 16 bit fixed point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 1024 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT1024_hfl(vspa_complex_float16 const *pIn, // Circular input buffer pointer for holding half precision complex values
                vspa_complex_fixed16 *pOut // Output buffer pointer for holding half precision complex values.
);

// ---------------------------------------------------------------------------
//! @brief           1024 pt HFX FFT using decimation in time approach for 16 bit floating point input data, 32 bit floating point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 1024 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT1024_sfl(vspa_complex_float16 const *pIn, // Input buffer pointer for holding half floating point complex values
                           vspa_complex_float32 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           1024 pt HFX IFFT using decimation in time approach for 16 bit floating point input data, 32 bit floating point
//! intermediate data, 16 bit fixed point output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! Additionally, this function uses scratch memory of size 2048 words at ifftDITScratchBuffer
//! This function calculates 1024 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT1024_sfl(vspa_complex_float16 const *pIn, // Circular input buffer pointer for holding half precision complex values
                vspa_complex_fixed16 *pOut // Output buffer pointer for holding half precision complex values.
);

// ---------------------------------------------------------------------------
//! @brief           2048 pt HFX FFT using decimation in time approach for 16 bit floating point input data, 16 bit fixed point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 2048 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = (1/N)*fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT2048_hfl(vspa_complex_float16 const *pIn, // Input buffer pointer for holding half floating point complex values
                           vspa_complex_fixed16 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           2048 pt HFX IFFT using decimation in time approach for 16 bit floating point input data, 16 bit fixed point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 2048 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT2048_hfl(vspa_complex_float16 const *pIn, // Circular input buffer pointer for holding half precision complex values
                vspa_complex_fixed16 *pOut // Output buffer pointer for holding half precision complex values.
);

// ---------------------------------------------------------------------------
//! @brief           2048 pt HFX FFT using decimation in time approach for 16 bit floating point input data, 32 bit floating point
//! output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! This function calculates 2048 pt FFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = fft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void fftDIT2048_sfl(vspa_complex_float16 const *pIn, // Input buffer pointer for holding half floating point complex values
                           vspa_complex_float32 *pOut // Output buffer pointer for holding half fixed point complex values.
);

// ---------------------------------------------------------------------------
//! @brief           2048 pt HFX IFFT using decimation in time approach for 16 bit floating point input data, 32 bit floating point
//! intermediate data, 16 bit fixed point output data
//!
//! @param[in]       pIn   Input buffer address
//! @param[out]      pOut  Output buffer address
//! @return          Void.
//! @cycle
//! @stack         	 0
//!
//! Additionally, this function uses scratch memory of size 4096 words at ifftDITScratchBuffer
//! This function calculates 2048 pt IFFT using decimation in time (DIT approach)
//! Equivalent MATLAB command: y = ifft(x)
//!
//! @attention       This function can operate in-place.
//! @attention       The output buffer must be vector-aligned.
// ---------------------------------------------------------------------------
extern void
ifftDIT2048_sfl(vspa_complex_float16 const *pIn, // Circular input buffer pointer for holding half precision complex values
                vspa_complex_fixed16 *pOut // Output buffer pointer for holding half precision complex values.
);

//! @} GROUP_FFT

#endif // __DITFFT_H__
