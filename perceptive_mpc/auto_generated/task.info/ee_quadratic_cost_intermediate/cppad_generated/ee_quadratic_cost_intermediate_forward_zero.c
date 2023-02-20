#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void ee_quadratic_cost_intermediate_forward_zero(double const *const * in,
                                                 double*const * out,
                                                 struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[37];

   v[0] = 2. * x[2];
   v[1] = v[0] * x[1];
   v[2] = 2. * x[3];
   v[3] = v[2] * x[4];
   v[4] = v[1] - v[3];
   v[5] = cos(x[10]);
   v[6] = sin(x[9]);
   v[7] = sin(x[10]);
   v[8] = 0.0996 * v[7];
   v[9] = cos(x[9]);
   v[10] = cos(x[8]);
   v[11] = sin(x[11]);
   v[12] = cos(x[11]);
   v[13] = sin(x[12]);
   v[14] = sin(x[8]);
   v[15] = cos(x[12]);
   v[8] = ((-0.0996 * v[5] * v[6] - v[8] * v[9]) * v[10] * v[11] + (0.0996 * v[5] * v[9] - v[8] * v[6]) * v[10] * v[12]) * v[13] + (0.0997 * v[7] * v[6] - 0.0997 * v[5] * v[9]) * v[10] * v[11] + (-0.0997 * v[5] * v[6] - 0.0997 * v[7] * v[9]) * v[10] * v[12] + ((0.425 + 0.392 * v[5]) * v[9] - 0.392 * v[7] * v[6]) * v[10] - 0.0996 * v[14] * v[15] - 0.1333 * v[14];
   v[16] = 0.0996 * v[7];
   v[16] = 0.0996 * v[10] * v[15] + ((-0.0996 * v[5] * v[6] - v[16] * v[9]) * v[14] * v[11] + (0.0996 * v[5] * v[9] - v[16] * v[6]) * v[14] * v[12]) * v[13] + (0.0997 * v[7] * v[6] - 0.0997 * v[5] * v[9]) * v[14] * v[11] + (-0.0997 * v[5] * v[6] - 0.0997 * v[7] * v[9]) * v[14] * v[12] + ((0.425 + 0.392 * v[5]) * v[9] - 0.392 * v[7] * v[6]) * v[14] + 0.1333 * v[10];
   v[17] = -1. * v[8] + 2.6794896412774e-08 * v[16];
   v[18] = v[2] * x[3];
   v[19] = v[0] * x[2];
   v[20] = 1 - v[18] - v[19];
   v[16] = 0.224311 + 1. * v[16] + 2.6794896412774e-08 * v[8];
   v[8] = v[2] * x[1];
   v[0] = v[0] * x[4];
   v[21] = v[8] + v[0];
   v[22] = 0.0997 * v[7];
   v[22] = 0.7625 + (0.0997 * v[5] * v[6] + v[22] * v[9]) * v[11] + ((0.0996 * v[7] * v[6] - 0.0996 * v[5] * v[9]) * v[11] + (-0.0996 * v[5] * v[6] - 0.0996 * v[7] * v[9]) * v[12]) * v[13] + (v[22] * v[6] - 0.0997 * v[5] * v[9]) * v[12] + (-0.392 * v[5] - 0.425) * v[6] - 0.392 * v[7] * v[9];
   y[8] = 3.16227766016838 * (x[26] - v[4] * v[17] - v[20] * v[16] - x[5] - v[21] * v[22]);
   v[23] = 2. * x[1];
   v[24] = v[23] * x[1];
   v[18] = 1 - v[18] - v[24];
   v[3] = v[1] + v[3];
   v[2] = v[2] * x[2];
   v[23] = v[23] * x[4];
   v[1] = v[2] - v[23];
   y[9] = 3.16227766016838 * (x[27] - v[18] * v[17] - v[3] * v[16] - x[6] - v[1] * v[22]);
   v[23] = v[2] + v[23];
   v[0] = v[8] - v[0];
   v[24] = 1 - v[19] - v[24];
   y[10] = 3.16227766016838 * (x[28] - v[23] * v[17] - v[0] * v[16] - x[7] - v[24] * v[22]);
   v[22] = v[7] * v[9];
   v[9] = v[5] * v[9];
   v[7] = v[7] * v[6];
   v[16] = v[9] - v[7];
   v[17] = v[16] * v[10] * v[12];
   v[19] = (((0 - v[5]) * v[6] - v[22]) * v[10] * v[11] + v[17]) * v[13] - v[14] * v[15];
   v[8] = (0 - v[5]) * v[6] - v[22];
   v[2] = v[16] * v[14] * v[12];
   v[25] = (v[8] * v[14] * v[11] + v[2]) * v[13] + v[10] * v[15];
   v[26] = -1. * v[19] + 2.6794896412774e-08 * v[25];
   v[25] = 2.6794896412774e-08 * v[19] + 1. * v[25];
   v[7] = v[7] - v[9];
   v[9] = v[7] * v[11];
   v[19] = (v[9] + ((0 - v[5]) * v[6] - v[22]) * v[12]) * v[13];
   v[27] = v[23] * v[26] + v[0] * v[25] + v[24] * v[19];
   v[28] = v[7] * v[10];
   v[29] = v[28] * v[11];
   v[30] = sin(x[13]);
   v[31] = v[14] * v[13];
   v[32] = cos(x[13]);
   v[17] = (v[29] + ((0 - v[5]) * v[6] - v[22]) * v[10] * v[12]) * v[30] + (v[31] + (((0 - v[5]) * v[6] - v[22]) * v[10] * v[11] + v[17]) * v[15]) * v[32];
   v[33] = v[7] * v[14];
   v[34] = v[33] * v[11];
   v[13] = v[10] * v[13];
   v[2] = (v[34] + ((0 - v[5]) * v[6] - v[22]) * v[14] * v[12]) * v[30] + ((((0 - v[5]) * v[6] - v[22]) * v[14] * v[11] + v[2]) * v[15] - v[13]) * v[32];
   v[35] = -1 * (-1. * v[17] + 2.6794896412774e-08 * v[2]);
   v[2] = -1 * (2.6794896412774e-08 * v[17] + 1. * v[2]);
   v[17] = v[5] * v[6] + v[22];
   v[7] = v[17] * v[11] + v[7] * v[12];
   v[9] = -1 * (v[7] * v[30] + (v[9] + ((0 - v[5]) * v[6] - v[22]) * v[12]) * v[15] * v[32]);
   v[36] = v[4] * v[35] + v[20] * v[2] + v[21] * v[9];
   v[31] = ((v[17] * v[10] * v[11] + v[28] * v[12]) * v[15] - v[31]) * v[30] + (v[29] + v[8] * v[10] * v[12]) * v[32];
   v[13] = (v[13] + (v[17] * v[14] * v[11] + v[33] * v[12]) * v[15]) * v[30] + (v[34] + ((0 - v[5]) * v[6] - v[22]) * v[14] * v[12]) * v[32];
   v[34] = -1 * (-1. * v[31] + 2.6794896412774e-08 * v[13]);
   v[13] = -1 * (2.6794896412774e-08 * v[31] + 1. * v[13]);
   v[7] = -1 * ((v[16] * v[11] + v[17] * v[12]) * v[15] * v[30] + v[7] * v[32]);
   v[17] = v[18] * v[34] + v[3] * v[13] + v[1] * v[7];
   v[32] = v[23] * v[34] + v[0] * v[13] + v[24] * v[7];
   v[30] = v[18] * v[26] + v[3] * v[25] + v[1] * v[19];
   v[16] = v[32] - v[30];
   v[19] = v[4] * v[26] + v[20] * v[25] + v[21] * v[19];
   v[24] = v[23] * v[35] + v[0] * v[2] + v[24] * v[9];
   v[0] = v[19] - v[24];
   if( v[36] > v[17] ) {
      v[23] = v[16];
   } else {
      v[23] = v[0];
   }
   v[25] = 0 - v[17];
   v[9] = v[18] * v[35] + v[3] * v[2] + v[1] * v[9];
   v[7] = v[4] * v[34] + v[20] * v[13] + v[21] * v[7];
   v[13] = v[9] - v[7];
   if( v[36] > v[17] ) {
      v[34] = 1 + v[36] - v[17] - v[27];
   } else {
      v[34] = 1 + v[17] - v[36] - v[27];
   }
   if( v[36] < 0 - v[17] ) {
      v[21] = 1 + v[27] - v[36] - v[17];
   } else {
      v[21] = 1 + v[36] + v[17] + v[27];
   }
   if( v[27] < 0 ) {
      v[21] = v[34];
   } else {
      v[21] = v[21];
   }
   if( v[36] < v[25] ) {
      v[34] = v[13];
   } else {
      v[34] = v[21];
   }
   if( v[27] < 0 ) {
      v[34] = v[23];
   } else {
      v[34] = v[34];
   }
   v[23] = 0.5 / sqrt(v[21]);
   v[34] = v[34] * v[23];
   v[7] = v[9] + v[7];
   if( v[36] > v[17] ) {
      v[9] = v[7];
   } else {
      v[9] = v[21];
   }
   v[30] = v[32] + v[30];
   if( v[36] < 0 - v[17] ) {
      v[0] = v[30];
   } else {
      v[0] = v[0];
   }
   if( v[27] < 0 ) {
      v[0] = v[9];
   } else {
      v[0] = v[0];
   }
   v[0] = v[0] * v[23];
   if( v[36] > v[17] ) {
      v[7] = v[21];
   } else {
      v[7] = v[7];
   }
   v[24] = v[19] + v[24];
   if( v[36] < v[25] ) {
      v[16] = v[24];
   } else {
      v[16] = v[16];
   }
   if( v[27] < 0 ) {
      v[16] = v[7];
   } else {
      v[16] = v[16];
   }
   v[16] = v[16] * v[23];
   if( v[36] > v[17] ) {
      v[24] = v[24];
   } else {
      v[24] = v[30];
   }
   if( v[36] < v[25] ) {
      v[21] = v[21];
   } else {
      v[21] = v[13];
   }
   if( v[27] < 0 ) {
      v[21] = v[24];
   } else {
      v[21] = v[21];
   }
   v[21] = v[21] * v[23];
   y[11] = 3.16227766016838 * (v[34] * x[22] + v[0] * x[24] - x[25] * v[16] - v[21] * x[23]);
   y[12] = 3.16227766016838 * (v[34] * x[23] + v[21] * x[22] - x[25] * v[0] - v[16] * x[24]);
   y[13] = 3.16227766016838 * (v[34] * x[24] + v[16] * x[23] - x[25] * v[21] - v[0] * x[22]);
   // dependent variables without operations
   y[0] = x[14];
   y[1] = x[15];
   y[2] = x[16];
   y[3] = x[17];
   y[4] = x[18];
   y[5] = x[19];
   y[6] = x[20];
   y[7] = x[21];
}

