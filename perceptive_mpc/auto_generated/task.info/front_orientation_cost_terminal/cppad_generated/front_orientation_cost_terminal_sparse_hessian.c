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

void front_orientation_cost_terminal_sparse_hessian(double const *const * in,
                                                    double*const * out,
                                                    struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];
   const double* mult = in[1];

   //dependent variables
   double* hess = out[0];

   // auxiliary variables

}

