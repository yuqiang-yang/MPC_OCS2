void manipulability_cost_intermediate_hessian_sparsity(unsigned long const** row,
                                                       unsigned long const** col,
                                                       unsigned long* nnz) {
   static unsigned long const rows[15] = {8,8,8,8,8,9,9,9,9,10,10,10,11,11,12};
   static unsigned long const cols[15] = {8,9,10,11,12,9,10,11,12,10,11,12,11,12,12};
   *row = rows;
   *col = cols;
   *nnz = 15;
}
