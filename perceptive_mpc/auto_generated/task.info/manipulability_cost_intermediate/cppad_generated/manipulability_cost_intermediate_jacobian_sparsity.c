void manipulability_cost_intermediate_jacobian_sparsity(unsigned long const** row,
                                                        unsigned long const** col,
                                                        unsigned long* nnz) {
   static unsigned long const rows[5] = {0,0,0,0,0};
   static unsigned long const cols[5] = {8,9,10,11,12};
   *row = rows;
   *col = cols;
   *nnz = 5;
}
