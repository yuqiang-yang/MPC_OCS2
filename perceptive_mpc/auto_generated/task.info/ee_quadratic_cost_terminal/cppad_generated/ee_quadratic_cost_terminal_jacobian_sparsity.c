void ee_quadratic_cost_terminal_jacobian_sparsity(unsigned long const** row,
                                                  unsigned long const** col,
                                                  unsigned long* nnz) {
   static unsigned long const rows[60] = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5};
   static unsigned long const cols[60] = {1,2,3,4,5,8,9,10,11,12,1,2,3,4,6,8,9,10,11,12,1,2,3,4,7,8,9,10,11,12,1,2,3,4,8,9,10,11,12,13,1,2,3,4,8,9,10,11,12,13,1,2,3,4,8,9,10,11,12,13};
   *row = rows;
   *col = cols;
   *nnz = 60;
}
