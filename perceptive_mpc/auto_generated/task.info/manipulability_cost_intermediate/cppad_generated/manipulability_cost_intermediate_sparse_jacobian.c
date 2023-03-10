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

void manipulability_cost_intermediate_sparse_jacobian(double const *const * in,
                                                      double*const * out,
                                                      struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[351];

   v[0] = sin(x[10]);
   v[1] = 0.0997 * v[0];
   v[2] = sin(x[9]);
   v[3] = cos(x[10]);
   v[4] = 0.0997 * v[3];
   v[5] = cos(x[9]);
   v[6] = v[1] * v[2] - v[4] * v[5];
   v[7] = cos(x[8]);
   v[8] = v[6] * v[7];
   v[9] = cos(x[11]);
   v[10] = 0.0997 * v[3];
   v[11] = 0.0997 * v[0];
   v[12] = v[10] * v[2] + v[11] * v[5];
   v[13] = v[12] * v[7];
   v[14] = sin(x[11]);
   v[15] = -0.392 * v[3] - 0.425;
   v[16] = 0.392 * v[0];
   v[17] = v[15] * v[2] - v[16] * v[5];
   v[18] = v[8] * v[9] + v[13] * v[14] + v[17] * v[7];
   v[19] = 0.0997 * v[3];
   v[20] = 0.0997 * v[0];
   v[21] = v[19] * v[2] + v[20] * v[5];
   v[22] = sin(x[8]);
   v[23] = v[21] * v[22];
   v[24] = v[19] * v[5] - v[20] * v[2];
   v[25] = v[24] * v[22];
   v[26] = 0.392 * v[0];
   v[27] = -0.392 * v[3] - 0.425;
   v[28] = v[26] * v[2] + v[27] * v[5];
   v[29] = v[23] * v[9] + v[25] * v[14] + v[28] * v[22] - 0.1333 * v[7];
   v[30] = 0.0997 * v[0];
   v[31] = 0.0997 * v[3];
   v[32] = v[30] * v[2] - v[31] * v[5];
   v[33] = v[32] * v[22];
   v[34] = v[31] * v[2] + v[30] * v[5];
   v[35] = v[34] * v[22];
   v[36] = -0.392 * v[3] - 0.425;
   v[37] = 0.392 * v[0];
   v[38] = v[36] * v[2] - v[37] * v[5];
   v[39] = v[33] * v[9] + v[35] * v[14] + v[38] * v[22];
   v[40] = -0.0997 * v[3];
   v[41] = 0.0997 * v[0];
   v[42] = v[40] * v[2] - v[41] * v[5];
   v[43] = v[42] * v[7];
   v[44] = 0.0997 * v[0];
   v[45] = 0.0997 * v[3];
   v[46] = v[44] * v[2] - v[45] * v[5];
   v[47] = v[46] * v[7];
   v[48] = 0.425 + 0.392 * v[3];
   v[49] = 0.392 * v[0];
   v[50] = v[48] * v[5] - v[49] * v[2];
   v[51] = v[43] * v[9] + v[47] * v[14] + v[50] * v[7] - 0.1333 * v[22];
   v[52] = v[18] * v[29] + v[39] * v[51];
   v[53] = -1 * v[52];
   v[54] = v[7] * v[7];
   v[55] = 0 - v[22];
   v[56] = v[55] * v[55];
   v[57] = 0.0997 * v[3];
   v[58] = 0.0997 * v[0];
   v[59] = v[57] * v[2] + v[58] * v[5];
   v[60] = v[57] * v[5] - v[58] * v[2];
   v[61] = 0.392 * v[0];
   v[62] = 0.392 * v[3];
   v[63] = v[59] * v[9] + v[60] * v[14] + v[61] * v[2] - v[62] * v[5];
   v[64] = 0.0997 * v[3];
   v[65] = 0.0997 * v[0];
   v[66] = v[64] * v[2] + v[65] * v[5];
   v[67] = 0.0997 * v[3];
   v[68] = 0.0997 * v[0];
   v[69] = v[67] * v[5] - v[68] * v[2];
   v[70] = 0.392 * v[0];
   v[71] = -0.392 * v[3] - 0.425;
   v[72] = v[66] * v[9] + v[69] * v[14] + v[70] * v[2] + v[71] * v[5];
   v[73] = v[63] * v[72];
   v[74] = 0.0997 * v[0];
   v[75] = 0.0997 * v[3];
   v[76] = v[74] * v[2] - v[75] * v[5];
   v[77] = v[76] * v[22];
   v[78] = v[75] * v[2] + v[74] * v[5];
   v[79] = v[78] * v[22];
   v[80] = -0.392 * v[3];
   v[81] = 0.392 * v[0];
   v[82] = v[80] * v[2] - v[81] * v[5];
   v[83] = v[77] * v[9] + v[79] * v[14] + v[82] * v[22];
   v[84] = v[83] * v[39];
   v[85] = 0.0997 * v[0];
   v[86] = 0.0997 * v[3];
   v[87] = v[85] * v[2] - v[86] * v[5];
   v[88] = v[87] * v[7];
   v[89] = v[86] * v[2] + v[85] * v[5];
   v[90] = v[89] * v[7];
   v[91] = -0.392 * v[3];
   v[92] = 0.392 * v[0];
   v[93] = v[91] * v[2] - v[92] * v[5];
   v[94] = v[88] * v[9] + v[90] * v[14] + v[93] * v[7];
   v[95] = v[94] * v[18];
   v[96] = v[54] + v[56] + v[73] + v[84] + v[95];
   v[97] = -1 * v[96];
   v[98] = v[94] * v[29] + v[83] * v[51];
   v[99] = 0.0997 * v[3];
   v[100] = 0.0997 * v[0];
   v[101] = v[99] * v[5] - v[100] * v[2];
   v[102] = 0.0997 * v[3];
   v[103] = v[102] * v[2] + v[100] * v[5];
   v[104] = v[101] * v[14] + v[103] * v[9];
   v[105] = 0.0997 * v[3];
   v[106] = 0.0997 * v[0];
   v[107] = v[105] * v[2] + v[106] * v[5];
   v[108] = v[107] * v[22];
   v[109] = 0.0997 * v[0];
   v[110] = 0.0997 * v[3];
   v[111] = v[109] * v[2] - v[110] * v[5];
   v[112] = v[111] * v[22];
   v[113] = v[108] * v[14] + v[112] * v[9];
   v[114] = v[45] * v[2] + v[44] * v[5];
   v[115] = v[114] * v[7];
   v[116] = 0.0997 * v[0];
   v[117] = 0.0997 * v[3];
   v[118] = v[116] * v[2] - v[117] * v[5];
   v[119] = v[118] * v[7];
   v[120] = v[115] * v[14] + v[119] * v[9];
   v[121] = v[54] + v[56] + v[104] * v[104] + v[113] * v[113] + v[120] * v[120];
   v[122] = v[120] * v[29] + v[113] * v[51];
   v[123] = v[104] * v[63];
   v[124] = v[113] * v[83];
   v[125] = v[120] * v[94];
   v[126] = v[54] + v[56] + v[123] + v[124] + v[125];
   v[127] = v[98] * v[121] - v[122] * v[126];
   v[128] = v[0] * v[5];
   v[129] = v[3] * v[2] + v[128];
   v[130] = v[0] * v[2];
   v[131] = v[3] * v[5];
   v[132] = v[130] - v[131];
   v[133] = v[129] * v[14] + v[132] * v[9];
   v[134] = v[132] * v[22];
   v[135] = 0 - v[3];
   v[136] = v[135] * v[2] - v[128];
   v[137] = v[136] * v[22];
   v[138] = v[134] * v[14] + v[137] * v[9];
   v[139] = v[132] * v[7];
   v[140] = 0 - v[3];
   v[141] = v[140] * v[2] - v[128];
   v[142] = v[141] * v[7];
   v[143] = v[139] * v[14] + v[142] * v[9];
   v[144] = v[133] * v[133] + v[138] * v[138] + v[143] * v[143];
   v[145] = 0 - v[3];
   v[146] = v[145] * v[2] - v[128];
   v[147] = v[132] * v[14] + v[146] * v[9];
   v[148] = sin(x[12]);
   v[149] = v[147] * v[148];
   v[150] = 0 - v[3];
   v[151] = v[150] * v[2] - v[128];
   v[152] = v[151] * v[22];
   v[131] = v[131] - v[130];
   v[130] = v[131] * v[22];
   v[153] = v[152] * v[14] + v[130] * v[9];
   v[154] = cos(x[12]);
   v[155] = v[153] * v[148] + v[7] * v[154];
   v[156] = 0 - v[3];
   v[128] = v[156] * v[2] - v[128];
   v[157] = v[128] * v[7];
   v[158] = v[131] * v[7];
   v[159] = v[157] * v[14] + v[158] * v[9];
   v[160] = v[159] * v[148] - v[22] * v[154];
   v[161] = v[149] * v[149] + v[155] * v[155] + v[160] * v[160];
   v[162] = v[149] * v[133];
   v[163] = v[155] * v[138];
   v[164] = v[160] * v[143];
   v[165] = v[162] + v[163] + v[164];
   v[164] = v[162] + v[163] + v[164];
   v[163] = v[144] * v[161] - v[165] * v[164];
   v[162] = v[127] * v[163];
   v[166] = v[160] * v[55] + v[155] * v[7];
   v[167] = v[98] * v[166];
   v[168] = v[167] - v[149] * v[126];
   v[169] = v[143] * v[55] + v[138] * v[7];
   v[170] = v[144] * v[166];
   v[171] = v[169] * v[164] - v[170];
   v[172] = v[168] * v[171];
   v[173] = v[122] * v[169];
   v[174] = v[173] - v[133] * v[121];
   v[175] = v[169] * v[161];
   v[176] = v[175] - v[165] * v[166];
   v[177] = v[174] * v[176];
   v[178] = v[133] * v[166] - v[149] * v[169];
   v[179] = v[169] * v[166];
   v[180] = v[179] - v[179];
   v[181] = v[178] * v[180];
   v[182] = v[98] * v[169];
   v[183] = v[182] - v[133] * v[126];
   v[184] = v[183] * v[176];
   v[185] = v[122] * v[166];
   v[186] = v[185] - v[149] * v[121];
   v[187] = v[186] * v[171];
   v[188] = v[162] + v[172] + v[177] + v[181] - v[184] - v[187];
   v[189] = v[54] + v[56] + v[63] * v[63] + v[83] * v[83] + v[94] * v[94];
   v[125] = v[54] + v[56] + v[123] + v[124] + v[125];
   v[124] = v[189] * v[121] - v[125] * v[126];
   v[123] = v[124] * v[163];
   v[190] = v[189] * v[166];
   v[191] = v[166] * v[126];
   v[192] = v[190] - v[191];
   v[193] = v[192] * v[171];
   v[194] = v[125] * v[169];
   v[195] = v[169] * v[121];
   v[196] = v[194] - v[195];
   v[197] = v[196] * v[176];
   v[198] = v[180] * v[180];
   v[199] = v[189] * v[169];
   v[200] = v[169] * v[126];
   v[201] = v[199] - v[200];
   v[202] = v[201] * v[176];
   v[203] = v[125] * v[166];
   v[204] = v[166] * v[121];
   v[205] = v[203] - v[204];
   v[206] = v[205] * v[171];
   v[207] = v[123] + v[193] + v[197] + v[198] - v[202] - v[206];
   v[208] = v[104] * v[72];
   v[209] = v[113] * v[39];
   v[210] = v[120] * v[18];
   v[211] = v[54] + v[56] + v[208] + v[209] + v[210];
   v[212] = v[98] * v[125] - v[122] * v[189];
   v[213] = v[212] * v[163];
   v[214] = v[167] - v[149] * v[189];
   v[215] = v[214] * v[171];
   v[216] = v[173] - v[133] * v[125];
   v[217] = v[216] * v[176];
   v[218] = v[182] - v[133] * v[189];
   v[219] = v[218] * v[176];
   v[220] = v[185] - v[149] * v[125];
   v[221] = v[220] * v[171];
   v[222] = v[213] + v[215] + v[217] + v[181] - v[219] - v[221];
   v[223] = -1 * v[169];
   v[175] = v[175] - v[166] * v[164];
   v[224] = v[212] * v[175];
   v[225] = v[121] * v[164] - v[179];
   v[226] = v[214] * v[225];
   v[227] = v[166] * v[166];
   v[228] = v[126] * v[161] - v[227];
   v[229] = v[216] * v[228];
   v[230] = v[191] - v[204];
   v[231] = v[178] * v[230];
   v[232] = v[121] * v[161] - v[227];
   v[233] = v[218] * v[232];
   v[234] = v[126] * v[164] - v[179];
   v[235] = v[220] * v[234];
   v[236] = v[224] + v[226] + v[229] + v[231] - v[233] - v[235];
   v[170] = v[169] * v[165] - v[170];
   v[237] = v[212] * v[170];
   v[238] = v[169] * v[169];
   v[239] = v[121] * v[144] - v[238];
   v[240] = v[214] * v[239];
   v[241] = v[126] * v[165] - v[179];
   v[242] = v[216] * v[241];
   v[243] = v[200] - v[195];
   v[244] = v[178] * v[243];
   v[245] = v[121] * v[165] - v[179];
   v[246] = v[218] * v[245];
   v[247] = v[126] * v[144] - v[238];
   v[248] = v[220] * v[247];
   v[249] = v[237] + v[240] + v[242] + v[244] - v[246] - v[248];
   v[250] = v[97] * v[188] + v[52] * v[207] + v[211] * v[222] + v[223] * v[236] + v[166] * v[249];
   v[251] = 1 + v[51] * v[51] + v[29] * v[29];
   v[252] = -1 * v[96];
   v[95] = v[54] + v[56] + v[73] + v[84] + v[95];
   v[210] = v[54] + v[56] + v[208] + v[209] + v[210];
   v[209] = v[95] * v[121] - v[210] * v[126];
   v[208] = v[209] * v[163];
   v[84] = v[95] * v[166];
   v[191] = v[84] - v[191];
   v[73] = v[191] * v[171];
   v[253] = v[210] * v[169];
   v[195] = v[253] - v[195];
   v[254] = v[195] * v[176];
   v[255] = v[95] * v[169];
   v[200] = v[255] - v[200];
   v[256] = v[200] * v[176];
   v[257] = v[210] * v[166];
   v[204] = v[257] - v[204];
   v[258] = v[204] * v[171];
   v[259] = v[208] + v[73] + v[254] + v[198] - v[256] - v[258];
   v[56] = v[54] + v[56] + v[72] * v[72] + v[39] * v[39] + v[18] * v[18];
   v[206] = v[123] + v[193] + v[197] + v[198] - v[202] - v[206];
   v[202] = v[95] * v[125] - v[210] * v[189];
   v[197] = v[202] * v[163];
   v[84] = v[84] - v[190];
   v[193] = v[84] * v[171];
   v[253] = v[253] - v[194];
   v[123] = v[253] * v[176];
   v[255] = v[255] - v[199];
   v[54] = v[255] * v[176];
   v[257] = v[257] - v[203];
   v[260] = v[257] * v[171];
   v[261] = v[197] + v[193] + v[123] + v[198] - v[54] - v[260];
   v[262] = -1 * v[169];
   v[263] = v[202] * v[175];
   v[264] = v[84] * v[225];
   v[265] = v[253] * v[228];
   v[266] = v[180] * v[230];
   v[267] = v[255] * v[232];
   v[268] = v[257] * v[234];
   v[269] = v[263] + v[264] + v[265] + v[266] - v[267] - v[268];
   v[270] = v[202] * v[170];
   v[271] = v[84] * v[239];
   v[272] = v[253] * v[241];
   v[273] = v[180] * v[243];
   v[274] = v[255] * v[245];
   v[275] = v[257] * v[247];
   v[276] = v[270] + v[271] + v[272] + v[273] - v[274] - v[275];
   v[277] = v[252] * v[259] + v[56] * v[206] + v[211] * v[261] + v[262] * v[269] + v[166] * v[276];
   v[278] = -1 * v[56];
   v[187] = v[162] + v[172] + v[177] + v[181] - v[184] - v[187];
   v[258] = v[208] + v[73] + v[254] + v[198] - v[256] - v[258];
   v[256] = v[98] * v[210] - v[122] * v[95];
   v[254] = v[256] * v[163];
   v[167] = v[167] - v[149] * v[95];
   v[73] = v[167] * v[171];
   v[173] = v[173] - v[133] * v[210];
   v[208] = v[173] * v[176];
   v[182] = v[182] - v[133] * v[95];
   v[184] = v[182] * v[176];
   v[185] = v[185] - v[149] * v[210];
   v[177] = v[185] * v[171];
   v[172] = v[254] + v[73] + v[208] + v[181] - v[184] - v[177];
   v[162] = -1 * v[169];
   v[279] = v[256] * v[175];
   v[280] = v[167] * v[225];
   v[281] = v[173] * v[228];
   v[282] = v[182] * v[232];
   v[283] = v[185] * v[234];
   v[284] = v[279] + v[280] + v[281] + v[231] - v[282] - v[283];
   v[285] = v[256] * v[170];
   v[286] = v[167] * v[239];
   v[287] = v[173] * v[241];
   v[288] = v[182] * v[245];
   v[289] = v[185] * v[247];
   v[290] = v[285] + v[286] + v[287] + v[244] - v[288] - v[289];
   v[291] = v[278] * v[187] + v[52] * v[258] + v[211] * v[172] + v[162] * v[284] + v[166] * v[290];
   v[292] = -1 * v[122];
   v[293] = -1 * v[56];
   v[221] = v[213] + v[215] + v[217] + v[181] - v[219] - v[221];
   v[260] = v[197] + v[193] + v[123] + v[198] - v[54] - v[260];
   v[177] = v[254] + v[73] + v[208] + v[181] - v[184] - v[177];
   v[184] = -1 * v[169];
   v[208] = v[125] * v[164] - v[179];
   v[73] = v[167] * v[208];
   v[254] = v[189] * v[161] - v[227];
   v[181] = v[173] * v[254];
   v[203] = v[190] - v[203];
   v[190] = v[178] * v[203];
   v[227] = v[125] * v[161] - v[227];
   v[54] = v[182] * v[227];
   v[123] = v[189] * v[164] - v[179];
   v[193] = v[185] * v[123];
   v[197] = v[279] + v[73] + v[181] + v[190] - v[54] - v[193];
   v[198] = v[125] * v[144] - v[238];
   v[219] = v[167] * v[198];
   v[217] = v[189] * v[165] - v[179];
   v[215] = v[173] * v[217];
   v[199] = v[199] - v[194];
   v[194] = v[178] * v[199];
   v[179] = v[125] * v[165] - v[179];
   v[213] = v[182] * v[179];
   v[238] = v[189] * v[144] - v[238];
   v[294] = v[185] * v[238];
   v[295] = v[285] + v[219] + v[215] + v[194] - v[213] - v[294];
   v[296] = v[293] * v[221] + v[52] * v[260] + v[96] * v[177] + v[184] * v[197] + v[166] * v[295];
   v[297] = -1 * v[56];
   v[235] = v[224] + v[226] + v[229] + v[231] - v[233] - v[235];
   v[268] = v[263] + v[264] + v[265] + v[266] - v[267] - v[268];
   v[283] = v[279] + v[280] + v[281] + v[231] - v[282] - v[283];
   v[282] = -1 * v[211];
   v[193] = v[279] + v[73] + v[181] + v[190] - v[54] - v[193];
   v[54] = v[256] * v[180];
   v[190] = v[167] * v[196];
   v[181] = v[173] * v[192];
   v[73] = v[178] * v[124];
   v[279] = v[182] * v[205];
   v[281] = v[185] * v[201];
   v[280] = v[54] + v[190] + v[181] + v[73] - v[279] - v[281];
   v[231] = v[297] * v[235] + v[52] * v[268] + v[96] * v[283] + v[282] * v[193] + v[166] * v[280];
   v[267] = -1 * v[149];
   v[266] = -1 * v[56];
   v[248] = v[237] + v[240] + v[242] + v[244] - v[246] - v[248];
   v[275] = v[270] + v[271] + v[272] + v[273] - v[274] - v[275];
   v[289] = v[285] + v[286] + v[287] + v[244] - v[288] - v[289];
   v[288] = -1 * v[211];
   v[294] = v[285] + v[219] + v[215] + v[194] - v[213] - v[294];
   v[281] = v[54] + v[190] + v[181] + v[73] - v[279] - v[281];
   v[279] = v[266] * v[248] + v[52] * v[275] + v[96] * v[289] + v[288] * v[294] + v[169] * v[281];
   v[73] = (-1 * 1 / sqrt(v[53] * v[250] + v[251] * v[277] + v[98] * v[291] + v[292] * v[296] + v[133] * v[231] + v[267] * v[279])) / 2.;
   v[181] = v[73] * v[133];
   v[292] = v[73] * v[292];
   v[190] = v[73] * v[98];
   v[53] = v[73] * v[53];
   v[267] = v[73] * v[267];
   v[54] = v[267] * v[169];
   v[213] = v[181] * v[166];
   v[194] = v[54] + v[213];
   v[288] = v[267] * v[288];
   v[215] = v[292] * v[166];
   v[219] = v[288] + v[215];
   v[282] = v[181] * v[282];
   v[184] = v[292] * v[184];
   v[285] = v[282] + v[184];
   v[287] = v[267] * v[96];
   v[286] = v[190] * v[166];
   v[244] = v[287] + v[286];
   v[274] = v[181] * v[96];
   v[162] = v[190] * v[162];
   v[273] = v[274] + v[162];
   v[96] = v[292] * v[96];
   v[272] = v[190] * v[211];
   v[271] = v[96] + v[272];
   v[198] = v[194] * v[196] + v[219] * v[198] + v[285] * v[208] + v[244] * v[239] + v[273] * v[225] + v[271] * v[171];
   v[266] = v[267] * v[266];
   v[208] = v[53] * v[166];
   v[270] = v[266] + v[208];
   v[297] = v[181] * v[297];
   v[223] = v[53] * v[223];
   v[246] = v[297] + v[223];
   v[293] = v[292] * v[293];
   v[242] = v[53] * v[211];
   v[240] = v[293] + v[242];
   v[237] = v[270] * v[239] + v[246] * v[225] + v[240] * v[171];
   v[278] = v[190] * v[278];
   v[97] = v[53] * v[97];
   v[265] = v[278] + v[97];
   v[264] = v[265] * v[171];
   v[263] = v[198] + v[237] + v[264];
   v[233] = 0 - v[54] - v[213];
   v[229] = 0 - v[288] - v[215];
   v[226] = 0 - v[282] - v[184];
   v[224] = 0 - v[287] - v[286];
   v[298] = 0 - v[274] - v[162];
   v[299] = 0 - v[96] - v[272];
   v[238] = v[233] * v[201] + v[229] * v[238] + v[226] * v[123] + v[224] * v[247] + v[298] * v[234] + v[299] * v[171];
   v[123] = 0 - v[266] - v[208];
   v[300] = 0 - v[297] - v[223];
   v[301] = 0 - v[293] - v[242];
   v[302] = v[123] * v[247] + v[300] * v[234] + v[301] * v[171];
   v[303] = 0 - v[278] - v[97];
   v[304] = v[303] * v[171];
   v[305] = v[238] + v[302] + v[304];
   v[306] = v[54] + v[213];
   v[307] = v[288] + v[215];
   v[308] = v[282] + v[184];
   v[309] = v[287] + v[266] + v[286] + v[208];
   v[310] = v[274] + v[297] + v[162] + v[223];
   v[311] = v[96] + v[293] + v[272] + v[278] + v[242] + v[97];
   v[199] = v[306] * v[124] + v[307] * v[199] + v[308] * v[203] + v[309] * v[243] + v[310] * v[230] + v[311] * v[180];
   v[251] = v[73] * v[251];
   v[203] = v[282] + v[274] + v[184] + v[162];
   v[312] = v[297] + v[223];
   v[313] = v[181] * v[52];
   v[262] = v[251] * v[262];
   v[314] = v[313] + v[262];
   v[315] = v[203] * v[256] + v[312] * v[212] + v[314] * v[202];
   v[316] = 0 - v[315];
   v[317] = 0 - v[282] - v[184];
   v[318] = v[317] * v[182];
   v[184] = v[282] + v[184];
   v[282] = v[184] * v[173];
   v[319] = 0 - v[274] - v[162];
   v[320] = 0 - v[297] - v[223];
   v[321] = 0 - v[313] - v[262];
   v[322] = v[319] * v[182] + v[320] * v[218] + v[321] * v[255];
   v[162] = v[274] + v[162];
   v[223] = v[297] + v[223];
   v[297] = v[313] + v[262];
   v[274] = v[162] * v[173] + v[223] * v[216] + v[297] * v[253];
   v[323] = 0 - v[318] - v[282] - v[322] - v[274];
   v[324] = v[267] * v[52];
   v[325] = v[251] * v[166];
   v[326] = v[324] + v[325];
   v[327] = v[313] + v[262];
   v[328] = v[292] * v[52];
   v[211] = v[251] * v[211];
   v[329] = v[328] + v[211];
   v[239] = v[326] * v[239] + v[327] * v[225] + v[329] * v[171];
   v[225] = v[190] * v[52];
   v[252] = v[251] * v[252];
   v[330] = v[225] + v[252];
   v[331] = v[330] * v[171];
   v[332] = v[239] + v[331];
   v[333] = 0 - v[324] - v[325];
   v[334] = 0 - v[313] - v[262];
   v[335] = 0 - v[328] - v[211];
   v[247] = v[333] * v[247] + v[334] * v[234] + v[335] * v[171];
   v[234] = 0 - v[225] - v[252];
   v[336] = v[234] * v[171];
   v[337] = v[247] + v[336];
   v[308] = v[308] * v[178];
   v[338] = v[54] + v[213];
   v[52] = v[53] * v[52];
   v[56] = v[251] * v[56];
   v[339] = v[52] + v[56];
   v[340] = v[338] * v[173] + v[339] * v[171];
   v[239] = v[308] - v[239] + v[340];
   v[262] = v[313] + v[262];
   v[310] = v[310] * v[178] + v[262] * v[180];
   v[340] = v[310] - v[331] - v[340];
   v[331] = 0 - v[96] - v[272];
   v[313] = v[96] + v[272];
   v[341] = 0 - v[293] - v[242];
   v[342] = v[293] + v[242];
   v[343] = 0 - v[278] - v[97];
   v[344] = v[278] + v[97];
   v[345] = 0 - v[328] - v[211];
   v[346] = v[328] + v[211];
   v[347] = 0 - v[225] - v[252];
   v[348] = v[225] + v[252];
   v[349] = 0 - v[52] - v[56];
   v[350] = v[52] + v[56];
   v[200] = v[331] * v[182] + v[313] * v[173] + v[341] * v[218] + v[342] * v[216] + v[343] * v[183] + v[344] * v[174] + v[345] * v[255] + v[346] * v[253] + v[347] * v[200] + v[348] * v[195] + v[349] * v[201] + v[350] * v[196];
   v[195] = 0 - v[200];
   v[201] = 0 - v[54] - v[213];
   v[196] = 0 - v[52] - v[56];
   v[171] = v[201] * v[182] + v[196] * v[171];
   v[308] = 0 - v[308] - v[247] + v[171];
   v[171] = 0 - v[310] - v[336] - v[171];
   v[310] = v[288] + v[287] + v[215] + v[286];
   v[336] = v[266] + v[208];
   v[247] = v[324] + v[325];
   v[183] = v[310] * v[256] + v[336] * v[212] + v[247] * v[202];
   v[196] = v[271] * v[167] + v[299] * v[185] + v[240] * v[214] + v[301] * v[220] + v[265] * v[168] + v[303] * v[186] + v[329] * v[84] + v[335] * v[257] + v[330] * v[191] + v[234] * v[204] + v[339] * v[192] + v[196] * v[205];
   v[339] = 0 - v[183] - v[196];
   v[234] = 0 - v[288] - v[215];
   v[335] = v[234] * v[182];
   v[215] = v[288] + v[215];
   v[288] = v[215] * v[173];
   v[285] = v[285] * v[167];
   v[226] = v[226] * v[185];
   v[330] = 0 - v[287] - v[286];
   v[329] = 0 - v[266] - v[208];
   v[303] = 0 - v[324] - v[325];
   v[182] = v[330] * v[182] + v[329] * v[218] + v[303] * v[255];
   v[286] = v[287] + v[286];
   v[208] = v[266] + v[208];
   v[266] = v[324] + v[325];
   v[173] = v[286] * v[173] + v[208] * v[216] + v[266] * v[253];
   v[327] = v[273] * v[167] + v[246] * v[214] + v[327] * v[84];
   v[334] = v[298] * v[185] + v[300] * v[220] + v[334] * v[257];
   v[213] = v[54] + v[213];
   v[325] = v[324] + v[325];
   v[324] = v[328] + v[225] + v[52] + v[211] + v[252] + v[56];
   v[324] = v[213] * v[256] + v[311] * v[178] + v[325] * v[243] + v[262] * v[230] + v[324] * v[180] + v[324] * v[180];
   v[324] = 0 - v[335] - v[288] - v[285] - v[226] - v[182] - v[173] - v[327] - v[334] + v[324] - v[324];
   v[323] = v[181] * v[280] + v[292] * v[295] + v[190] * v[290] + v[53] * v[249] + v[263] * v[98] + v[305] * v[122] + v[199] * v[133] + v[251] * v[276] + v[316] * v[164] + v[323] * v[166] + v[323] * v[166] + v[332] * v[95] + v[337] * v[210] + v[239] * v[189] + v[340] * v[126] + v[195] * v[165] + v[308] * v[125] + v[171] * v[121] + v[339] * v[144] + v[324] * v[169];
   v[330] = v[201] * v[205] + v[234] * v[179] + v[317] * v[227] + v[330] * v[245] + v[319] * v[232] + v[331] * v[176];
   v[329] = v[329] * v[245] + v[320] * v[232] + v[341] * v[176];
   v[343] = v[343] * v[176];
   v[341] = v[330] + v[329] + v[343];
   v[286] = v[338] * v[192] + v[215] * v[217] + v[184] * v[254] + v[286] * v[241] + v[162] * v[228] + v[313] * v[176];
   v[208] = v[208] * v[241] + v[223] * v[228] + v[342] * v[176];
   v[344] = v[344] * v[176];
   v[342] = v[286] + v[208] + v[344];
   v[223] = 0 - v[199];
   v[219] = v[219] * v[167];
   v[229] = v[229] * v[185];
   v[326] = v[244] * v[167] + v[270] * v[214] + v[326] * v[84];
   v[333] = v[224] * v[185] + v[123] * v[220] + v[333] * v[257];
   v[123] = 0 - v[219] - v[229] - v[326] - v[333];
   v[303] = v[303] * v[245] + v[321] * v[232] + v[345] * v[176];
   v[347] = v[347] * v[176];
   v[345] = v[303] + v[347];
   v[266] = v[266] * v[241] + v[297] * v[228] + v[346] * v[176];
   v[348] = v[348] * v[176];
   v[346] = v[266] + v[348];
   v[307] = v[307] * v[178];
   v[349] = v[233] * v[185] + v[349] * v[176];
   v[303] = v[307] - v[303] + v[349];
   v[325] = v[309] * v[178] + v[325] * v[180];
   v[349] = v[325] - v[347] - v[349];
   v[350] = v[194] * v[167] + v[350] * v[176];
   v[307] = 0 - v[307] - v[266] + v[350];
   v[350] = 0 - v[325] - v[348] - v[350];
   v[200] = v[315] + v[200];
   v[123] = v[267] * v[281] + v[292] * v[197] * -1 + v[190] * v[284] * -1 + v[53] * v[236] * -1 + v[341] * v[98] + v[342] * v[122] + v[223] * v[149] + v[183] * v[165] + v[123] * v[169] + v[123] * v[169] + v[251] * v[269] * -1 + v[345] * v[95] + v[346] * v[210] + v[303] * v[189] + v[349] * v[126] + v[307] * v[125] + v[350] * v[121] + v[200] * v[161] + v[196] * v[164] + v[324] * v[166];
   v[97] = v[278] + v[97];
   v[278] = v[97] * v[163];
   v[344] = 0 - v[344];
   v[304] = 0 - v[304];
   v[252] = v[225] + v[252];
   v[225] = v[252] * v[163];
   v[56] = v[52] + v[56];
   v[306] = v[306] * v[178] + v[56] * v[163];
   v[350] = v[278] * v[98] + v[344] * v[133] + v[304] * v[149] + v[182] * v[165] + v[326] * v[144] + v[322] * v[161] + v[327] * v[164] + v[225] * v[95] + v[306] * v[189] + v[350] * v[169] + v[171] * v[166];
   v[171] = 0 - v[278];
   v[343] = 0 - v[343];
   v[264] = 0 - v[264];
   v[178] = 0 - v[225];
   v[52] = 0 - v[306];
   v[349] = v[171] * v[122] + v[343] * v[133] + v[264] * v[149] + v[173] * v[165] + v[333] * v[144] + v[274] * v[161] + v[334] * v[164] + v[178] * v[210] + v[52] * v[125] + v[349] * v[169] + v[340] * v[166];
   v[294] = v[267] * v[294] * -1 + v[181] * v[193] * -1 + v[190] * v[172] + v[53] * v[222] + v[251] * v[261];
   v[242] = v[293] + v[242];
   v[336] = v[336] * v[170] + v[312] * v[175] + v[242] * v[163];
   v[208] = 0 - v[208];
   v[302] = 0 - v[302];
   v[211] = v[328] + v[211];
   v[247] = v[247] * v[170] + v[314] * v[175] + v[211] * v[163];
   v[52] = v[335] * v[165] + v[219] * v[144] + v[318] * v[161] + v[285] * v[164] + v[336] * v[98] + v[208] * v[133] + v[302] * v[149] + v[247] * v[95] + v[52] * v[126] + v[307] * v[169] + v[308] * v[166];
   v[307] = 0 - v[336];
   v[329] = 0 - v[329];
   v[237] = 0 - v[237];
   v[308] = 0 - v[247];
   v[306] = v[288] * v[165] + v[229] * v[144] + v[282] * v[161] + v[226] * v[164] + v[307] * v[122] + v[329] * v[133] + v[237] * v[149] + v[308] * v[210] + v[306] * v[121] + v[303] * v[169] + v[239] * v[166];
   v[289] = v[267] * v[289] + v[181] * v[283] + v[292] * v[177] + v[53] * v[188] * -1 + v[251] * v[259] * -1;
   v[272] = v[96] + v[272];
   v[213] = v[213] * v[180] + v[310] * v[170] + v[203] * v[175] + v[272] * v[163];
   v[286] = 0 - v[286];
   v[238] = 0 - v[238];
   v[308] = v[213] * v[98] + v[286] * v[133] + v[238] * v[149] + v[308] * v[189] + v[178] * v[126] + v[346] * v[169] + v[337] * v[166];
   v[178] = 0 - v[213];
   v[330] = 0 - v[330];
   v[198] = 0 - v[198];
   v[247] = v[178] * v[122] + v[330] * v[133] + v[198] * v[149] + v[247] * v[125] + v[225] * v[121] + v[345] * v[169] + v[332] * v[166];
   v[251] = v[267] * v[248] * -1 + v[181] * v[235] * -1 + v[292] * v[221] * -1 + v[190] * v[187] * -1 + v[251] * v[206];
   v[248] = v[350] + v[349] + v[294] + v[52] + v[306] + v[289] + v[308] + v[247] + v[251];
   v[178] = v[73] * v[296] * -1 + v[178] * v[95] + v[307] * v[189] + v[171] * v[126] + v[342] * v[169] + v[305] * v[166];
   v[213] = v[73] * v[291] + v[213] * v[210] + v[336] * v[125] + v[278] * v[121] + v[341] * v[169] + v[263] * v[166];
   v[267] = v[267] * v[275] + v[181] * v[268] + v[292] * v[260] + v[190] * v[258] + v[73] * v[250] * -1 + v[53] * v[207];
   v[277] = v[73] * v[277];
   v[53] = v[178] * v[113] + v[213] * v[83] + v[267] * v[39] + v[277] * v[51] + v[277] * v[51];
   v[190] = v[53] * v[14];
   v[292] = v[53] * v[9];
   v[181] = v[349] + v[52];
   v[275] = v[294] + v[308];
   v[268] = v[350] * v[120] + v[350] * v[120] + v[181] * v[94] + v[275] * v[18] + v[178] * v[29];
   v[260] = v[268] * v[14];
   v[258] = v[268] * v[9];
   v[250] = v[289] + v[247];
   v[181] = v[181] * v[120] + v[306] * v[94] + v[306] * v[94] + v[250] * v[18] + v[213] * v[29];
   v[207] = v[181] * v[14];
   v[336] = v[181] * v[9];
   v[250] = v[275] * v[120] + v[250] * v[94] + v[251] * v[18] + v[251] * v[18] + v[267] * v[29];
   v[275] = v[250] * v[14];
   v[278] = v[250] * v[9];
   v[277] = v[178] * v[120] + v[213] * v[94] + v[267] * v[18] + v[277] * v[29] + v[277] * v[29];
   v[272] = v[272] * v[256] + v[242] * v[212] + v[97] * v[127] + v[211] * v[202] + v[252] * v[209] + v[56] * v[124];
   v[200] = v[318] * v[125] + v[282] * v[189] + v[322] * v[121] + v[274] * v[126] + v[272] * v[144] + v[200] * v[169];
   v[274] = 0 - v[272];
   v[334] = v[285] * v[125] + v[226] * v[189] + v[316] * v[166] + v[327] * v[121] + v[334] * v[126] + v[274] * v[165] + v[196] * v[169];
   v[274] = v[335] * v[125] + v[288] * v[189] + v[183] * v[169] + v[182] * v[121] + v[173] * v[126] + v[274] * v[164] + v[195] * v[166];
   v[173] = v[334] + v[274];
   v[182] = v[200] * v[155] + v[200] * v[155] + v[173] * v[138] + v[323] * v[7];
   v[288] = v[334] + v[274];
   v[335] = v[200] * v[160] + v[200] * v[160] + v[288] * v[143] + v[323] * v[55];
   v[183] = v[335] * v[148];
   v[195] = v[183] * v[14];
   v[164] = v[183] * v[9];
   v[272] = v[219] * v[125] + v[229] * v[189] + v[326] * v[121] + v[333] * v[126] + v[272] * v[161] + v[339] * v[166];
   v[288] = v[288] * v[160] + v[272] * v[143] + v[272] * v[143] + v[123] * v[55];
   v[333] = v[288] * v[14];
   v[326] = v[288] * v[9];
   v[229] = v[349] + v[52];
   v[219] = v[294] + v[308];
   v[178] = v[350] * v[113] + v[350] * v[113] + v[229] * v[83] + v[219] * v[39] + v[178] * v[51];
   v[339] = v[178] * v[14];
   v[161] = v[178] * v[9];
   v[327] = v[289] + v[247];
   v[229] = v[229] * v[113] + v[306] * v[83] + v[306] * v[83] + v[327] * v[39] + v[213] * v[51];
   v[213] = v[229] * v[14];
   v[226] = v[229] * v[9];
   v[327] = v[219] * v[113] + v[327] * v[83] + v[251] * v[39] + v[251] * v[39] + v[267] * v[51];
   v[219] = v[327] * v[14];
   v[267] = v[327] * v[9];
   v[113] = v[277] * v[14];
   v[83] = v[277] * v[9];
   v[51] = v[182] * v[148];
   v[39] = v[51] * v[14];
   v[285] = v[51] * v[9];
   v[173] = v[173] * v[155] + v[272] * v[138] + v[272] * v[138] + v[123] * v[7];
   v[196] = v[173] * v[14];
   v[316] = v[173] * v[9];
   v[165] = 0 - v[335];
   v[322] = v[350] + v[349] + v[294] + v[52] + v[306] + v[289] + v[308] + v[247] + v[251];
   jac[0] = 0 - (v[323] * v[155] + v[123] * v[138] + v[248] * v[7] + v[248] * v[7] + v[190] * v[46] + v[292] * v[42] + v[53] * v[50] + v[260] * v[114] + v[258] * v[118] + v[207] * v[89] + v[336] * v[87] + v[181] * v[93] + v[275] * v[12] + v[278] * v[6] + v[250] * v[17] + (0 - v[277]) * 0.1333 + v[182] * v[154] + v[195] * v[128] + v[164] * v[131] + v[333] * v[132] + v[326] * v[141]) * sin(x[8]) + (v[339] * v[107] + v[161] * v[111] + v[213] * v[78] + v[226] * v[76] + v[229] * v[82] + v[219] * v[34] + v[267] * v[32] + v[327] * v[38] + (0 - v[53]) * 0.1333 + v[113] * v[24] + v[83] * v[21] + v[277] * v[28] + v[39] * v[151] + v[285] * v[131] + v[196] * v[132] + v[316] * v[136] + v[165] * v[154] - (v[323] * v[160] + v[123] * v[143] + v[322] * v[55] + v[322] * v[55])) * cos(x[8]);
   v[52] = v[349] + v[52];
   v[308] = v[294] + v[308];
   v[350] = v[350] * v[104] + v[350] * v[104] + v[52] * v[63] + v[308] * v[72];
   v[294] = v[350] * v[14];
   v[349] = v[350] * v[9];
   v[247] = v[289] + v[247];
   v[52] = v[52] * v[104] + v[306] * v[63] + v[306] * v[63] + v[247] * v[72];
   v[306] = v[52] * v[14];
   v[289] = v[52] * v[9];
   v[322] = 0 - v[52];
   v[247] = v[308] * v[104] + v[247] * v[63] + v[251] * v[72] + v[251] * v[72];
   v[308] = v[247] * v[14];
   v[251] = v[247] * v[9];
   v[339] = v[339] * v[22];
   v[161] = v[161] * v[22];
   v[104] = 0 - v[161];
   v[213] = v[213] * v[22];
   v[226] = v[226] * v[22];
   v[72] = 0 - v[226];
   v[63] = v[229] * v[22];
   v[248] = 0 - v[63];
   v[219] = v[219] * v[22];
   v[267] = v[267] * v[22];
   v[123] = 0 - v[267];
   v[323] = v[327] * v[22];
   v[160] = 0 - v[323];
   v[190] = v[190] * v[7];
   v[128] = 0 - v[190];
   v[292] = v[292] * v[7];
   v[155] = 0 - v[292];
   v[154] = v[53] * v[7];
   v[260] = v[260] * v[7];
   v[258] = v[258] * v[7];
   v[131] = 0 - v[258];
   v[207] = v[207] * v[7];
   v[336] = v[336] * v[7];
   v[151] = 0 - v[336];
   v[143] = v[181] * v[7];
   v[141] = 0 - v[143];
   v[275] = v[275] * v[7];
   v[278] = v[278] * v[7];
   v[138] = 0 - v[278];
   v[136] = v[250] * v[7];
   v[118] = 0 - v[136];
   v[113] = v[113] * v[22];
   v[83] = v[83] * v[22];
   v[114] = v[277] * v[22];
   v[285] = v[285] * v[22] + v[164] * v[7];
   v[274] = v[334] + v[274];
   v[200] = v[73] * v[279] * -1 + v[198] * v[95] + v[238] * v[210] + v[237] * v[189] + v[302] * v[125] + v[264] * v[126] + v[304] * v[121] + v[223] * v[169] + v[200] * v[149] + v[200] * v[149] + v[274] * v[133];
   v[148] = v[200] * v[148];
   v[274] = v[73] * v[231] + v[330] * v[95] + v[286] * v[210] + v[329] * v[189] + v[208] * v[125] + v[343] * v[126] + v[344] * v[121] + v[199] * v[166] + v[274] * v[149] + v[272] * v[133] + v[272] * v[133];
   v[196] = v[148] * v[14] + v[274] * v[9] + v[196] * v[22] + v[333] * v[7];
   v[333] = v[285] - v[196];
   v[9] = v[148] * v[9];
   v[14] = v[274] * v[14];
   v[39] = v[39] * v[22];
   v[316] = v[316] * v[22];
   v[195] = v[195] * v[7];
   v[326] = v[326] * v[7];
   v[272] = 0 - v[9] + v[14] - v[39] - v[316] - v[195] - v[326];
   v[330] = 0 - v[294];
   v[286] = 0 - v[306];
   v[329] = 0 - v[308];
   v[208] = 0 - v[154];
   v[343] = 0 - v[113];
   v[196] = 0 - v[285] + v[196];
   jac[1] = 0 - (v[294] * v[99] + v[349] * v[100] + v[306] * v[57] + v[289] * v[58] + v[322] * v[62] + v[308] * v[67] + v[251] * v[65] + v[247] * v[71] + v[339] * v[106] + v[104] * v[110] + v[213] * v[74] + v[72] * v[75] + v[248] * v[81] + v[219] * v[30] + v[123] * v[31] + v[160] * v[37] + v[128] * v[45] + v[155] * v[41] + v[154] * v[48] + v[260] * v[44] + v[131] * v[117] + v[207] * v[85] + v[151] * v[86] + v[141] * v[92] + v[275] * v[11] + v[138] * v[4] + v[118] * v[16] + v[113] * v[19] + v[83] * v[20] + v[114] * v[27] + v[333] * v[3] + v[272] * v[0]) * sin(x[9]) + (v[330] * v[100] + v[349] * v[102] + v[286] * v[58] + v[289] * v[57] + v[52] * v[61] + v[329] * v[68] + v[251] * v[64] + v[247] * v[70] + v[339] * v[105] + v[161] * v[109] + v[213] * v[75] + v[226] * v[74] + v[63] * v[80] + v[219] * v[31] + v[267] * v[30] + v[323] * v[36] + v[190] * v[44] + v[292] * v[40] + v[208] * v[49] + v[260] * v[45] + v[258] * v[116] + v[207] * v[86] + v[336] * v[85] + v[143] * v[91] + v[275] * v[10] + v[278] * v[1] + v[136] * v[15] + v[343] * v[20] + v[83] * v[19] + v[114] * v[26] + v[9] * v[145] + v[14] * v[3] + v[39] * v[150] + v[316] * v[135] + v[195] * v[156] + v[196] * v[0] + v[326] * v[140]) * cos(x[9]);
   jac[2] = 0 - (v[294] * v[5] * 0.0997 + v[349] * v[2] * 0.0997 + (v[306] * v[5] + v[289] * v[2]) * 0.0997 + v[322] * v[5] * 0.392 + v[308] * v[5] * 0.0997 + v[251] * v[2] * 0.0997 + v[247] * v[5] * -0.392 + v[339] * v[2] * 0.0997 + v[104] * v[5] * 0.0997 + (v[213] * v[2] + v[72] * v[5]) * 0.0997 + v[63] * v[2] * -0.392 + (v[219] * v[2] + v[123] * v[5]) * 0.0997 + v[323] * v[2] * -0.392 + v[292] * v[2] * -0.0997 + v[154] * v[5] * 0.392 + (v[128] * v[5] + v[260] * v[2]) * 0.0997 + v[131] * v[5] * 0.0997 + (v[207] * v[2] + v[151] * v[5]) * 0.0997 + v[143] * v[2] * -0.392 + v[275] * v[2] * 0.0997 + v[138] * v[5] * 0.0997 + v[136] * v[2] * -0.392 + (v[113] * v[5] + v[83] * v[2]) * 0.0997 + v[114] * v[5] * -0.392 - v[9] * v[2] + v[14] * v[2] - v[39] * v[2] - v[316] * v[2] - v[195] * v[2] + v[333] * v[5] - v[326] * v[2]) * sin(x[10]) + ((v[330] * v[2] + v[349] * v[5]) * 0.0997 + (v[286] * v[2] + v[289] * v[5]) * 0.0997 + v[52] * v[2] * 0.392 + v[329] * v[2] * 0.0997 + v[251] * v[5] * 0.0997 + v[247] * v[2] * 0.392 + v[339] * v[5] * 0.0997 + v[161] * v[2] * 0.0997 + (v[213] * v[5] + v[226] * v[2]) * 0.0997 + v[248] * v[5] * 0.392 + (v[219] * v[5] + v[267] * v[2]) * 0.0997 + v[160] * v[5] * 0.392 + v[155] * v[5] * 0.0997 + v[208] * v[2] * 0.392 + (v[190] * v[2] + v[260] * v[5]) * 0.0997 + v[258] * v[2] * 0.0997 + (v[207] * v[5] + v[336] * v[2]) * 0.0997 + v[141] * v[5] * 0.392 + v[275] * v[5] * 0.0997 + v[278] * v[2] * 0.0997 + v[118] * v[5] * 0.392 + (v[343] * v[2] + v[83] * v[5]) * 0.0997 + v[114] * v[2] * 0.392 + v[196] * v[2] + v[272] * v[5]) * cos(x[10]);
   jac[3] = 0 - (v[350] * v[103] + v[52] * v[59] + v[247] * v[66] + v[178] * v[112] + v[229] * v[77] + v[327] * v[33] + v[53] * v[43] + v[268] * v[119] + v[181] * v[88] + v[250] * v[8] + v[277] * v[23] + v[148] * v[146] + v[274] * v[132] + v[51] * v[130] + v[173] * v[137] + v[183] * v[158] + v[288] * v[142]) * sin(x[11]) + (v[350] * v[101] + v[52] * v[60] + v[247] * v[69] + v[178] * v[108] + v[229] * v[79] + v[327] * v[35] + v[53] * v[47] + v[268] * v[115] + v[181] * v[90] + v[250] * v[13] + v[277] * v[25] + v[148] * v[132] + v[274] * v[129] + v[51] * v[152] + v[173] * v[134] + v[183] * v[157] + v[288] * v[139]) * cos(x[11]);
   jac[4] = 0 - (v[182] * v[7] + v[165] * v[22]) * sin(x[12]) + (v[200] * v[147] + v[182] * v[153] + v[335] * v[159]) * cos(x[12]);
}

