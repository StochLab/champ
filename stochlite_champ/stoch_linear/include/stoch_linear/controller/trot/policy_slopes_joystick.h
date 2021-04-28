const double policy[20][11] = {{-3.82422434e-02,-7.45086005e-02, -3.02945572e-03, -2.71146732e-02,
   1.82154575e-02, -1.43516326e-01,  6.98484046e-02,  2.28557128e-02,
  -1.18082414e-01, -4.95004686e-02,  4.62607443e-02},
      {-3.67110676e-03,  5.47303350e-02, -5.17510119e-02, -6.90279431e-02,
  -7.37441510e-02, -5.98624827e-03, -4.30887557e-02, -1.15215576e-01,
  -5.98709917e-02,  7.41881790e-02,  1.78610482e-02},
      {-1.51470284e-02, -4.95838546e-02,  5.33552777e-02,  1.19261297e-02,
   5.80728729e-02, -6.57358514e-02, -1.32372405e-02,  6.94897934e-02,
   4.07103650e-02,  9.01790678e-02, -4.13176629e-03},
      {6.44439401e-02,  6.91375312e-02,  2.65918214e-02,  1.84783623e-02,
   1.60324341e-03, -1.59065628e-03,  4.80843861e-03, -7.86988914e-02,
  -2.74432486e-02, -1.84915431e-02,  3.87583551e-02},
      { 8.64397648e-03, -1.11844664e-01, -8.13133596e-02,  7.99377619e-02,
  -3.39769931e-02, -5.01266861e-02,  3.57257732e-02, -1.59497714e-01,
  -9.29708292e-02, -2.63264946e-02,  6.71602762e-02},
      { 8.28271801e-03, -1.14489522e-01, -8.27217583e-02, -4.09161428e-02,
  -1.10370822e-01, -1.29815851e-01, -1.95943809e-02,  9.99861269e-02,
  -2.76299071e-02, -3.32973462e-02, -5.93553840e-02},
 { 7.24341521e-02,  1.88871169e-04,  7.41699859e-03,  9.97971369e-02,
   3.64596078e-02,  1.12078690e-01, -3.88089782e-02, -3.14382041e-03,
   2.36611121e-02, -7.23145091e-02, -9.00816912e-03},
   { 7.55070382e-02, -8.90678612e-02,  7.56929072e-02,  1.13841025e-01,
   6.60992321e-02,  1.29181866e-01,  3.51827980e-02, -3.10700802e-02,
   6.34197786e-02, -1.03184210e-01,  7.22893378e-02},
 {-2.17453917e-01, -1.10818884e-01, -2.54305719e-01, -3.20936850e-01,
  -1.28725381e-01, -3.25379032e-01, -1.97964095e-01, -3.21530038e-01,
  -3.63715848e-01,  1.09546623e-01, -3.67014870e-02},
 { 3.28257637e-01, -1.01921299e-01,  7.45806818e-02,  3.47479044e-01,
  -9.05281846e-02,  1.54549528e-01,  2.17226899e-01, -1.22482240e-01,
   3.01771438e-01, -1.15890644e-01,  1.47142767e-02},
 { 6.72745873e-02, -1.88960238e-01, -4.07284577e-02,  2.90416513e-01,
  -4.11030857e-02, -2.56216921e-01,  5.94382193e-01, -2.49377144e-02,
  -3.08560926e-01, -2.44886315e-01, -2.74858570e-01},
 {-1.13049598e-01,  2.55852289e-02, -1.55378684e-01, -2.83584703e-01,
  -2.50062200e-01, -3.80763796e-02, -3.64760870e-01, -3.42974795e-01,
   1.16995421e-01,  3.11219836e-02, -2.76465081e-01},
 {-2.50808203e-01, -1.45437458e-01,  7.27517477e-02, -1.90753282e-01,
  -1.33523135e-01, -6.32100268e-02, -1.08259677e-01, -5.20156844e-02,
  -1.18433767e-01, -1.43319741e-02,  1.11900159e-02},
 { 1.67789430e-01, -2.27135372e-01, -2.98427868e-02,  1.53664567e-02,
  -9.10743526e-02,  3.35030782e-02,  1.54584009e-01, -1.34421676e-01,
  -9.23988738e-02, -4.62149704e-02,  3.50538801e-02},
 {-8.56573876e-02,  1.87360216e-02, -6.06733345e-03, -1.35997918e-01,
   2.88332323e-02,  6.17386460e-02, -2.39934945e-01, -2.70206641e-02,
   8.01969224e-02,  7.33012151e-02,  3.34644492e-02},
 { 1.14003252e-01, -8.35028428e-02,  1.77242111e-01,  1.31609076e-01,
   5.77549479e-02,  7.61834169e-02,  1.39049652e-01,  8.65592975e-02,
  -1.94468896e-02,  9.53619571e-02, -1.49655317e-02},
  {-5.04074307e-03,  1.01991154e-02, -7.15015443e-02, -1.11342780e-01,
  -1.93585138e-01,  6.66178219e-02, -1.17217915e-01, -1.86149685e-01,
   5.04956610e-02,  8.94771137e-02,  1.59431740e-01},
   { 1.59106220e-01, -5.01548607e-03,  1.75610128e-01,  1.94651006e-01,
  -1.99270873e-01,  1.54151083e-01,  1.03613926e-01, -3.39634685e-02,
   1.58022166e-01, -8.03184619e-03,  8.74518069e-02},
   {-1.74090251e-01,  5.78576787e-02,  2.57729197e-02, -8.28993693e-02,
  -7.04718051e-02,  9.37197193e-03, -3.46522203e-02,  1.17433124e-01,
  -5.99757206e-02,  5.24896721e-04, -4.87508824e-02},
  { 1.45252326e-01, -3.21261529e-02, -2.03245642e-01,  9.25757970e-02,
  -7.11007857e-03, -1.50379282e-01,  5.54771998e-02, -5.75204733e-02,
  -1.77454439e-01,  6.79111057e-02, -1.82753733e-02}};
