// random snippets of useful code

// histogram in beginning of sonar callback
// int count0 = 0;
// int count1 = 0;
// int count2 = 0;
// int count3 = 0;
// for (int i = 0; i < floatImg.rows; i++) {
//   for (int j = 0; j < floatImg.cols; j++) {
//     int curX = j;
//     int curY = i;
//     float curVal = floatImg.at<float>(curY, curX);
//     if (0 > curVal && curVal < 0.25) {
//       count0++;
//     } else if (0.25 >= curVal && curVal < 0.5) {
//       count1++;
//     } else if (0.5 >= curVal && curVal < 0.75) {
//       count2++;
//     } else {
//       count3++;
//     }
//   }
// }
// cout << "basic distribution of points: \n";
// cout << "between 0 and 0.25:   " << count0 << "\n";
// cout << "between 0.25 and 0.5: " << count1 << "\n";
// cout << "between 0.5 and 0.75: " << count2 << "\n";
// cout << "between 0.75 and 1.0: " << count3 << "\n";


// draw float value in interpolation_img after interpolation call
// val = 1.0;
// for (int i = x-6; i < x+6; i++) {
//   for (int j = y-6; j < y+6; j++) {
//     interpolation_img.at<float>(j,i) = val;
//   }
// }

// output from run:
// min: -1.69474e+38
// max: 1.86489e+37
// bin width: 9.40614e+36
// avg of 1.53749e+32
// point interpolation finished
// current point: 666, 186
// min: 0
// max: 0.617068
// bin width: 0.0308534
// avg of 0.205792
// point interpolation finished
// current point: 740, 86
// min: 0
// max: 9.68838e+28
// bin width: 4.84419e+27
// avg of 0.248861
// point interpolation finished
// current point: 497, 85
// min: 0
// max: 0.450129
// bin width: 0.0225065
// avg of 5.23473e-13
// point interpolation finished
// current point: 646, 295
// min: 0
// max: 0.677492
// bin width: 0.0338746
// avg of 0
// point interpolation finished
// current point: 617, 231
// min: 0
// max: 0.689512
// bin width: 0.0344756
// avg of 0.35924
// point interpolation finished
// current point: 647, 281
// min: 0
// max: 0.565038
// bin width: 0.0282519
// avg of 0.400886
// point interpolation finished
// current point: 736, 217
// min: 0
// max: 0.743793
// bin width: 0.0371897
// avg of 0.526347
// point interpolation finished
// current point: 589, 277
// min: 0
// max: 0.523821
// bin width: 0.0261911
// avg of 0.439037
// point interpolation finished
// current point: 523, 64
// min: -6.86674e-08
// max: 2.74362e+38
// bin width: 1.37181e+37
// avg of nan
// point interpolation finished
// current point: 380, 158
// min: 0
// max: 0.604176
// bin width: 0.0302088
// avg of 0.137557
// point interpolation finished
// current point: 538, 103
// min: 0
// max: 0.605676
// bin width: 0.0302838
// avg of 0.259277
// point interpolation finished
// current point: 622, 195
// min: 0
// max: 0.667813
// bin width: 0.0333907
// avg of 0.316132
// point interpolation finished
// current point: 547, 247
// min: 0
// max: 0.521899
// bin width: 0.0260949
// avg of 0.39684
// point interpolation finished
// current point: 667, 240
// min: 0
// max: 0.518074
// bin width: 0.0259037
// avg of 0.276261
// point interpolation finished
// current point: 385, 152
// min: 0
// max: 0.604176
// bin width: 0.0302088
// avg of 0.139582
// point interpolation finished
// current point: 519, 293
// min: 0
// max: 0.5462
// bin width: 0.02731
// avg of 0
// point interpolation finished
// current point: 619, 73
// min: 0
// max: 0.613845
// bin width: 0.0306923
// avg of 1.37939e-25
// point interpolation finished
// current point: 574, 151
// min: 0
// max: 0.689859
// bin width: 0.0344929
// avg of 0.500365
// point interpolation finished
// current point: 720, 157
// min: 0
// max: 0.742439
// bin width: 0.037122
// avg of 0.472095
// point interpolation finished
// current point: 531, 70
// min: -1.40389e+09
// max: 8.05186e+37
// bin width: 4.02593e+36
// avg of nan
// point interpolation finished
// current point: 567, 150
// min: 0
// max: 0.610485
// bin width: 0.0305242
// avg of 0.501134
// point interpolation finished
// current point: 684, 111
// min: 0
// max: 0.569569
// bin width: 0.0284784
// avg of 0.274035
// point interpolation finished
// current point: 589, 177
// min: 0
// max: 0.699272
// bin width: 0.0349636
// avg of 0.550939
// point interpolation finished
// current point: 741, 123
// min: 0
// max: 0.784415
// bin width: 0.0392207
// avg of 0
// point interpolation finished
// current point: 456, 217
// min: 0
// max: 0.461165
// bin width: 0.0230583
// avg of 0.226706
// point interpolation finished
// current point: 556, 246
// min: 0
// max: 0.506711
// bin width: 0.0253356
// avg of 0.36361
// point interpolation finished
// current point: 489, 295
// min: 0
// max: 0.693754
// bin width: 0.0346877
// avg of 0
// point interpolation finished
// current point: 517, 177
// min: 0
// max: 0.58671
// bin width: 0.0293355
// avg of 0.582566
// point interpolation finished
// current point: 703, 129
// min: 0
// max: 0.484057
// bin width: 0.0242028
// avg of 0.25823
// point interpolation finished
// current point: 612, 108
// min: 0
// max: 0.610993
// bin width: 0.0305496
// avg of 0.436445
// point interpolation finished
// current point: 599, 232
// min: 0
// max: 0.656935
// bin width: 0.0328467
// avg of 0.441425
// point interpolation finished
// current point: 571, 156
// min: 0
// max: 0.603503
// bin width: 0.0301751
// avg of 0.498908
// point interpolation finished
// current point: 497, 219
// min: 0
// max: 0.659169
// bin width: 0.0329585
// avg of 0.364734
// point interpolation finished
// current point: 609, 277
// min: 0
// max: 0.49429
// bin width: 0.0247145
// avg of 0.406626
// point interpolation finished
// current point: 612, 235
// min: 0
// max: 0.669502
// bin width: 0.0334751
// avg of 0.491756
// point interpolation finished
// current point: 502, 159
// min: 0
// max: 0.693218
// bin width: 0.0346609
// avg of 0.252578
// point interpolation finished
// current point: 721, 98
// min: 0
// max: 0.875396
// bin width: 0.0437698
// avg of 0.50961
// point interpolation finished
// current point: 711, 99
// min: 0
// max: 0.561449
// bin width: 0.0280725
// avg of 0.31317
// point interpolation finished
// current point: 659, 48
// min: 0
// max: 0.761028
// bin width: 0.0380514
// avg of 2.50102e-17
// point interpolation finished
// current point: 675, 142
// min: 0
// max: 0.802466
// bin width: 0.0401233
// avg of 0.519061
// point interpolation finished
// current point: 601, 186
// min: 0
// max: 0.585346
// bin width: 0.0292673
// avg of 0.545521
// point interpolation finished
// current point: 416, 174
// min: 0
// max: 0.774926
// bin width: 0.0387463
// avg of 0.206187
// point interpolation finished
// current point: 628, 143
// min: 0
// max: 0.538418
// bin width: 0.0269209
// avg of 0.413245
// point interpolation finished
// current point: 585, 289
// min: 0
// max: 0.692315
// bin width: 0.0346158
// avg of 0.469603
// point interpolation finished
// current point: 535, 201
// min: 0
// max: 0.573966
// bin width: 0.0286983
// avg of 0.4934
// point interpolation finished
// current point: 569, 216
// min: 0
// max: 0.577735
// bin width: 0.0288867
// avg of 0.476384
// point interpolation finished
// current point: 572, 241
// min: 0
// max: 0.50713
// bin width: 0.0253565
// avg of 0.361735
// point interpolation finished
// current point: 589, 107
// min: 0
// max: 0.567075
// bin width: 0.0283537
// avg of 0.318582
// point interpolation finished
// current point: 451, 137
// min: 0
// max: 0.490668
// bin width: 0.0245334
// avg of 0.172358
// point interpolation finished
// current point: 659, 41
// min: -4.91724e-26
// max: 9.8978e+32
// bin width: 4.9489e+31
// avg of 2.2599e+25
// point interpolation finished
// current point: 615, 103
// min: 0
// max: 0.62902
// bin width: 0.031451
// avg of 0.434922
// point interpolation finished
// current point: 701, 95
// min: 0
// max: 0.571785
// bin width: 0.0285892
// avg of 0.408005
// point interpolation finished
// current point: 578, 156
// min: 0
// max: 0.609374
// bin width: 0.0304687
// avg of 0.499485
// point interpolation finished
// current point: 719, 272
// min: 0
// max: 0.602353
// bin width: 0.0301177
// avg of 0.339037
// point interpolation finished
// current point: 637, 47
// min: 0
// max: 0.816949
// bin width: 0.0408475
// avg of 3.59032e-22
// point interpolation finished
// current point: 499, 202
// min: 0
// max: 0.72447
// bin width: 0.0362235
// avg of 0.504873
// point interpolation finished
// current point: 562, 179
// min: 0
// max: 0.638587
// bin width: 0.0319293
// avg of 0.497318
// point interpolation finished
// current point: 524, 190
// min: 0
// max: 0.58818
// bin width: 0.029409
// avg of 0.578566
// point interpolation finished
// current point: 562, 271
// min: 0
// max: 0.580613
// bin width: 0.0290307
// avg of 0.42543
// point interpolation finished
// current point: 672, 188
// min: 0
// max: 0.513678
// bin width: 0.0256839
// avg of 0.213043
// point interpolation finished
// current point: 587, 286
// min: 0
// max: 0.657926
// bin width: 0.0328963
// avg of 0.441422
// point interpolation finished
// current point: 543, 97
// min: 0
// max: 0.572995
// bin width: 0.0286497
// avg of 0.395387
// point interpolation finished
// current point: 547, 292
// min: 0
// max: 0.666039
// bin width: 0.033302
// avg of 0.525757
// point interpolation finished
// current point: 628, 241
// min: 0
// max: 0.5354
// bin width: 0.02677
// avg of 0.361988
// point interpolation finished
// current point: 730, 60
// min: -8.76249e-08
// max: 7.61986e+37
// bin width: 3.80993e+36
// avg of 2.12233e+33
// point interpolation finished
// current point: 635, 289
// min: 0
// max: 0.575713
// bin width: 0.0287856
// avg of 0.375969
// point interpolation finished
// current point: 723, 220
// min: 0
// max: 0.601747
// bin width: 0.0300874
// avg of 0.425776
// point interpolation finished
// current point: 580, 283
// min: 0
// max: 0.60692
// bin width: 0.030346
// avg of 0.469595
// point interpolation finished
// current point: 685, 114
// min: 0
// max: 0.529065
// bin width: 0.0264532
// avg of 0.232993
// point interpolation finished
// current point: 561, 181
// min: 0
// max: 0.646956
// bin width: 0.0323478
// avg of 0.499228
// point interpolation finished
// current point: 545, 176
// min: 0
// max: 0.743143
// bin width: 0.0371572
// avg of 0.524662
// point interpolation finished
// current point: 675, 239
// min: 0
// max: 0.484104
// bin width: 0.0242052
// avg of 0.28307
// point interpolation finished
// current point: 592, 227
// min: 0
// max: 0.56132
// bin width: 0.028066
// avg of 0.428482
// point interpolation finished
// current point: 562, 262
// min: 0
// max: 0.514295
// bin width: 0.0257148
// avg of 0.425805
// point interpolation finished
// current point: 536, 115
// min: 0
// max: 0.49393
// bin width: 0.0246965
// avg of 0.255913
// point interpolation finished
// current point: 584, 173
// min: 0
// max: 0.647265
// bin width: 0.0323633
// avg of 0.569042
// point interpolation finished
// current point: 721, 90
// min: 0
// max: 0.875396
// bin width: 0.0437698
// avg of 0.515462
// point interpolation finished
// current point: 625, 197
// min: 0
// max: 0.778932
// bin width: 0.0389466
// avg of 0.248947
// point interpolation finished
// current point: 553, 94
// min: 0
// max: 0.460467
// bin width: 0.0230233
// avg of 0.398359
// point interpolation finished
// current point: 641, 258
// min: 0
// max: 0.502321
// bin width: 0.0251161
// avg of 0.398908
// point interpolation finished
// current point: 655, 288
// min: 0
// max: 0.780993
// bin width: 0.0390497
// avg of 0.40537
// point interpolation finished
// current point: 741, 169
// min: 0
// max: 0.886258
// bin width: 0.0443129
// avg of 0
// point interpolation finished
// current point: 696, 268
// min: 0
// max: 0.45247
// bin width: 0.0226235
// avg of 0.370989
// point interpolation finished
// current point: 490, 205
// min: 0
// max: 0.795302
// bin width: 0.0397651
// avg of 0.418878
// point interpolation finished
// current point: 371, 181
// min: 0
// max: 0.709852
// bin width: 0.0354926
// avg of 0.498568
// point interpolation finished
// current point: 501, 205
// min: 0
// max: 0.678396
// bin width: 0.0339198
// avg of 0.505538
// point interpolation finished
// current point: 562, 261
// min: 0
// max: 0.514295
// bin width: 0.0257148
// avg of 0.425801
// point interpolation finished
// current point: 397, 185
// min: 0
// max: 0.793708
// bin width: 0.0396854
// avg of 0.265693
// point interpolation finished
// current point: 513, 290
// min: 0
// max: 0.603297
// bin width: 0.0301648
// avg of 0
// point interpolation finished
// current point: 590, 126
// min: 0
// max: 0.591354
// bin width: 0.0295677
// avg of 0.34708
// point interpolation finished
// current point: 286, 114
// min: 0
// max: 0.758136
// bin width: 0.0379068
// avg of 0.149313
// point interpolation finished
// current point: 511, 239
// min: 0
// max: 0.437658
// bin width: 0.0218829
// avg of 0.317483
// point interpolation finished
// current point: 542, 181
// min: 0
// max: 0.720649
// bin width: 0.0360324
// avg of 0.517601
// point interpolation finished
// current point: 520, 101
// min: 0
// max: 0.469412
// bin width: 0.0234706
// avg of 0.321686
// point interpolation finished
// current point: 674, 221
// min: 0
// max: 0.551357
// bin width: 0.0275678
// avg of 0.282105
// point interpolation finished
// current point: 552, 175
// min: 0
// max: 0.743143
// bin width: 0.0371572
// avg of 0.526424
// point interpolation finished
// current point: 507, 145
// min: 0
// max: 0.554166
// bin width: 0.0277083
// avg of 0.212373
// point interpolation finished
// current point: 658, 77
// min: 0
// max: 0.446773
// bin width: 0.0223386
// avg of 8.00627e-20
// point interpolation finished
// current point: 532, 205
// min: 0
// max: 0.573966
// bin width: 0.0286983
// avg of 0.466916
// point interpolation finished
// current point: 687, 77
// min: 0
// max: 0.625797
// bin width: 0.0312898
// avg of 1.93126e-15
// point interpolation finished
// current point: 584, 85
// min: 0
// max: 74.3553
// bin width: 3.71776
// avg of 0.195113
// point interpolation finished
// current point: 690, 274
// min: 0
// max: 0.572766
// bin width: 0.0286383
// avg of 0.442293
// point interpolation finished
// current point: 446, 145
// min: 0
// max: 0.716688
// bin width: 0.0358344
// avg of 0.165244
// point interpolation finished
// current point: 661, 247
// min: 0
// max: 0.524372
// bin width: 0.0262186
// avg of 0.26932
// point interpolation finished
// current point: 704, 268
// min: 0
// max: 0.45247
// bin width: 0.0226235
// avg of 0.369456
// point interpolation finished
// current point: 553, 114
// min: 0
// max: 0.530419
// bin width: 0.026521
// avg of 0.258357
// point interpolation finished
// current point: 537, 213
// min: 0
// max: 0.492558
// bin width: 0.0246279
// avg of 0.491055
// point interpolation finished
// current point: 541, 196
// min: 0
// max: 0.631534
// bin width: 0.0315767
// avg of 0.493953
// point interpolation finished
// current point: 513, 174
// min: 0
// max: 0.677615
// bin width: 0.0338808
// avg of 0.582145
// point interpolation finished
// current point: 583, 193
// min: 0
// max: 0.698142
// bin width: 0.0349071
// avg of 0.323839
// point interpolation finished
// current point: 702, 254
// min: 0
// max: 0.516893
// bin width: 0.0258446
// avg of 0.43079
// point interpolation finished
// current point: 569, 95
// min: 0
// max: 0.571017
// bin width: 0.0285508
// avg of 0.406584
// point interpolation finished
// current point: 686, 130
// min: 0
// max: 0.453818
// bin width: 0.0226909
// avg of 0.232065
// point interpolation finished
// current point: 634, 167
// min: 0
// max: 0.555216
// bin width: 0.0277608
// avg of 0.357159
// point interpolation finished
// current point: 462, 252
// min: 0
// max: 0.632282
// bin width: 0.0316141
// avg of 0.174886
// point interpolation finished
// current point: 602, 132
// min: 0
// max: 0.504862
// bin width: 0.0252431
// avg of 0.411643
// point interpolation finished
// current point: 531, 82
// min: 0
// max: 0.520683
// bin width: 0.0260341
// avg of 0
// point interpolation finished
// current point: 570, 288
// min: 0
// max: 0.568498
// bin width: 0.0284249
// avg of 0.357228
// point interpolation finished
// current point: 728, 277
// min: 0
// max: 0.675902
// bin width: 0.0337951
// avg of 0.335543
// point interpolation finished
// current point: 501, 213
// min: 0
// max: 0.678396
// bin width: 0.0339198
// avg of 0.505046
// point interpolation finished
// current point: 574, 53
// min: -1.52146e+38
// max: 3.24634e+38
// bin width: inf
// avg of inf
// point interpolation finished
// current point: 680, 62
// min: 0
// max: 0.697343
// bin width: 0.0348672
// avg of 2.47764e-11
// point interpolation finished
// current point: 597, 187
// min: 0
// max: 0.591339
// bin width: 0.029567
// avg of 0.546669
// point interpolation finished
// current point: 619, 157
// min: 0
// max: 0.571897
// bin width: 0.0285949
// avg of 0.402029
// point interpolation finished
// current point: 647, 230
// min: 0
// max: 0.574108
// bin width: 0.0287054
// avg of 0.254478
// point interpolation finished
// current point: 684, 228
// min: 0
// max: 0.512731
// bin width: 0.0256366
// avg of 0.269576
// point interpolation finished
// current point: 557, 157
// min: 0
// max: 0.627156
// bin width: 0.0313578
// avg of 0.447854
// point interpolation finished
// current point: 522, 69
// min: -1.93704e-16
// max: 8.05186e+37
// bin width: 4.02593e+36
// avg of nan
// point interpolation finished
// interpolation finished in: 1.02833s
