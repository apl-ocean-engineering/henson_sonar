

// http://blog.marcinchwedczuk.pl/iterative-algorithm-for-drawing-hilbert-curve

#include <array>
#include <iostream>
using namespace std;

// for fixing openCV y axis
int fixCoord(int val, int max) {
  return max - val;
}

// returns last N bits of given value
int last2bits(int val) {
  return (val & 3);
}

array<int, 2> hIndexToXY(int hindex, int n) {
  array<array<int, 2>, 4> positions = {{{0,0}, {0,1}, {1,1}, {1,0}}};
  int tmpindex = last2bits(hindex);
  array<int, 2> tmp = {positions[tmpindex]};

  // cout << "temp: " << tmp[0] << tmp[1] << endl;

  // hindex = (unsigned int) hindex;
  // hindex = hindex >> 2;
  hindex = (int) hindex / 4;

  int x = tmp[0];
  int y = tmp[1];

  int i2;
  for (int i = 4; i <= n; i *= 2) {
    i2 = i / 2;

    switch(last2bits(hindex)) {
      case 0: {
        int swap0 = x;
        x = y;
        y = swap0;
        break;
      }


      case 1: {
        x = x;
        y = y + i2;
        break;
      }

      case 2: {
        x = x + i2;
        y = y + i2;
        break;
      }

      case 3: {
        int swap3 = y;
        y = (i2 - 1) - x;
        x = (i2 - 1) - swap3;
        x = x + i2;
        break;
      }
    }

    // hindex = (unsigned int) hindex;
    // hindex = hindex >> 2;
    hindex = (int) hindex / 4;
  }

  array<int, 2> result = {x, y};
  return result;
}

int main() {
  int n = 4;
  array<int, 2> curPoint;
  for(int i = 0; i < n * n; i++) {
    curPoint = hIndexToXY(i, n);
    cout << curPoint[0] << ", " << curPoint[1] << endl;
  }
}
