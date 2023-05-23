#include "MathVector.h"
#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

MathVector::MathVector(int numElements) : numElements(numElements) {
  elements.resize(numElements);
}

bool MathVector::readElements(string str) {
  stringstream ss(str);
  string item;
  int i = 0;
  while (getline(ss, item, ',')) {
    try {
      if (i >= numElements) {
        cerr << "Unable to read line [" << str << "] (more than " << numElements << " elements)" << endl;
        return false;
      }
      elements[i] = stod(item);
      i++;
    } catch (invalid_argument&) {
      cerr << "Unable to read element " << i+1 << " of [" << str << "] (expected " << numElements << " elements)" << endl;
      return false;
    } catch (out_of_range&) {
      cerr << "Unable to read element " << i+1 << " of [" << str << "] (out of range)" << endl;
      return false;
    }
  }
  if (i < numElements) {
    cerr << "Unable to read element " << i+1 << " of [" << str << "] (expected " << numElements << " elements)" << endl;
    return false;
  }
  return true;
}

string MathVector::toString() {
  ostringstream ss;
  ss << fixed << setprecision(3);
  for (int i = 0; i < numElements; i++) {
    ss << elements[i];
    if (i < numElements - 1) {
      ss << ",";
    }
  }
  return ss.str();
}