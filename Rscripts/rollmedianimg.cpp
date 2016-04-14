#include <Rcpp.h>
using namespace Rcpp;

// [[Rcpp::export]]
NumericVector rollmedianimgC(SEXP obj, IntegerVector dim, int n){
  NumericVector data(obj); 
  NumericVector res(dim[0]*dim[1]*dim[2]);
  int fr(dim[2]), px(dim[0]*dim[1]);
  NumericVector tmp(n);
  double median;
  
  for (int j = 0; j < fr-n+1; j++) {
    for (int i = 0; i < px; i++) {
      for (int k = 0; k < n; k++) {
        tmp[k] = data[i + (j+k)*px];
      }
      std::sort(tmp.begin(), tmp.end());
      median = tmp[n / 2];
      res[i + j*px] = median;
    }
  }
  
  res.attr("dim") = dim;
  return res;
}

