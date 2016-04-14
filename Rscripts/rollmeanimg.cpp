#include <Rcpp.h>
using namespace Rcpp;

// [[Rcpp::export]]
NumericVector rollmeanimgC(SEXP obj, IntegerVector dim, int n){
  NumericVector data(obj); 
  NumericVector res(dim[0]*dim[1]*dim[2]);
  int fr(dim[2]), px(dim[0]*dim[1]);
  
  for (int j = 0; j < fr-n+1; j++) {
    for (int i = 0; i < px; i++) {
      for (int k = 0; k < n; k++) {
        if(k==0){
          res[i + j*px] = data[i + (j+k)*px];
        }
        res[i + j*px] = res[i + j*px] + data[i + (j+k)*px];
      }
    }
  }
  
  res.attr("dim") = dim;
  return res;
}
