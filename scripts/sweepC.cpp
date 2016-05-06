#include <Rcpp.h>
using namespace Rcpp;

// [[Rcpp::export]]
NumericVector sweepC(SEXP obj, SEXP m, IntegerVector dim, int op){
  NumericVector data(obj), ref(m);
  NumericVector res(dim[0]*dim[1]*dim[2]);
  int fr(dim[2]), px(dim[0]*dim[1]);
  
  if(op == 1){
    for (int j = 0; j < fr; j++) {
      for (int i = 0; i < px; i++) {
        res[i + j*px] = data[i + j*px] - ref[i];
      }
    }
  }
  
  if(op == 2){
    for (int j = 0; j < fr; j++) {
      for (int i = 0; i < px; i++) {
        res[i + j*px] = data[i + j*px] * ref[i];
      }
    }
  }
  
  res.attr("dim") = dim;
  return res;
}
