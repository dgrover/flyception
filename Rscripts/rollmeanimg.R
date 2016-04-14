rollmeanimg <- function(obj, n){
  require(Rcpp)
  sourceCpp(paste0(rdir, "rollmeanimg.cpp"))
  rollmeanimgC(obj, dim(obj), n)/n
}