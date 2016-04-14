ssweep <- function(obj, ref, op){
  require(Rcpp)
  sourceCpp(paste0(rdir, "sweepC.cpp"))
  if(op == "-") {
    sweepC(obj, ref, dim(obj), 1)
  } else if(op == "*") {
    sweepC(obj, ref, dim(obj), 2)
  }
}