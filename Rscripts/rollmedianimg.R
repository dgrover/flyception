rollmedianimg <- function(obj, n){
  require(Rcpp)
  sourceCpp(paste0(rdir, "rollmedianimg.cpp"))
  rollmedianimgC(obj, dim(obj), n)/n
}
