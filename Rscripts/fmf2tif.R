fmf2tif <- function(filename, skip=0){
  source("C:/Users/GreenspanLab/Dropbox/R/readFMF2.R")
  require(EBImage)
  print("Converting an fmf file to tif...")
  fmfimg <- readFMF(file=filename, skip=skip)
  prefix <- substr(filename, 1, nchar(filename) - 4)
  writeImage(fmfimg/255, paste0(prefix, ".tif"))
  rm(fmfimg)
}